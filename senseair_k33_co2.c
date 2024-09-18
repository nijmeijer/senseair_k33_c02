// SPDX-License-Identifier: GPL-2.0
/*
 * Senseair K33 CO2 sensor driver.
 *
 * Copyright (C) 2024 Alex Nijmeijer
 *
 * List of features not yet supported by the driver:
 * - calibration
 */

#include <linux/bitops.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/time64.h>

#include <linux/iio/iio.h>

#define DRIVER_NAME "senseair_K33_co2"

#define SENSEAIR_K33_ERROR_STATUS_REG	 	        0x1E
#define SENSEAIR_K33_CO2_FILTERED_COMP_REG		0x08
#define SENSEAIR_K33_CHIP_TEMPERATURE_REG		0x12
#define SENSEAIR_K33_RH_REG	                 	0x14
#define SENSEAIR_K33_READ_RAM_COMMAND                 	0x20

struct SENSEAIR_K33_dev {
	struct i2c_client *client;
	struct regmap *regmap;
	/* Protects access to IIO attributes. */
	struct mutex lock;
	bool ignore_nak;
};

/* Custom regmap read/write operations: perform unlocked access to the i2c bus. */

static int SENSEAIR_K33_regmap_read(void *context, const void *reg_buf,
			       size_t reg_size, void *val_buf, size_t val_size)
{
	struct i2c_client *client = context;
	struct SENSEAIR_K33_dev *SENSEAIR_K33 = i2c_get_clientdata(client);


        //dev_err(&SENSEAIR_K33->client->dev, "regmap_read val_size: %d\n", val_size);


        char buf[5];
        struct i2c_msg msg_req = {
		.addr = client->addr,
		.flags = 0,
		.len = 4,
		.buf = (__u8 *) &buf,
	};
        struct i2c_msg msg_resp = {
		.addr = client->addr,
		.flags =  I2C_M_RD,
		.len = val_size+2,
		.buf = (__u8 *) &buf,
	};


	int ret;
        u8 count;
	u8 sum;

	if (reg_size != 1 || !val_size || val_size > 4)
	  return -EINVAL;


        // Send Request command
        buf[0]=SENSEAIR_K33_READ_RAM_COMMAND | (val_size & 0x0F);                 // Read RAM command + requested number of bytes
        buf[1]=0;                    // ram_address high
        buf[2]= ((u8 *)reg_buf)[0];  // ram_address low
        sum = 0;
        count = 0;
        while (count<3) {
  	  //dev_err(&SENSEAIR_K33->client->dev, "regmap_request: sending %d\n", buf[count]);
          sum += buf[count] & 0xFF;
          count++;
         }
        buf[3]=sum ;                   // checksum

        ret = __i2c_transfer(SENSEAIR_K33->client->adapter, &msg_req, 1);

        // Twait
	usleep_range(18000, 21000);

        // Receive response
        ret = __i2c_transfer(SENSEAIR_K33->client->adapter, &msg_resp, 1);
        count = 0;
        sum = 0;
        while (count<val_size+1) {
  	  //dev_err(&SENSEAIR_K33->client->dev, "regmap_received: %d\n", buf[count]);
          sum += buf[count] & 0xFF;
          count++;
         }
         if (sum != buf[val_size+1]) {
    	    dev_err(&SENSEAIR_K33->client->dev, "Checksum error. Received %d, expected %d \n", buf[val_size+1], sum);
            ret = -EIO;
         }


	if (ret < 0)
		return ret;

	memcpy(val_buf, &buf[1], val_size); 

	return 0;
}


static int SENSEAIR_K33_regmap_write(void *context, const void *val_buf, size_t count)
{
	struct i2c_client *client = context;
	struct SENSEAIR_K33_dev *SENSEAIR_K33 = i2c_get_clientdata(client);
	union i2c_smbus_data data;

	/* Discard reg address from values count. */
	if (!count)
		return -EINVAL;
	count--;

	memset(&data, 0, sizeof(data));
	data.block[0] = count;
	memcpy(&data.block[1], (u8 *)val_buf + 1, count);

	//dev_err(&SENSEAIR_K33->client->dev, "regmap_write: sending count %d \n", data.block[0]);
	//dev_err(&SENSEAIR_K33->client->dev, "regmap_write: sending %d \n", data.block[1]);
	//dev_err(&SENSEAIR_K33->client->dev, "regmap_write: sending %d \n", data.block[2]);
	//dev_err(&SENSEAIR_K33->client->dev, "regmap_write: sending %d \n", data.block[3]);
	//dev_err(&SENSEAIR_K33->client->dev, "regmap_write: sending %d \n", data.block[4]);


	__i2c_smbus_xfer(client->adapter, client->addr,
			 SENSEAIR_K33->ignore_nak ? I2C_M_IGNORE_NAK : 0,
			 I2C_SMBUS_WRITE, 0, I2C_SMBUS_BYTE_DATA, &data);

	usleep_range(500, 1500);

	return __i2c_smbus_xfer(client->adapter, client->addr, client->flags,
				I2C_SMBUS_WRITE, ((u8 *)val_buf)[0],
				I2C_SMBUS_I2C_BLOCK_DATA, &data);
}


/*
 * SENSEAIR_K33 i2c read/write operations: lock the i2c segment to avoid losing the
 * wake up session. Use custom regmap operations that perform unlocked access to
 * the i2c bus.
 */
static int SENSEAIR_K33_read_byte(struct SENSEAIR_K33_dev *SENSEAIR_K33, u8 reg)
{
	const struct i2c_client *client = SENSEAIR_K33->client;
	const struct device *dev = &client->dev;
	unsigned int val;
	int ret;

	i2c_lock_bus(client->adapter, I2C_LOCK_SEGMENT);
	ret = regmap_read(SENSEAIR_K33->regmap, reg, &val);
	i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);
	if (ret) {
		dev_err(dev, "Read byte failed: reg 0x%02x (%d)\n", reg, ret);
		return ret;
	}

	return val;
}

static int SENSEAIR_K33_read_word(struct SENSEAIR_K33_dev *SENSEAIR_K33, u8 reg, u16 *val)
{
	const struct i2c_client *client = SENSEAIR_K33->client;
	const struct device *dev = &client->dev;
	__be16 be_val;
	int ret;

	i2c_lock_bus(client->adapter, I2C_LOCK_SEGMENT);

	//dev_err(&SENSEAIR_K33->client->dev, "calling bulk_read: reg %d \n", reg);

	ret = regmap_bulk_read(SENSEAIR_K33->regmap, reg, &be_val, sizeof(be_val));

	//dev_err(&SENSEAIR_K33->client->dev, "called bulk_read: be_val %d    size %d\n", be_val, sizeof(be_val));


	i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);
	if (ret) {
		dev_err(dev, "Read word failed: reg 0x%02x (%d)\n", reg, ret);
		return ret;
	}

	*val = be16_to_cpu(be_val);

	return 0;
}

#if 0
static int SENSEAIR_K33_write_byte(struct SENSEAIR_K33_dev *SENSEAIR_K33, u8 reg, u8 val)
{
	const struct i2c_client *client = SENSEAIR_K33->client;
	const struct device *dev = &client->dev;
	int ret;

	i2c_lock_bus(client->adapter, I2C_LOCK_SEGMENT);

	dev_err(&SENSEAIR_K33->client->dev, "write byte reg=%d val=%d\n", reg, val);

	ret = regmap_write(SENSEAIR_K33->regmap, reg, val);
	i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);
	if (ret)
		dev_err(dev, "Write byte failed: reg 0x%02x (%d)\n", reg, ret);

	return ret;
}

static int SENSEAIR_K33_write_word(struct SENSEAIR_K33_dev *SENSEAIR_K33, u8 reg, u16 data)
{
	const struct i2c_client *client = SENSEAIR_K33->client;
	const struct device *dev = &client->dev;
	__be16 be_data = cpu_to_be16(data);
	int ret;

	i2c_lock_bus(client->adapter, I2C_LOCK_SEGMENT);
	ret = regmap_bulk_write(SENSEAIR_K33->regmap, reg, &be_data, sizeof(be_data));
	i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);
	if (ret)
		dev_err(dev, "Write word failed: reg 0x%02x (%d)\n", reg, ret);

	return ret;
}
#endif

 /* Enumerate and retrieve the chip error status. */
enum {
	SENSEAIR_K33_ERROR_FATAL,
	SENSEAIR_K33_ERROR_OFFSET_REGULATION_ERROR,
	SENSEAIR_K33_ERROR_HUM_TEMP_COMM_ERROR,
	SENSEAIR_K33_ERROR_RH_ERROR,
	SENSEAIR_K33_ERROR_DETECTOR_TEMP_OUT_OF_RANGE,
	SENSEAIR_K33_ERROR_CO2_OUT_OF_RANGE,
	SENSEAIR_K33_ERROR_MEMORY_ERROR,
	SENSEAIR_K33_ERROR_SPACE_TEMP_OUT_OF_RANGE,
};

static const char * const SENSEAIR_K33_error_statuses[] = {
	[SENSEAIR_K33_ERROR_FATAL] = "error_fatal ",
	[SENSEAIR_K33_ERROR_OFFSET_REGULATION_ERROR] = "error_i2c ",
	[SENSEAIR_K33_ERROR_HUM_TEMP_COMM_ERROR] = "humidity_temperature_sensor_communication_error ",
	[SENSEAIR_K33_ERROR_RH_ERROR] = "rh_error ",
	[SENSEAIR_K33_ERROR_DETECTOR_TEMP_OUT_OF_RANGE] = "temperature_out_of_range ",
	[SENSEAIR_K33_ERROR_CO2_OUT_OF_RANGE] = "co2_out_of_range ",
	[SENSEAIR_K33_ERROR_MEMORY_ERROR] = "error_memory ",
	[SENSEAIR_K33_ERROR_SPACE_TEMP_OUT_OF_RANGE] = "error_space_temp_out_of_range ",
};

static const struct iio_enum SENSEAIR_K33_error_statuses_enum = {
	.items = SENSEAIR_K33_error_statuses,
	.num_items = ARRAY_SIZE(SENSEAIR_K33_error_statuses),
};

static ssize_t SENSEAIR_K33_error_status_read(struct iio_dev *iiodev,
					 uintptr_t private,
					 const struct iio_chan_spec *chan,
					 char *buf)
{
	struct SENSEAIR_K33_dev *SENSEAIR_K33 = iio_priv(iiodev);
	unsigned long errors;
	ssize_t len = 0;
//	u16 value;
	int ret;
	u8 i;

	mutex_lock(&SENSEAIR_K33->lock);
	ret = SENSEAIR_K33_read_byte(SENSEAIR_K33, SENSEAIR_K33_ERROR_STATUS_REG);
        dev_err(&SENSEAIR_K33->client->dev, "Error word: 0x%02X\n", ret);

	if (ret) {
		mutex_unlock(&SENSEAIR_K33->lock);
		return ret;
	}

	errors = ret;
	for_each_set_bit(i, &errors, ARRAY_SIZE(SENSEAIR_K33_error_statuses))
		len += sysfs_emit_at(buf, len, "%s ", SENSEAIR_K33_error_statuses[i]);

	if (len)
		buf[len - 1] = '\n';

	mutex_unlock(&SENSEAIR_K33->lock);

	return len;
}

static const struct iio_chan_spec_ext_info SENSEAIR_K33_concentration_ext_info[] = {
	/* Error statuses. */
	{
		.name = "error_status",
		.read = SENSEAIR_K33_error_status_read,
		.shared = IIO_SHARED_BY_ALL,
	},
	{
		.name = "error_status_available",
		.shared = IIO_SHARED_BY_ALL,
		.read = iio_enum_available_read,
		.private = (uintptr_t)&SENSEAIR_K33_error_statuses_enum,
	},
	{}
};

static const struct iio_chan_spec SENSEAIR_K33_channels[] = {
	{
		.type = IIO_CONCENTRATION,
		.modified = 1,
		.channel2 = IIO_MOD_CO2,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.ext_info = SENSEAIR_K33_concentration_ext_info,
	},
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	},
	{
		.type = IIO_HUMIDITYRELATIVE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	},
};

static int SENSEAIR_K33_read_raw(struct iio_dev *iio_dev,
			    const struct iio_chan_spec *chan,
			    int *val, int *val2, long mask)
{
	struct SENSEAIR_K33_dev *SENSEAIR_K33 = iio_priv(iio_dev);
	u16 value;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_CONCENTRATION:
			mutex_lock(&SENSEAIR_K33->lock);

			ret = SENSEAIR_K33_read_word(SENSEAIR_K33, SENSEAIR_K33_CO2_FILTERED_COMP_REG,
						&value);
			mutex_unlock(&SENSEAIR_K33->lock);

			if (ret)
				return ret;

			*val = value;
			return IIO_VAL_INT;

		case IIO_TEMP:
			mutex_lock(&SENSEAIR_K33->lock);
			ret = SENSEAIR_K33_read_word(SENSEAIR_K33, SENSEAIR_K33_CHIP_TEMPERATURE_REG,
						&value);
			mutex_unlock(&SENSEAIR_K33->lock);

			if (ret)
				return ret;

			*val = value;
			return IIO_VAL_INT;

		case IIO_HUMIDITYRELATIVE:
			mutex_lock(&SENSEAIR_K33->lock);
			ret = SENSEAIR_K33_read_word(SENSEAIR_K33, SENSEAIR_K33_RH_REG,
						&value);
			mutex_unlock(&SENSEAIR_K33->lock);

			if (ret)
				return ret;

			*val = value;
			return IIO_VAL_INT;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_CONCENTRATION:
			/*
			 * 1 / 10^4 to comply with IIO scale for CO2
			 * (percentage). The chip CO2 reading range is [400 -
			 * 5000] ppm which corresponds to [0,004 - 0,5] %.
			 */
			*val = 1;
			*val2 = 10000;
			return IIO_VAL_FRACTIONAL;

		case IIO_TEMP:
			/* x10 to comply with IIO scale (millidegrees celsius). */
			*val = 10;
			return IIO_VAL_INT;

		case IIO_HUMIDITYRELATIVE:
			/* x10 to comply with IIO scale (millidegrees celsius). */   // TO BE CHECKED
			*val = 10;
			return IIO_VAL_INT;

		default:
			return -EINVAL;
		}

	default:
		return -EINVAL;
	}
}

static const struct iio_info SENSEAIR_K33_info = {
	.read_raw = SENSEAIR_K33_read_raw,
};

static const struct regmap_bus SENSEAIR_K33_regmap_bus = {
	.read = SENSEAIR_K33_regmap_read,
	.write = SENSEAIR_K33_regmap_write,
};

static const struct regmap_config SENSEAIR_K33_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int SENSEAIR_K33_probe(struct i2c_client *client)
{
	struct SENSEAIR_K33_dev *SENSEAIR_K33;
	struct iio_dev *iio_dev;
        int value,value1, value2, value3;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA |
						      I2C_FUNC_SMBUS_BLOCK_DATA)) {
		dev_err(&client->dev,
			"Adapter does not support required functionalities\n");
		return -EOPNOTSUPP;
	}

	iio_dev = devm_iio_device_alloc(&client->dev, sizeof(*SENSEAIR_K33));
	if (!iio_dev)
		return -ENOMEM;

	SENSEAIR_K33 = iio_priv(iio_dev);
	SENSEAIR_K33->client = client;
	mutex_init(&SENSEAIR_K33->lock);

	i2c_set_clientdata(client, SENSEAIR_K33);

	SENSEAIR_K33->regmap = devm_regmap_init(&client->dev, &SENSEAIR_K33_regmap_bus,
					   client, &SENSEAIR_K33_regmap_config);
	if (IS_ERR(SENSEAIR_K33->regmap)) {
		dev_err(&client->dev, "Failed to initialize regmap\n");
		return PTR_ERR(SENSEAIR_K33->regmap);
	}

	/*
	 * The chip nacks the wake up message. If the adapter does not support
	 * protocol mangling do not set the I2C_M_IGNORE_NAK flag at the expense
	 * of possible cruft in the logs.
	 */
	if (i2c_check_functionality(client->adapter, I2C_FUNC_PROTOCOL_MANGLING))
		SENSEAIR_K33->ignore_nak = true;

	iio_dev->info = &SENSEAIR_K33_info;
	iio_dev->name = DRIVER_NAME;
	iio_dev->channels = SENSEAIR_K33_channels;
	iio_dev->num_channels = ARRAY_SIZE(SENSEAIR_K33_channels);
	iio_dev->modes = INDIO_DIRECT_MODE;

        // read sensor type, serial, and firmware version
        mutex_lock(&SENSEAIR_K33->lock);
        value =SENSEAIR_K33_read_byte(SENSEAIR_K33, 0x2C);
        value1=SENSEAIR_K33_read_byte(SENSEAIR_K33, 0x2D);
        value2=SENSEAIR_K33_read_byte(SENSEAIR_K33, 0x2E);
	dev_info(&SENSEAIR_K33->client->dev, "K33 reports sensor type 0x%02X.%02X.%02X\n", value, value1,value2);

        value =SENSEAIR_K33_read_byte(SENSEAIR_K33, 0x28);
        value1=SENSEAIR_K33_read_byte(SENSEAIR_K33, 0x29);
        value2=SENSEAIR_K33_read_byte(SENSEAIR_K33, 0x2A);
        value3=SENSEAIR_K33_read_byte(SENSEAIR_K33, 0x2B);
	dev_info(&SENSEAIR_K33->client->dev, "K33 reports serial number 0x%02X.%02X.%02X.%02X \n", value, value1, value2, value3);

        value =SENSEAIR_K33_read_byte(SENSEAIR_K33, 0x62);
        value1=SENSEAIR_K33_read_byte(SENSEAIR_K33, 0x63);
        value2=SENSEAIR_K33_read_byte(SENSEAIR_K33, 0x64);
	dev_info(&SENSEAIR_K33->client->dev, "K33 reports firmware type 0x%02X, main_ver 0x%02X, sub_ver 0x%02X\n", value, value1, value2);
        mutex_unlock(&SENSEAIR_K33->lock);

	return devm_iio_device_register(&client->dev, iio_dev);
}

static const struct of_device_id SENSEAIR_K33_of_match[] = {
	{ .compatible = "senseair,senseair-k33-c02" },
	{}
};
MODULE_DEVICE_TABLE(of, SENSEAIR_K33_of_match);

static struct i2c_driver SENSEAIR_K33_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = SENSEAIR_K33_of_match,
	},
	.probe = SENSEAIR_K33_probe,
};
module_i2c_driver(SENSEAIR_K33_driver);

MODULE_AUTHOR("Alex Nijmeijer <alex.nijmeijer@neads.nl>");
MODULE_DESCRIPTION("Senseair K33 CO2 sensor IIO driver");
MODULE_LICENSE("GPL v2");
