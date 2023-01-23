// SPDX-License-Identifier: GPL-2.0
/*
 * DACx0508 Octal, 14-Bit Serial input Digital-to-Analog Converter
 *
 * Licensed under the GPL-2.
 */
/*
 * NOTES
 * https://www.kernel.org/doc/html/v5.10/driver-api/spi.html
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/regulator/consumer.h>

#define DACx0508_NUM_CH		8
#define DACx0508_READ_BIT	BIT(7)
#define DACx0508_DEV_REG	0x01
#define DACx0508_SYNC_REG	0x02 
#define DACx0508_CFG_REG  	0x03 
#define DACx0508_GAIN_REG  	0x04
#define DACx0508_TRIGG_REG  	0x05
#define DACx0508_BRDCAST_REG  	0x06
#define DACx0508_STATUS_REG  	0x07
#define DACx0508_CHAN_REG(ch)	(0x08 + ch)
#define DACx0508_CHAN_MASK(ch)  (BIT(0)) //TODO!!
#define DACx0508_REFDIV_MASK	BIT(8)
#define DACx0508_REFPWDN_MASK	BIT(8)
#define DACx0508_LDAC_MASK	BIT(4)

/**
 * struct dacx0508
 * @broadcast: true if feature is enabled
 * @powerdown: powerdown status per channel
 * @val: cached value for each output
 * @vref: ref. voltage
 * @buf: buffer for xfer
 */  
struct dacx0508 {
	struct spi_device *spi;
	//struct gpio_desc *special_gpiod;
	bool broadcast;
	bool powerdown[DACx0508_NUM_CH];
	uint16_t val[DACx0508_NUM_CH];
	int vref;
	struct mutex lock;
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	uint8_t buf[3] __aligned(IIO_DMA_MINALIGN);
};

/**
 * writes 2 bytes into given register
 */
static int dacx0508_write_reg(struct dacx0508 *priv, u8 reg, u16 data) 
{
	priv->buf[0] = reg & 0x0F;
	priv->buf[1] = (data & 0xFF00)>>8;
	priv->buf[2] = data & 0xFF;
	return spi_write(priv->spi, priv->buf, 3);
}

/**
 * reads 2 bytes from given register
 */
static int dacx0508_read_reg(struct dacx0508 *priv, u8 reg) 
{
	priv->buf[0] = reg & 0x0F;
	priv->buf[0] |= DACx0508_READ_BIT;
	priv->buf[1] &= 0x00;
	priv->buf[2] &= 0x00;
	return spi_write_then_read(priv->spi, priv->buf, 1, priv->buf+1, 2);
}

/**
 * powers up/down given channel
 */
static int dacx0508_chan_power(struct dacx0508 *priv, u8 chan, bool powerdown)
{
	u16 val = 0;
	int ret = dacx0508_read_reg(priv, DACx0508_CFG_REG);
	if (ret)
		return ret;
	val = 0;// = priv->buf[x] TODO a finir
	if (powerdown)
		val = priv->buf[2] | 0x01<<chan;
	//else //TODO a FINIR mask out
		//data = priv->buf[2] & (0x01<<chan;
	return dacx0508_write_reg(priv, DACx0508_CFG_REG, val); 
}

/**
 * sets up device to broadcast given value on x8 channels
 */
static int dacx0508_set_broadcast(struct dacx0508 *priv, bool broadcast)
{
	u16 val = 0;
	int ret = dacx0508_read_reg(priv, DACx0508_SYNC_REG);
	if (ret)
		return ret;
	if (broadcast)
		val |= 0xFF<<8;
	val |= priv->buf[2];
	return dacx0508_write_reg(priv, DACx0508_SYNC_REG, val);
}

/**
 * writes desired value to selected DAC channel
 */
static int dacx0508_write_chan(struct dacx0508 *priv, u8 chan, u16 val)
{
	if (priv->powerdown[chan]) 
		return 0;
	return dacx0508_write_reg(priv, DACx0508_CHAN_REG(chan), val); 
}

/**
 * read powerdown special feature
 */
static int dacx0508_read_powerdown(struct iio_dev *indio_dev,
				   uintptr_t private,
			 	   const struct iio_chan_spec *chan,
				   char *buf)
{
	struct dacx0508 *priv = iio_priv(indio_dev);
	
	return sysfs_emit(buf, "%d\n", priv->powerdown[chan->channel]);
}

/**
 * write powerdown special feature
 */
static int dacx0508_write_powerdown(struct iio_dev *indio_dev,
				    uintptr_t private,
			 	    const struct iio_chan_spec *chan,
				    const char *buf, size_t len)
{
	struct dacx0508 *priv = iio_priv(indio_dev);
	bool powerdown;
	int ret;

	ret = kstrtobool(buf, &powerdown);
	if (ret)
		return ret;
	
	if (priv->powerdown[chan->channel] == powerdown)
		return len; // already applied

	mutex_lock(&priv->lock);
	ret = dacx0508_chan_power(priv, chan->channel, powerdown);
	if (!ret) 
		priv->powerdown[chan->channel] = powerdown;
	mutex_unlock(&priv->lock);
	return ret ? ret : len;
}

/**
 * read broadcast special feature
 */
static int dacx0508_read_broadcast(struct iio_dev *indio_dev,
				   uintptr_t private,
			 	   const struct iio_chan_spec *chan,
				   char *buf)
{
	struct dacx0508 *priv = iio_priv(indio_dev);
	if (!priv->broadcast) 
		return -EINVAL;
	
	return sysfs_emit(buf, "%d\n", priv->val[0]);
}

/**
 * write broadcast special feature
 */
static int dacx0508_write_broadcast(struct iio_dev *indio_dev,
				    uintptr_t private,
			 	    const struct iio_chan_spec *chan,
				    const char *buf, size_t len)
{
	struct dacx0508 *priv = iio_priv(indio_dev);
	u16 val;
	int ret;
	
	ret = kstrtou16(buf, 10, &val);
	if (ret) 
		return ret;
	
	mutex_lock(&priv->lock);
	if (!priv->broadcast) { 
		ret = dacx0508_set_broadcast(priv, true);
		if (ret) {
			return ret;
		}
		priv->broadcast = true;
	}

	ret = dacx0508_write_reg(priv, DACx0508_BRDCAST_REG, val); 
	if (!ret) 
		priv->val[0] = val;
	mutex_unlock(&priv->lock);
	return ret ? ret : len;
}

/**
 * reads device description register
 */
static void dacx0508_read_dev_infos(struct dacx0508 *priv) 
{
	u8 res, num_ch, midscale;
	int ret = dacx0508_read_reg(priv, DACx0508_DEV_REG);
	if (ret < 0){
		printk(KERN_ERR "dacx0508: failed to read dev infos\n");
	} else {
		printk(KERN_INFO "dacx0508: dev infos:\n");
		res = (priv->buf[1] & 0x70) >> 4;
		num_ch = priv->buf[1] & 0x0F;
		midscale = (priv->buf[2] & 0x80) >> 7;
		printk(KERN_INFO "resolution: %d\n", res);
		printk(KERN_INFO "num_chan: %d\n", num_ch);
		printk(KERN_INFO "is_midscale: %d\n", midscale);
	}
}

/**
 * assert LDAC bit
 */
static int dacx0508_assert_ldac(struct dacx0508 *priv)
{
	return dacx0508_write_reg(priv, DACx0508_TRIGG_REG, DACx0508_LDAC_MASK);
}
/**
 * deassert LDAC bit
 */
static int dacx0508_deassert_ldac(struct dacx0508 *priv)
{
	return dacx0508_write_reg(priv, DACx0508_TRIGG_REG, 0x00);
}

static const struct iio_chan_spec_ext_info dacx0508_ext_infos[] = {
	{
		.name = "powerdown",
		.read = dacx0508_read_powerdown,
		.write = dacx0508_write_powerdown,
		.shared = IIO_SHARED_BY_DIR,
	},
	{
		.name = "broadcast",
		.read = dacx0508_read_broadcast,
		.write = dacx0508_write_broadcast,
		.shared = IIO_SHARED_BY_ALL,
	},
};

#define DACx0508_CHANNEL(chan, name) {				\
	.type = IIO_VOLTAGE,					\
	.channel = (chan),					\
	.indexed = true,					\
	.output = true,						\
	.datasheet_name = name,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				BIT(IIO_CHAN_INFO_SCALE),	\
	.ext_info = dacx0508_ext_infos,				\
}

static const struct iio_chan_spec dacx0508_channels[] = {
	DACx0508_CHANNEL(0, "OUT1"),
	DACx0508_CHANNEL(1, "OUT2"),
	DACx0508_CHANNEL(2, "OUT3"),
	DACx0508_CHANNEL(3, "OUT4"),
	DACx0508_CHANNEL(4, "OUT5"),
	DACx0508_CHANNEL(5, "OUT6"),
	DACx0508_CHANNEL(6, "OUT7"),
	DACx0508_CHANNEL(7, "OUT8"),
};

static int dacx0508_get_scale(struct dacx0508 *priv, u8 ch,
			      u16 *range)
{
	int ret, vref;
	u16 val = 0;
	
	mutex_lock(&priv->lock);
	ret = dacx0508_read_reg(priv, DACx0508_GAIN_REG);
	if (ret)
		goto err;
	mutex_unlock(&priv->lock);

	*range = priv->vref / 1000;
	if (val & DACx0508_CHAN_MASK(ch)) 
		*range *= 2;
	if (val & DACx0508_REFDIV_MASK) 
		*range /= 2;
	return ret;

err:
	mutex_unlock(&priv->lock);
	return ret;
}

static int dacx0508_read_raw(struct iio_dev *iio_dev,
			    const struct iio_chan_spec *chan,
			    int *val, int *val2, long mask)
{
	int ret;
	struct dacx0508 *priv;
	u8 ch = chan->channel;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		priv = iio_priv(iio_dev);
		*val = priv->val[ch];
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		mutex_lock(&priv->lock);
		ret = dacx0508_get_scale(priv, ch, val);
		*val2 = 14; //priv->spec->resolution;
		mutex_unlock(&priv->lock);
		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_ENABLE:
		mutex_lock(&priv->lock);
		ret = dacx0508_read_reg(priv, DACx0508_CFG_REG);
		mutex_unlock(&priv->lock);
		if (ret < 0)
			return ret;
		*val != ((tmp_val & DACx0508_MASK_CH_DAC_POWERDOWN(ch)) >>
			__ffs(DACx0508_MASK_CH_DAC_POWERDOWN(ch)));
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int dacx0508_write_raw(struct iio_dev *iio_dev,
			     const struct iio_chan_spec *chan,
			     int val, int val2, long mask)
{
	struct dacx0508 *priv = iio_priv(iio_dev);
	int ret;

	if (mask != IIO_CHAN_INFO_RAW) 
		return -EINVAL;
	//if (val < 0)
	//	return -EINVAL;
	if (val == priv->val[chan->channel]) // value already applied 
		return 0;

	mutex_lock(&priv->lock);
	
	if (priv->broadcast) {
		/*
 		 * disable broadcast
		 */
		ret = dacx0508_set_broadcast(priv, false);
		if (ret) 
			return -EINVAL;
		priv->broadcast = false;
	}

	ret = dacx0508_write_chan(priv, chan->channel, val);
	mutex_unlock(&priv->lock);

	return ret;
}

static const struct iio_info dacx0508_info = {
	.read_raw = dacx0508_read_raw,
	.write_raw = dacx0508_write_raw,
};

static int dacx0508_probe(struct spi_device *spi)
{
	struct iio_dev *iio_dev;
	struct dacx0508 *priv;
	int i, ret;
	struct regulator *vref_reg;

	iio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*priv));
	if (!iio_dev)
		return -ENOMEM;

	priv = iio_priv(iio_dev);

	priv->spi = spi;
	iio_dev->dev.parent = &spi->dev;
	iio_dev->info = &dacx0508_info;
	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->name = spi_get_device_id(spi)->name;
	iio_dev->channels = dacx0508_channels;
	iio_dev->num_channels = DACx0508_NUM_CH +1; // 1 channel for broadcast feature 
	spi_set_drvdata(spi, iio_dev);
	
	vref_reg = devm_regulator_get_optional(&spi->dev, "vref");
	if (IS_ERR(vref_reg)) {
		if (PTR_ERR(vref_reg) != -ENODEV)
			return dev_err_probe(&spi->dev, PTR_ERR(vref_reg),
					     "Failed to get vref regulator\n");
		vref_reg = NULL;
		/* internal reference */
		priv->vref = 4096; //TODO
	} else {
		ret = regulator_enable(vref_reg);
		if (ret) 
			return dev_err_probe(&spi->dev, ret,
					     "Failed to enable vref regulator\n");
		/*ret = devm_add_action_or_reset(dev, dacx0508_disable_regulator,
					       vref_reg);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to attach vref disabling action\n");
		*/
		priv->vref = ret / 1000;	
	}

	mutex_init(&priv->lock);
	
	dacx0508_read_dev_infos(priv);
/*
	*
	 * initialize output to 0 or midscale, depending on device 
	 *
	for (i = 0; i < ARRAY_SIZE(priv->cache); i++) {
		ret = dacx0508_cmd_single(priv, i, 0);
		if (ret) {
			goto err;
		}
	}
*/
	return devm_iio_device_register(&spi->dev, iio_dev);

err:
	mutex_destroy(&priv->lock);
	return ret;
}

static void dacx0508_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct dacx0508 *priv = iio_priv(indio_dev);
	iio_device_unregister(indio_dev);
	mutex_destroy(&priv->lock);
}

static const struct spi_device_id dacx0508_id[] = {
	{"ti-dacx0508"},
	{}
};
MODULE_DEVICE_TABLE(spi, dacx0508_id);

static const struct of_device_id dacx0508_of_match[] = {
	{ .compatible = "ti,dacx0508" },
	{ .compatible = "ti,dac70508" },
	{ .compatible = "ti,dac70508m" },
	{ },
};
MODULE_DEVICE_TABLE(of, dacx0508_of_match);

static struct spi_driver dacx0508_driver = {
	.driver = {
		   .name = "ti-dacx0508",
		   .of_match_table = dacx0508_of_match,
	},
	.probe = dacx0508_probe,
	.id_table = dacx0508_id,
	.remove = dacx0508_remove,
};
module_spi_driver(dacx0508_driver);

MODULE_AUTHOR("Guillaume W. Bres <guillaume.bres@bertin.group>");
MODULE_DESCRIPTION("Texas Instruments Octal 12/14/16 bit DACx0508 driver");
MODULE_LICENSE("GPL v2");
