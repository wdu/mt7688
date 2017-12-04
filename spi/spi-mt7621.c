/*
 * spi-mt7621.c -- MediaTek MT7621 SPI controller driver
 *
 * Copyright (C) 2011 Sergiy <piratfm@gmail.com>
 * Copyright (C) 2011-2013 Gabor Juhos <juhosg@openwrt.org>
 * Copyright (C) 2014-2015 Felix Fietkau <nbd@nbd.name>
 * Copyright (C) 2017 Wim Dumon <wim@emweb.be>
 *
 * Some parts are based on spi-orion.c:
 *   Author: Shadi Ammouri <shadi@marvell.com>
 *   Copyright (C) 2007-2008 Marvell Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/swab.h>
#include <linux/gpio.h>

#include <ralink_regs.h>

#undef dev_dbg
//#define dev_dbg dev_info
#define dev_dbg(...)

#define SPI_BPW_MASK(bits) BIT((bits) - 1)

#define DRIVER_NAME			"spi-mt7621"
/* in msec */
#define RALINK_SPI_WAIT_TIMEOUT 8

/* SPISTAT register bit field */
#define SPISTAT_BUSY			BIT(0)

#define MT7621_SPI_TRANS	0x00
#define SPITRANS_BUSY		BIT(16)

#define MT7621_SPI_OPCODE	0x04
#define MT7621_SPI_DATA0	0x08
#define MT7621_SPI_DATA1	0x0C
#define MT7621_SPI_DATA2	0x10
#define MT7621_SPI_DATA3	0x14
#define MT7621_SPI_DATA4	0x18
#define MT7621_SPI_DATA5	0x1C
#define MT7621_SPI_DATA6	0x20
#define MT7621_SPI_DATA7	0x24
#define SPI_CTL_TX_RX_CNT_MASK	0xff
#define SPI_CTL_START		BIT(8)

#define MT7621_SPI_POLAR	0x38
#define MT7621_SPI_MASTER	0x28
#define MT7621_SPI_MOREBUF	0x2c
#define MT7621_SPI_SPACE	0x3c

#define MT7621_CPHA		BIT(5)
#define MT7621_CPOL		BIT(4)
#define MT7621_LSB_FIRST	BIT(3)

#define RT2880_SPI_MODE_BITS	(SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST | SPI_CS_HIGH)

struct mt7621_spi;

struct mt7621_spi {
	struct spi_master	*master;
	void __iomem		*base;
	unsigned int		sys_freq;
	unsigned int		speed;
	struct clk		*clk;
	spinlock_t		lock;

	struct mt7621_spi_ops	*ops;
};

static inline struct mt7621_spi *spidev_to_mt7621_spi(struct spi_device *spi)
{
	return spi_master_get_devdata(spi->master);
}

static inline u32 mt7621_spi_read(struct mt7621_spi *rs, u32 reg)
{
	return ioread32(rs->base + reg);
}

static inline void mt7621_spi_write(struct mt7621_spi *rs, u32 reg, u32 val)
{
	iowrite32(val, rs->base + reg);
}

static void mt7621_spi_reg_dump(struct mt7621_spi *rs)
{
	dev_dbg(&rs->master->dev, "SPI_OP_ADDR: %08x\n", mt7621_spi_read(rs, MT7621_SPI_OPCODE));
	dev_dbg(&rs->master->dev, "SPI_DIDO: %08x %08x %08x %08x\n",
		mt7621_spi_read(rs, MT7621_SPI_DATA0),
		mt7621_spi_read(rs, MT7621_SPI_DATA1),
		mt7621_spi_read(rs, MT7621_SPI_DATA2),
		mt7621_spi_read(rs, MT7621_SPI_DATA3));
	dev_dbg(&rs->master->dev, "SPI_DIDO: %08x %08x %08x %08x\n",
		mt7621_spi_read(rs, MT7621_SPI_DATA4),
		mt7621_spi_read(rs, MT7621_SPI_DATA5),
		mt7621_spi_read(rs, MT7621_SPI_DATA6),
		mt7621_spi_read(rs, MT7621_SPI_DATA7));
	dev_dbg(&rs->master->dev, "SPI_MASTER: %08x\n", mt7621_spi_read(rs, MT7621_SPI_MASTER));
	dev_dbg(&rs->master->dev, "SPI_MORE_BUF: %08x\n", mt7621_spi_read(rs, MT7621_SPI_MOREBUF));
}

static void mt7621_spi_set_cs(struct spi_device *spi, bool enable)
{
	struct mt7621_spi *rs = spidev_to_mt7621_spi(spi);


	u32 polar = 0;
	int cs = spi->chip_select;

	dev_dbg(&spi->dev, "set CS %d, enable %d\n", cs, (int)enable);

	if (cs > 7) {
		dev_err(&spi->dev, "CS > 7 not supported\n");
	} else {
		if (!enable)
			polar = BIT(cs);
	}
	mt7621_spi_write(rs, MT7621_SPI_POLAR, polar);
}

static int mt7621_spi_prepare(struct spi_device *spi, unsigned int speed)
{
	struct mt7621_spi *rs = spidev_to_mt7621_spi(spi);
	u32 rate;
	u32 reg;

	dev_dbg(&spi->dev, "speed:%u\n", speed);

	rate = DIV_ROUND_UP(rs->sys_freq, speed);
	// can't reach highest speed if max frequency happens to be odd
	// due to rounding errors
	if (speed == spi->master->max_speed_hz)
		rate = 2;

	dev_dbg(&spi->dev, "rate-1:%u\n", rate);

	if (rate > 4097)
		return -EINVAL;

	if (rate < 2)
		rate = 2;
	rs->speed = rs->sys_freq / rate;

	//rate = 4;

	//rate = 50; // FIXME!! remove me

	reg = mt7621_spi_read(rs, MT7621_SPI_MASTER);

	// CS strategy: preferably use GPIO CS exlusively, the system is
	// more flexible. We're not really using the HW mechanism to set
	// CS0/CS1 anyway, but are effectively setting then manually (like
	// GPIOs) by togling the CS polarity while they are in idle state.
	// To this purpose, CS7 is always selected as the CS to use, and
	// the polarity register is modified when CS needs to be toggled.
	reg |= 7 << 29;

	// always use more_buf mode
	reg |= 1 << 2;

	// full duplex does not work reliably
	reg &= ~(1 << 10);

	// set clock speed
	reg &= ~(0xfff << 16);
	reg |= (rate - 2) << 16;

	reg &= ~MT7621_LSB_FIRST;
	if (spi->mode & SPI_LSB_FIRST)
		reg |= MT7621_LSB_FIRST;

	reg &= ~(MT7621_CPHA | MT7621_CPOL);
	switch(spi->mode & (SPI_CPOL | SPI_CPHA)) {
		case SPI_MODE_0:
			break;
		case SPI_MODE_1:
			reg |= MT7621_CPHA;
			break;
		case SPI_MODE_2:
			reg |= MT7621_CPOL;
			break;
		case SPI_MODE_3:
			reg |= MT7621_CPOL | MT7621_CPHA;
			break;
	}
	mt7621_spi_write(rs, MT7621_SPI_MASTER, reg);

	return 0;
}

static inline int mt7621_spi_wait_till_ready(struct spi_device *spi)
{
	struct mt7621_spi *rs = spidev_to_mt7621_spi(spi);

	unsigned long deadline = jiffies + msecs_to_jiffies(RALINK_SPI_WAIT_TIMEOUT);
	while (time_before(jiffies, deadline)) {
		u32 status;
		status = mt7621_spi_read(rs, MT7621_SPI_TRANS);
		if ((status & SPITRANS_BUSY) == 0) {
			return 0;
		}
		cpu_relax();
	}

	return -ETIMEDOUT;
}


// returns amount of bytes transfered
static int mt7621_spi_transfer_chunk(struct spi_master *master,
				   struct spi_device *spi,
				   struct spi_transfer *t,
				   int offset)
{
	u32 data[9] = {0};
	u32 val;
	int i;
	struct mt7621_spi *rs = spi_master_get_devdata(master);
	int len = t->len - offset;

	dev_dbg(&spi->dev, "chunk: offset=%d, len=%d\n", offset, len);

	if (t->rx_buf) {
		// RX bytes are stored in DIDO registers. So max number of
		// bytes to RX is 8 * 4 = 32
		if (len > 32)
			len = 32;
		val = (len * 8) << 12;
		mt7621_spi_write(rs, MT7621_SPI_MOREBUF, val);
		dev_dbg(&spi->dev, "Wrote %08x to MOREBUF\n", val);
	}
	if (t->tx_buf) {
		const unsigned char *buf = t->tx_buf;
		// First 4 bytes to TX are stored in opcode register,
		// rest of TX bytes are stored in DIDO registers. So
		// max number of bytes to TX is (1 + 8) * 4 = 36.
		if (len > 36)
			len = 36;

		val = (min_t(int, len, 4) * 8) << 24;
		if (len > 4)
			val |= (len - 4) * 8;
		mt7621_spi_write(rs, MT7621_SPI_MOREBUF, val);
		dev_dbg(&spi->dev, "Wrote %08x to MOREBUF\n", val);

		dev_dbg(&spi->dev, "copying TX buf (%02x)\n", (int)buf[offset]);
		// FIXME: silly copy?
		for (i = 0; i < len; i++)
			data[i / 4] |= buf[i + offset] << (8 * (i & 3));

		data[0] = swab32(data[0]);
		if (len < 4)
			data[0] >>= (4 - len) * 8;

		for (i = 0; i < 9; i++)
			mt7621_spi_write(rs, MT7621_SPI_OPCODE + i * 4, data[i]);
	}


	dev_dbg(&spi->dev, "initializing transfer\n");
	mt7621_spi_reg_dump(rs);
	val = mt7621_spi_read(rs, MT7621_SPI_TRANS);
	val |= SPI_CTL_START;
	mt7621_spi_write(rs, MT7621_SPI_TRANS, val);

	// Ideally use an interrupt here iso busy waiting, if clock is slow
	mt7621_spi_wait_till_ready(spi);
	mt7621_spi_reg_dump(rs);

	if (t->rx_buf) {
		unsigned char *buf = t->rx_buf;
		dev_dbg(&spi->dev, "writing RX buf\n");
		for (i = 0; i < 9; i++)
			data[i] = mt7621_spi_read(rs, MT7621_SPI_DATA0 + i * 4);
		for (i = 0; i < len; i++)
			buf[i + offset] = data[i / 4] >> (8 * (i & 3));
	}
	return len;
}

static int mt7621_spi_transfer_half_duplex(struct spi_master *master,
					   struct spi_device *spi,
					   struct spi_transfer *t)
{
	int status = 0;
	int offset = 0;
	unsigned int speed = spi->max_speed_hz;

	mt7621_spi_wait_till_ready(spi);

	if (t->speed_hz < speed)
		speed = t->speed_hz;

	if (t->rx_buf && t->tx_buf) {
		// MT7688 has a HW bug, causing the second bit of the first RX
		// byte of a full-duplex SPI transfer to be corrupted in
		// some cases
		dev_err(&spi->dev, "full duplex not supported\n");
		status = -EIO;
		goto msg_done;
	}

	if (mt7621_spi_prepare(spi, speed)) {
		dev_dbg(&spi->dev, "mt7621_spi_prepare failed\n");
		status = -EIO;
		goto msg_done;
	}

	while (offset < t->len)
		offset += mt7621_spi_transfer_chunk(master, spi, t, offset);

msg_done:
	spi_finalize_current_transfer(master);

	return status;
}


static int mt7621_spi_transfer_one(struct spi_master *master,
				   struct spi_device *spi,
				   struct spi_transfer *t)
{
	return mt7621_spi_transfer_half_duplex(master, spi, t);
}

static int mt7621_spi_setup(struct spi_device *spi)
{
	int ret = 0;
	if (spi->cs_gpio != -ENOENT) {
		ret = gpio_is_valid(spi->cs_gpio);
		if (!ret) {
			dev_err(&spi->dev, "gpio cs %d not valid\n",
				spi->cs_gpio);
			ret = -ENOENT;
			goto exit;
		}
		ret = gpio_request(spi->cs_gpio, DRIVER_NAME);
		if (ret) {
			dev_err(&spi->dev, "failed to request gpio cs %d\n",
				spi->cs_gpio);
			goto exit;
		}
		ret = gpio_direction_output(spi->cs_gpio,
					    !(spi->mode & SPI_CS_HIGH));
		if (ret) {
			dev_err(&spi->dev,
				"Failed to set gpio cs %d as output\n",
				spi->cs_gpio);
			goto exit;
		}
	}
exit:
	return ret;

}

static int mt7621_spi_cleanup(struct spi_device *spi)
{
	if(gpio_is_valid(spi->cs_gpio)) {
		gpio_free(spi->cs_gpio);
	}
}

static const struct of_device_id mt7621_spi_match[] = {
	{ .compatible = "ralink,mt7621-spi" },
	{},
};
MODULE_DEVICE_TABLE(of, mt7621_spi_match);

static int mt7621_spi_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct spi_master *master;
	struct mt7621_spi *rs;
	unsigned long flags;
	void __iomem *base;
	struct resource *r;
	int status = 0;
	struct clk *clk;
	int sys_freq;
	struct mt7621_spi_ops *ops;

	match = of_match_device(mt7621_spi_match, &pdev->dev);
	if (!match)
		return -EINVAL;
	ops = (struct mt7621_spi_ops *)match->data;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(base))
		return PTR_ERR(base);

	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "unable to get SYS clock, err=%d\n",
			status);
		return PTR_ERR(clk);
	}

	status = clk_prepare_enable(clk);
	if (status)
		return status;
	sys_freq = clk_get_rate(clk);

	master = spi_alloc_master(&pdev->dev, sizeof(*rs));
	if (master == NULL) {
		dev_info(&pdev->dev, "master allocation failed\n");
		return -ENOMEM;
	}

	master->mode_bits = RT2880_SPI_MODE_BITS;

	master->setup = mt7621_spi_setup;
	master->transfer_one = mt7621_spi_transfer_one;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->dev.of_node = pdev->dev.of_node;
	master->num_chipselect = 64;
	master->flags = SPI_MASTER_HALF_DUPLEX;
	master->set_cs = mt7621_spi_set_cs;
	master->max_speed_hz = sys_freq / 2;
	master->min_speed_hz = sys_freq / 4097;

	dev_set_drvdata(&pdev->dev, master);

	rs = spi_master_get_devdata(master);
	rs->base = base;
	rs->clk = clk;
	rs->master = master;
	rs->sys_freq = sys_freq;
	rs->ops = ops;
	dev_info(&pdev->dev, "sys_freq: %u\n", rs->sys_freq);
	spin_lock_irqsave(&rs->lock, flags);

	device_reset(&pdev->dev);

	return spi_register_master(master);
}

static int mt7621_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master;
	struct mt7621_spi *rs;

	master = dev_get_drvdata(&pdev->dev);
	rs = spi_master_get_devdata(master);

	clk_disable(rs->clk);
	spi_unregister_master(master);

	return 0;
}

MODULE_ALIAS("platform:" DRIVER_NAME);

static struct platform_driver mt7621_spi_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mt7621_spi_match,
	},
	.probe = mt7621_spi_probe,
	.remove = mt7621_spi_remove,
};

module_platform_driver(mt7621_spi_driver);

MODULE_DESCRIPTION("MT7621 SPI driver");
MODULE_AUTHOR("Felix Fietkau <nbd@nbd.name>");
MODULE_LICENSE("GPL");
