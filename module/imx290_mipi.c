/*
 * Copyright (C) 2018 2Pi-AS, All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#define DEBUG_NO_SENSOR

#define MIN_FPS 25
#define MAX_FPS 60
#define DEFAULT_FPS 25

#define CAN_CHANGE_STANDBY(imx) \
	( ((imx)->on) && ((imx)->standby) )

enum imx290_inclk {
	imx290_inclk_37mhz = 0,
	imx290_inclk_74mhz,
	imx290_inclk_MAX,
};

enum imx290_bpp {
	imx290_bpp12 = 0,
	imx290_bpp10,
};

enum imx290_csi2_lanes {
	imx290_lanes_0 = 0,
	imx290_lanes_2,
	imx290_lanes_4,
};

enum imx290_gpio {
	imx290_en_reg_12 = 0,
	imx290_en_reg_18,
	imx290_en_reg_29,
	imx290_en_inclk,
	imx290_xclr,
	imx290_gpio_max,
};

static const char* imx290_gpio_ofname[] = {
		"en_reg_12",
		"en_reg_18",
		"en_reg_29",
		"en_inclk",
		"xclr",
};

enum imx290_mode {
	imx290_mode_MIN = 0,
	imx290_mode_720P = 1,
	imx290_mode_1080P = 2,
	imx290_mode_CROP = 3,
	imx290_mode_PG = 4,		/* Pattern generator / test mode */
	imx290_mode_MAX = 5,
	imx290_mode_INIT = 0xff, /*only for sensor init*/
};

enum imx290_frame_rate {
	imx290_fps_min = 0,
	imx290_25_fps,
	imx290_30_fps,
	imx290_60_fps,
	imx290_fps_max,
};

struct imx290_datafmt {
	u32	code;
	enum v4l2_colorspace		colorspace;
	enum imx290_bpp bpp;
};

struct reg_value {
	u16 addr;
	u8 val;
	u8 mask;
	u32 delay_us;
};

#define ARRAY(x) \
	x, ARRAY_SIZE(x)

struct imx290_reg_set {
	struct reg_value* reg;
	u32 reg_count;
};

struct imx290_mode_info {
	enum imx290_mode mode;
	u32 width;
	u32 height;
	struct reg_value *base;
	u32 base_size;
};

struct imx290 {
	struct v4l2_subdev		subdev;
	struct i2c_client *i2c_client;
	const struct imx290_datafmt	*fmt;
	struct v4l2_captureparm streamcap;

	int standby;
	int on;

	enum imx290_inclk inclk;
	enum imx290_bpp bpp;
	enum imx290_csi2_lanes csi2_lanes;

	enum imx290_frame_rate fps;
	enum imx290_mode mode;

	u8 i2c_addr;

	int gpio[imx290_gpio_max];

	struct clk *sensor_clk;

	void (*io_init)(void);
};
/*!
 * Maintains the information on the current state of the sesor.
 */
static struct imx290 imx290_data;

/**
 * Registers to be changed from their default after reset.
 */
static struct reg_value imx290_init_settings[] = {
	{0x300f, 0x00, 0, 0}, {0x3010, 0x21, 0, 0}, {0x3012, 0x64, 0, 0},
	{0x3016, 0x09, 0, 0}, {0x3070, 0x02, 0, 0}, {0x3071, 0x11, 0, 0},
	{0x309b, 0x10, 0 ,0}, {0x309c, 0x22, 0, 0}, {0x30a2, 0x02, 0, 0},
	{0x30a6, 0x20, 0, 0}, {0x30a8, 0x20, 0, 0}, {0x30aa, 0x20, 0, 0},
	{0x30ac, 0x20, 0, 0}, {0x30b0, 0x43, 0, 0}, {0x3119, 0x9e, 0, 0},
	{0x311c, 0x1e, 0, 0}, {0x311e, 0x08, 0, 0}, {0x3128, 0x05, 0, 0},
	{0x313d, 0x83, 0, 0}, {0x3150, 0x03, 0, 0}, {0x317e, 0x00, 0, 0},
	{0x32b8, 0x50, 0, 0}, {0x32b9, 0x10, 0, 0}, {0x32ba, 0x00, 0 ,0},
	{0x32bb, 0x04, 0, 0}, {0x32c8, 0x50, 0, 0}, {0x32c9, 0x10, 0, 0},
	{0x32ca, 0x00, 0, 0}, {0x32cb, 0x04, 0, 0}, {0x332c, 0xd3, 0, 0},
	{0x332d, 0x10, 0, 0}, {0x332e, 0x0d, 0, 0}, {0x3358, 0x06, 0, 0},
	{0x3359, 0xe1, 0, 0}, {0x335a, 0x11, 0, 0}, {0x3360, 0x1e, 0, 0},
	{0x3361, 0x61, 0, 0}, {0x3362, 0x10, 0, 0}, {0x33b0, 0x50, 0, 0},
	{0x33b2, 0x1a, 0, 0}, {0x33b3, 0x04, 0, 0},
};

static struct reg_value imx290_setting_bpp12[] = {
	{0x3005, 0x01, 0, 0}, {0x3441, 0x0c, 0, 0}, {0x3442, 0x0c, 0, 0},
	{0x3129, 0x00, 0, 0}, {0x317c, 0x00, 0, 0}, {0x31ec, 0x0e, 0, 0},
};

static struct reg_value imx290_setting_bpp10[] = {
	{0x3005, 0x00, 0, 0}, {0x3441, 0x0a, 0, 0}, {0x3442, 0x0a, 0, 0},
	{0x3129, 0x1d, 0, 0}, {0x317c, 0x12, 0, 0}, {0x31ec, 0x37, 0, 0},
};

static struct reg_value imx290_setting_mode720_25fps_base[] = {
		{0x3007, 0x10, 0, 0},  {0x3009, 0x02, 0, 0}, {0x3012, 0x64, 0, 0},
		{0x3013, 0x00, 0, 0}, {0x3018, 0xee, 0, 0}, {0x3019, 0x02, 0, 0},
		{0x301a, 0x00, 0, 0}, {0x301c, 0xf0, 0, 0}, {0x301d, 0x1e, 0, 0},
		{0x3405, 0x10, 0, 0}, {0x3414, 0x04, 0, 0}, {0x3418, 0xd9, 0, 0},
		{0x3419, 0x02, 0, 0}, {0x3446, 0x4f, 0, 0}, {0x3447, 0x00, 0, 0},
		{0x3448, 0x2f, 0, 0}, {0x3449, 0x00, 0, 0}, {0x344a, 0x17, 0, 0},
		{0x344b, 0x00, 0, 0}, {0x344c, 0x17, 0, 0}, {0x344d, 0x00, 0, 0},
		{0x344e, 0x17, 0, 0}, {0x344f, 0x00, 0, 0}, {0x3450, 0x57, 0, 0},
		{0x3451, 0x00, 0, 0}, {0x3452, 0x17, 0, 0}, {0x3453, 0x00, 0, 0},
		{0x3454, 0x17, 0, 0}, {0x3455, 0x00, 0, 0}, {0x3472, 0x1c, 0, 0},
		{0x3473, 0x05, 0, 0},
};

static struct reg_value imx290_setting_mode720_37clk[] = {
		{0x305c, 0x20, 0, 0}, {0x305d, 0x00, 0, 0}, {0x305e, 0x20, 0, 0},
		{0x305f, 0x01, 0, 0}, {0x315e, 0x1a, 0, 0}, {0x3164, 0x1a, 0, 0},
		{0x3480, 0x49, 0, 0},
};

static struct reg_value imx290_setting_mode1080_37clk[] = {
		{0x305c, 0x18, 0, 0}, {0x305d, 0x03, 0, 0}, {0x305e, 0x20, 0, 0},
		{0x305f, 0x01, 0, 0}, {0x315e, 0x1a, 0, 0}, {0x3164, 0x1a, 0, 0},
		{0x3480, 0x49, 0, 0},
};

static struct reg_value imx290_setting_mode720_74clk[] = {
		{0x305c, 0x10, 0, 0}, {0x305d, 0x00, 0, 0}, {0x305e, 0x10, 0, 0},
		{0x305f, 0x01, 0, 0}, {0x315e, 0x1b, 0, 0}, {0x3164, 0x1b, 0, 0},
		{0x3480, 0x92, 0, 0},
};

static struct reg_value imx290_setting_mode1080_74clk[] = {
		{0x305c, 0x0c, 0, 0}, {0x305d, 0x03, 0, 0}, {0x305e, 0x10, 0, 0},
		{0x305f, 0x01, 0, 0}, {0x315e, 0x1b, 0, 0}, {0x3164, 0x1b, 0, 0},
		{0x3480, 0x92, 0, 0},
};

static struct imx290_reg_set imx290_setting_mode_clk[imx290_mode_MAX][imx290_inclk_MAX] =
{
		{ /* mode_MIN */
				{NULL, 0},		/* 37MHz */
				{NULL, 0},		/* 74MHz */
		},
		{ /* imx290_mode_720P */
				{imx290_setting_mode720_37clk, ARRAY_SIZE(imx290_setting_mode720_37clk)},		/* 37MHz */
				{imx290_setting_mode720_74clk, ARRAY_SIZE(imx290_setting_mode720_74clk)},		/* 74MHz */
		},
		{ /* imx290_mode_1080P */
				{imx290_setting_mode1080_37clk, ARRAY_SIZE(imx290_setting_mode1080_37clk)},		/* 37MHz */
				{imx290_setting_mode1080_74clk, ARRAY_SIZE(imx290_setting_mode1080_74clk)},		/* 74MHz */
		},
		{ /* imx290_mode_CROP */
				{imx290_setting_mode1080_37clk, ARRAY_SIZE(imx290_setting_mode1080_37clk)},		/* 37MHz */
				{imx290_setting_mode1080_74clk, ARRAY_SIZE(imx290_setting_mode1080_74clk)},		/* 74MHz */
		},
		{ /* imx290_mode_PG */
				{imx290_setting_mode720_37clk, ARRAY_SIZE(imx290_setting_mode720_37clk)},		/* 37MHz */
				{imx290_setting_mode720_74clk, ARRAY_SIZE(imx290_setting_mode720_74clk)},		/* 74MHz */
		},
};


static struct imx290_mode_info imx290_mode_info_data[imx290_fps_max][imx290_mode_MAX] = {
	{	/* MIN/INVALID */
		{imx290_mode_MIN, 0, 0, NULL, 0},
		{imx290_mode_720P, 0, 0, NULL, 0},
		{imx290_mode_1080P, 0, 0, NULL, 0},
		{imx290_mode_CROP, 0, 0, NULL, 0},
		{imx290_mode_PG, 0, 0, NULL, 0},
	},
	{	/* 25 FPS */
		{imx290_mode_MIN,  0, 0, NULL, 0},
		{imx290_mode_720P,  1080, 720, ARRAY(imx290_setting_mode720_25fps_base)},
		{imx290_mode_1080P,  0, 0, NULL, 0},
		{imx290_mode_CROP,  0, 0, NULL, 0},
		{imx290_mode_PG,  0, 0, NULL, 0},
	},
	{	/* 30 FPS */
		{imx290_mode_MIN,  0, 0, NULL, 0},
		{imx290_mode_720P,  1080, 720, ARRAY(imx290_setting_mode720_25fps_base)},
		{imx290_mode_1080P,  0, 0, NULL, 0},
		{imx290_mode_CROP,  0, 0, NULL, 0},
		{imx290_mode_PG,  0, 0, NULL, 0},
	},
	{	/* 60 FPS */
		{imx290_mode_MIN,  0, 0, NULL, 0},
		{imx290_mode_720P,  1080, 720, ARRAY(imx290_setting_mode720_25fps_base)},
		{imx290_mode_1080P,  0, 0, NULL, 0},
		{imx290_mode_CROP,  0, 0, NULL, 0},
		{imx290_mode_PG,  0, 0, NULL, 0},
	},
};

static int imx290_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int imx290_remove(struct i2c_client *client);

static const struct i2c_device_id imx290_id[] = {
	{"imx290_mipi", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, imx290_id);

static struct i2c_driver imx290_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "imx290_mipi",
		  },
	.probe  = imx290_probe,
	.remove = imx290_remove,
	.id_table = imx290_id,
};

static const struct imx290_datafmt imx290_colour_fmts[] = {
	{MEDIA_BUS_FMT_Y10_1X10, V4L2_COLORSPACE_RAW, imx290_bpp10},
	{MEDIA_BUS_FMT_Y12_1X12, V4L2_COLORSPACE_RAW, imx290_bpp12},
};

static struct imx290 *to_imx290(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct imx290, subdev);
}

/* Find a data format by a pixel code in an array */
static const struct imx290_datafmt *imx290_find_datafmt(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(imx290_colour_fmts); i++)
		if (imx290_colour_fmts[i].code == code)
			return imx290_colour_fmts + i;

	return NULL;
}

static inline void imx290_delay_us(long us)
{
	if (us < 500)
		udelay(us);
	else if (us < 20000)
		usleep_range(us, us+1000);
	else
		msleep(us/1000);
}

static inline void imx290_power_up(struct imx290* imx)
{
	dev_dbg(&imx->i2c_client->dev, "%s()\n", __func__);

	if (gpio_is_valid(imx->gpio[imx290_en_reg_12])) {
		gpiod_set_raw_value(gpio_to_desc(imx->gpio[imx290_en_reg_12]), 1);
		imx290_delay_us(100);
	}

	if (gpio_is_valid(imx->gpio[imx290_en_reg_18])) {
		gpiod_set_raw_value(gpio_to_desc(imx->gpio[imx290_en_reg_18]), 1);
		imx290_delay_us(100);
	}

	if (gpio_is_valid(imx->gpio[imx290_en_reg_29])) {
		gpiod_set_raw_value(gpio_to_desc(imx->gpio[imx290_en_reg_29]), 1);
		imx290_delay_us(100);
	}

	if (gpio_is_valid(imx->gpio[imx290_en_inclk])) {
		gpiod_set_raw_value(gpio_to_desc(imx->gpio[imx290_en_inclk]), 1);
		imx290_delay_us(100);
	}
}

static inline void imx290_power_down(struct imx290* imx)
{
	dev_dbg(&imx->i2c_client->dev, "%s()\n", __func__);

	if (gpio_is_valid(imx->gpio[imx290_en_inclk])) {
		gpiod_set_raw_value(gpio_to_desc(imx->gpio[imx290_en_inclk]), 0);
		imx290_delay_us(100);
	}
	if (gpio_is_valid(imx->gpio[imx290_en_reg_29])) {
		gpiod_set_raw_value(gpio_to_desc(imx->gpio[imx290_en_reg_29]), 0);
		imx290_delay_us(100);
	}
	if (gpio_is_valid(imx->gpio[imx290_en_reg_18])) {
		gpiod_set_raw_value(gpio_to_desc(imx->gpio[imx290_en_reg_18]), 0);
		imx290_delay_us(100);
	}
	if (gpio_is_valid(imx->gpio[imx290_en_reg_12])) {
		gpiod_set_raw_value(gpio_to_desc(imx->gpio[imx290_en_reg_12]), 0);
		imx290_delay_us(100);
	}

	imx290_delay_us(5000);
}

static void imx290_reset(struct imx290* imx)
{
	dev_dbg(&imx->i2c_client->dev, "%s()\n", __func__);

	if (gpio_is_valid(imx->gpio[imx290_xclr])) {
		gpiod_set_value_cansleep(gpio_to_desc(imx->gpio[imx290_xclr]), 0);
		imx290_delay_us(1000);
		gpiod_set_value_cansleep(gpio_to_desc(imx->gpio[imx290_xclr]), 1);
		imx290_delay_us(10000);
	}
}

/**
 *
 * Perform sequential random read from IMX290 register using I2C.
 *
 * First write the address to read from (the index), then perform a read
 * with repeated start condition. Hence use i2c_transfer to transfer multiple
 * messages with Sr support.
 *
 * @param reg
 * @param buf
 * @param count
 * @return	>= 0 on success, -errno on failure
 */
static int imx290_read_reg(struct imx290* imx, u16 reg, u8 *buf, int count)
{
	struct i2c_msg msg[2];
	u8 reg_buf[2];
	int ret;

	if (!imx->i2c_client || !imx->i2c_client->adapter)
		return -1;

#ifdef DEBUG_NO_SENSOR
	dev_dbg(&imx->i2c_client->dev, "%s:read reg pass:reg=%x,count=%i\n", __func__, reg, count);
	return ARRAY_SIZE(msg);
#else

	/* First perform dummy write with the address */
	reg_buf[0] = (reg >> 8);
	reg_buf[1] = (reg & 0xff);
	msg[0].addr = imx->i2c_addr;
	msg[0].flags = 0;	// write
	msg[0].buf = reg_buf;
	msg[0].len = 2;

	/* Second, using repeated start, receive the data from reg, reg+1, ... */
	msg[1].addr = imx->i2c_addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = count;

	dev = &imx->i2c_client->dev;
	ret = i2c_transfer(imx->i2c_client->adapter, msg, ARRAY_SIZE(msg));


	if (ret != ARRAY_SIZE(msg))
		dev_dbg(&imx->i2c_client->dev, "%s:read reg error:reg=%x,count=%i,val=%i\n", __func__, reg, count,ret);
	else
		dev_dbg(&imx->i2c_client->dev, "%s:read reg success:reg=%x,count=%i\n", __func__, reg, count);
	return ret;
#endif
}

/**
 *
 * Perform sequential random read from IMX290 register using I2C
 *
 * @param reg
 * @param buf
 * @param count
 * @return	>= 0 on success, -errno on failure
 */
static int imx290_write_reg(struct imx290* imx, u16 reg, u8 val)
{
	u8 reg_buf[3];
	int ret;

	if (!imx->i2c_client || !imx->i2c_client->adapter)
		return -1;

#ifdef DEBUG_NO_SENSOR
	dev_dbg(&imx->i2c_client->dev, "%s: success(faked):reg=%x,val=%i\n", __func__, reg, val);
	return 0;
#else
	/* First perform dummy write with the address */
	reg_buf[0] = (reg >> 8);
	reg_buf[1] = (reg & 0xff);
	reg_buf[2] = val;

	dev = &imx->i2c_client->dev;
	ret = i2c_master_send(imx->i2c_client, reg_buf, 3);
	if (ret != 3) {
		dev_dbg(&imx->i2c_client->dev, "%s: error:reg=%x,val=%i,ret=%i\n", __func__, reg, val, ret);
		return ret;
	} else {
		dev_dbg(&imx->i2c_client->dev, "%s: success:reg=%x,val=%i\n", __func__, reg, val);
		return 0;
	}
#endif
}

static int imx290_write_settings(struct imx290* imx, struct reg_value* reg, int count)
{
	int ret;
	int n;
	struct device *dev;

	dev = &imx->i2c_client->dev;

	for (n=0; n<count; n++) {
		struct reg_value* r = &reg[n];
		ret = imx290_write_reg(imx, r->addr, r->val);
		if (ret < 0)
			return ret;
		if (r->delay_us)
			imx290_delay_us(r->delay_us);
	}

	return 0;
}

/**
 * Configure the BPP.
 *
 * @pre Call during standby
 *
 * @param imx
 * @param bpp
 * @return
 */
static int imx290_set_bpp(struct imx290* imx, enum imx290_bpp bpp)
{
	int ret;

	if (!CAN_CHANGE_STANDBY(imx))
		return -EPERM;

#ifdef DEBUG_NO_SENSOR
	imx->bpp = bpp;
	return 0;
#else
	if (bpp == imx290_bpp12) {
		dev_dbg(&imx->i2c_client->dev, "%s: configuring for 12bpp\n", __func__);
		ret = imx290_write_settings(imx, imx290_setting_bpp12,
				ARRAY_SIZE(imx290_setting_bpp12));
	} else {
		dev_dbg(&imx->i2c_client->dev, "%s: configuring for 10bpp\n", __func__);
		ret = imx290_write_settings(imx, imx290_setting_bpp10,
				ARRAY_SIZE(imx290_setting_bpp10));
	}
	imx->bpp = bpp;
	return ret;
#endif
}

static int imx290_init_chip(struct imx290* imx)
{
	u8 chip_id[2];
	int ret;

	dev_dbg(&imx->i2c_client->dev, "%s()\n", __func__);

	ret = imx290_read_reg(imx, 0x301e, chip_id, 2);
#ifdef DEBUG_NO_SENSOR
	dev_dbg(&imx->i2c_client->dev, "%s: probed imx290 sensor (faked)\n", __func__);
#else
	if (ret < 0 || !(chip_id[0] == 0xb2 && chip_id[1] == 0x01)) {
		dev_warn(&imx->i2c_client->dev, "%s: unable to probe imx290 sensor\n", __func__);
		return -ENODEV;
	} else
		dev_dbg(&imx->i2c_client->dev, "%s: probed imx290 sensor\n", __func__);
#endif

	imx->on = 1;
	imx->standby = 1;
	ret = imx290_write_settings(imx, imx290_init_settings,
			ARRAY_SIZE(imx290_init_settings));
	if (ret) {
		dev_err(&imx->i2c_client->dev, "%s: failure init imx290 sensor\n", __func__);
		return ret;
	}

	if (imx->csi2_lanes == imx290_lanes_2) {
		dev_dbg(&imx->i2c_client->dev, "%s: configuring for 2 lane operation\n", __func__);
		ret = imx290_write_reg(imx, 0x3407, 0x01);
		if (ret) goto err_csi;
		ret = imx290_write_reg(imx, 0x3443, 0x01);
		if (ret) goto err_csi;
	} else if (imx->csi2_lanes == imx290_lanes_4) {
		dev_dbg(&imx->i2c_client->dev, "%s: configuring for 4 lane operation\n", __func__);
		ret = imx290_write_reg(imx, 0x3407, 0x03);
		if (ret) goto err_csi;
		ret = imx290_write_reg(imx, 0x3443, 0x03);
		if (ret) goto err_csi;
	} else {
		dev_err(&imx->i2c_client->dev, "%s: invalid csi2 configure imx290 sensor\n", __func__);
		return -EINVAL;
	}

	return 0;

err_csi:
	dev_err(&imx->i2c_client->dev, "%s: error configuring csi2: %i\n", __func__, ret);
	return ret;
}

static int imx290_power_ctrl(struct imx290* imx, int enable)
{
	dev_dbg(&imx->i2c_client->dev, "%s: enable=%i\n", __func__, enable);

	if (enable) {
		imx290_power_up(imx);
		imx290_reset(imx);
		return imx290_init_chip(imx);
	} else {
		imx->on = 0;
		imx290_power_down(imx);
		return 0;
	}
}

/**
 *
 * Put the sensor in standby mode. Transition will take +- 20ms
 *
 * @param imx		The sensor
 * @param enable	value 1 means standby active, use 0 to go to normal opteration.
 * @return
 */
static int imx290_standby_mode(struct imx290* imx, int enable)
{
	int ret;

	dev_dbg(&imx->i2c_client->dev, "%s: enable=%i\n", __func__, enable);

	if (enable) {
		imx->standby = 1;
		ret = imx290_write_reg(imx, 0x3000, 0x01);
	} else {
		imx->standby = 0;
		ret = imx290_write_reg(imx, 0x3000, 0x00);
	}

	imx290_delay_us(50000);
	return ret;
}

static int imx290_set_mode(struct imx290* imx,
		enum imx290_frame_rate frame_rate, enum imx290_mode mode)
{
	dev_dbg(&imx->i2c_client->dev, "%s: fps=%i mode=%i\n", __func__, frame_rate, mode);
	struct imx290_reg_set* sensor_inclk;
	struct imx290_mode_info* sensor_mode;
	int ret;

	if (imx290_standby_mode(imx, 0) < 0)
		return -EPERM;

	if (!(frame_rate > imx290_fps_min && frame_rate < imx290_fps_max
		&& mode > imx290_mode_MIN && mode < imx290_mode_MAX))
	{
		return -EINVAL;
	}

	sensor_mode = &imx290_mode_info_data[frame_rate][mode];

	if (!sensor_mode->base) {
		return -EINVAL;
	}

	ret = imx290_write_settings(imx, sensor_mode->base,
			sensor_mode->base_size);
	if (ret < 0)
		return ret;

	sensor_inclk = &imx290_setting_mode_clk[mode][imx->inclk];
	ret = imx290_write_settings(imx, sensor_inclk->reg,
			sensor_inclk->reg_count);

	dev_dbg(&imx->i2c_client->dev, "%s: ret=%i\n", __func__, ret);
	return ret;
}

/*!
 * imx290_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int imx290_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx290 *sensor = to_imx290(client);

	dev_dbg(&sensor->i2c_client->dev, "%s: on=%i\n", __func__, on);
	if (on && !sensor->on) {
		return imx290_power_ctrl(sensor, 1);
	} else if (!on) {
		return imx290_power_ctrl(sensor, 0);
	}
	return 0;
}

/*!
 * imx290_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int imx290_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx290 *sensor = to_imx290(client);
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	dev_dbg(&sensor->i2c_client->dev, "%s: type=%i\n", __func__, a->type);

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		dev_dbg(&client->dev, "   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum imx290_frame_rate get_fps_from_v4l2(struct v4l2_fract* timeperframe)
{
	u32 tgt_fps;
	/* Check that the new frame rate is allowed. */
	if ((timeperframe->numerator == 0) ||
	    (timeperframe->denominator == 0)) {
		timeperframe->denominator = DEFAULT_FPS;
		timeperframe->numerator = 1;
	}

	tgt_fps = timeperframe->denominator /
		  timeperframe->numerator;

	if (tgt_fps > MAX_FPS) {
		timeperframe->denominator = MAX_FPS;
		timeperframe->numerator = 1;
	} else if (tgt_fps < MIN_FPS) {
		timeperframe->denominator = MIN_FPS;
		timeperframe->numerator = 1;
	}

	/* Actual frame rate we use */
	tgt_fps = timeperframe->denominator /
		  timeperframe->numerator;

	switch (tgt_fps) {
		case 25: return imx290_25_fps;
		case 30: return imx290_30_fps;
		case 60: return imx290_60_fps;
		default:
			return imx290_25_fps;
	}
}

/*!
 * ov5460_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int imx290_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx290 *sensor = to_imx290(client);
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	enum imx290_frame_rate frame_rate = get_fps_from_v4l2(timeperframe);
	enum imx290_mode orig_mode;
	int ret = 0;

	dev_dbg(&sensor->i2c_client->dev, "%s: type=%i\n", __func__, a->type);

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		orig_mode = sensor->streamcap.capturemode;
		ret = imx290_set_mode(sensor, frame_rate,
				(u32)a->parm.capture.capturemode);
		if (ret < 0)
			return ret;

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode =
				(u32)a->parm.capture.capturemode;

		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		dev_dbg(&client->dev,
		 "   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n", a->type);
		ret = -EINVAL;
		break;

	default:
		dev_dbg(&client->dev, "   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int imx290_set_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_format *format)
{
	int ret;
	struct v4l2_mbus_framefmt *mf = &format->format;
	const struct imx290_datafmt *fmt = imx290_find_datafmt(mf->code);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx290 *sensor = to_imx290(client);

	dev_dbg(&sensor->i2c_client->dev, "%s()\n", __func__);

	if (!fmt) {
		mf->code = imx290_colour_fmts[0].code;
		mf->colorspace = imx290_colour_fmts[0].colorspace;
		fmt = &imx290_colour_fmts[0];
	}

	mf->field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	sensor->fmt = fmt;

	ret = imx290_set_bpp(sensor, fmt->bpp);
	if (ret)
		dev_warn(&client->dev, "%s: failed to set format ret=%i\n", __func__, ret);

	return ret;
}


static int imx290_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx290 *sensor = to_imx290(client);
	const struct imx290_datafmt *fmt = sensor->fmt;

	if (format->pad || sensor->fps >= imx290_fps_max
			|| sensor->fps <= imx290_fps_min
			|| sensor->mode >= imx290_mode_MIN || sensor->mode >= imx290_mode_MAX)
		return -EINVAL;

	mf->height      = imx290_mode_info_data[sensor->fps][sensor->mode].height;
	mf->width      = imx290_mode_info_data[sensor->fps][sensor->mode].width;
	mf->code		= fmt->code;
	mf->colorspace	= fmt->colorspace;
	mf->field		= V4L2_FIELD_NONE;

	dev_dbg(&sensor->i2c_client->dev, "%s() %x %ix%i\n", __func__,
			mf->code, mf->width, mf->height
			);

	return 0;
}

static int imx290_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx290 *sensor = to_imx290(client);

	if (code->pad || code->index >= ARRAY_SIZE(imx290_colour_fmts))
		return -EINVAL;

	code->code = imx290_colour_fmts[code->index].code;
	dev_dbg(&sensor->i2c_client->dev, "%s() %x\n", __func__, code->code);
	return 0;
}

/*!
 * imx290_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int imx290_enum_framesizes(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx290 *sensor = to_imx290(client);

	if (fse->index > imx290_mode_MAX)
		return -EINVAL;

	fse->max_width = imx290_mode_info_data[imx290_25_fps][fse->index].width;
	fse->min_width = fse->max_width;
	fse->max_height = imx290_mode_info_data[imx290_25_fps][fse->index].height;
	fse->min_height = fse->max_height;

	dev_dbg(&sensor->i2c_client->dev, "%s() %ix%i\n", __func__,
			fse->max_width , fse->max_height
			);

	return 0;
}

/*!
 * imx290_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int imx290_enum_frameintervals(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_interval_enum *fie)
{
	int i, j, count = 0;

	if (fie->index < 0 || fie->index > imx290_mode_MAX)
		return -EINVAL;

	if (fie->width == 0 || fie->height == 0 ||
	    fie->code == 0) {
		pr_warning("Please assign pixel format, width and height.\n");
		return -EINVAL;
	}

	fie->interval.numerator = 1;

	count = 0;
	for (i = 0; i < ARRAY_SIZE(imx290_mode_info_data); i++) {
		for (j = 0; j < (imx290_mode_MAX + 1); j++) {
			if (fie->width == imx290_mode_info_data[i][j].width
			 && fie->height == imx290_mode_info_data[i][j].height
			 && imx290_mode_info_data[i][j].base != NULL) {
				count++;
			}
			if (fie->index == (count - 1)) {
				fie->interval.denominator = 0;
						//imx290_framerates[i];
				return 0;
			}
		}
	}

	return -EINVAL;
}

static int imx290_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx290 *sensor = to_imx290(client);

	ret = imx290_standby_mode(sensor, enable ? 0 : 1);

	if (ret)
		dev_warn(&client->dev, "%s: warning:"
				"failed to set steam to %i, ret=%i\n", __func__, enable, ret);

	return ret;
}

static struct v4l2_subdev_video_ops imx290_subdev_video_ops = {
	.g_parm = imx290_g_parm,
	.s_parm = imx290_s_parm,
	.s_stream = imx290_s_stream,
};

static const struct v4l2_subdev_pad_ops imx290_subdev_pad_ops = {
	.enum_frame_size       = imx290_enum_framesizes,
	.enum_frame_interval   = imx290_enum_frameintervals,
	.enum_mbus_code        = imx290_enum_mbus_code,
	.set_fmt               = imx290_set_fmt,
	.get_fmt               = imx290_get_fmt,
};

static struct v4l2_subdev_core_ops imx290_subdev_core_ops = {
	.s_power	= imx290_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= imx290_get_register,
	.s_register	= imx290_set_register,
#endif
};

static struct v4l2_subdev_ops imx290_subdev_ops = {
	.core	= &imx290_subdev_core_ops,
	.video	= &imx290_subdev_video_ops,
	.pad	= &imx290_subdev_pad_ops,
};

static void oftree_gpio_get(struct imx290* imx)
{
	struct device *dev;
	int retval;
	int n;

	dev = &imx->i2c_client->dev;

	for (n=0; n<ARRAY_SIZE(imx290_gpio_ofname); n++) {
		imx->gpio[n] = of_get_named_gpio(dev->of_node, imx290_gpio_ofname[n], 0);
		if (!gpio_is_valid(imx->gpio[n]))
			dev_warn(dev, "no sensor %s pin available\n", imx290_gpio_ofname[n]);
		else {
//			retval = devm_gpio_request(dev, imx->gpio[n],
//					imx290_gpio_ofname[n]);

			retval = devm_gpio_request_one(dev, imx->gpio[n], GPIOF_OUT_INIT_LOW,
							imx290_gpio_ofname[n]);
			if (retval < 0) {
				dev_warn(dev, "Failed to set %s pin\n", imx290_gpio_ofname[n]);
				dev_warn(dev, "retval=%d\n", retval);
				return;
			}
			gpiod_set_value_cansleep(gpio_to_desc(imx->gpio[n]), 0);
			dev_warn(dev, "initialized gpio %s\n", imx290_gpio_ofname[n]);
		}
	}
}

static void oftree_gpio_release(struct imx290* imx)
{
	struct device *dev;
	int n;

	dev = &imx->i2c_client->dev;

	for (n=0; n<ARRAY_SIZE(imx290_gpio_ofname); n++) {
		if (gpio_is_valid(imx->gpio[n]))
			devm_gpio_free(dev, imx->gpio[n]);
	}
}


/*!
 * imx290 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int imx290_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{

	struct device *dev = &client->dev;
	int retval, lanes = -1;

	dev_dbg(dev, "imx290_probe() - 3\n");


	/* Set initial values for the sensor struct. */
	memset(&imx290_data, 0, sizeof(imx290_data));
	imx290_data.i2c_client = client;
	imx290_data.sensor_clk = devm_clk_get(dev, "csi_mclk");

	if (IS_ERR(imx290_data.sensor_clk)) {
		/* assuming clock enabled by default */
		imx290_data.sensor_clk = NULL;
		dev_warn(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(imx290_data.sensor_clk);
	}

	/* CSI data lanes .... */
	retval = of_property_read_u32(dev->of_node, "csi_id", &lanes);

	if (retval) {
		dev_warn(dev, "%s: CSI data lane count property not set! defaulting to 2\n", __func__);
		lanes = 2;
	}

	if (lanes == 2) {
		imx290_data.csi2_lanes = imx290_lanes_2;
		dev_dbg(dev, "%s: configuring for 2 lanes\n", __func__);
	} else if (lanes == 4) {
		imx290_data.csi2_lanes = imx290_lanes_4;
		dev_dbg(dev, "%s: configuring for 4 lanes\n", __func__);
	} else {
		dev_err(dev, "%s: CSI data lane count invalid (got %i)\n", __func__, lanes);
		return -EINVAL;
	}

	oftree_gpio_get(&imx290_data);
	clk_prepare_enable(imx290_data.sensor_clk);

	//imx290_data.io_init = imx290_io_init;

	imx290_data.bpp = imx290_bpp12;
	imx290_data.mode = imx290_mode_720P;
	imx290_data.fps = imx290_25_fps;
	imx290_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	imx290_data.streamcap.capturemode = 0;
	imx290_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	imx290_data.streamcap.timeperframe.numerator = 1;

	retval = imx290_power_ctrl(&imx290_data, 1);
	if (retval < 0) {
		dev_warn(dev, "%s: sensor imx290 init failed\n", __func__);
		goto err;
	}

	v4l2_i2c_subdev_init(&imx290_data.subdev, client, &imx290_subdev_ops);

	imx290_data.subdev.grp_id = 290;
	retval = v4l2_async_register_subdev(&imx290_data.subdev);
	if (retval < 0) {
		dev_err(&client->dev,
					"%s--Async register failed, ret=%d\n", __func__, retval);
		goto err;
	}

	retval = imx290_standby_mode(&imx290_data, 1);
	dev_info(dev, "%s: sensor imx290_mipi found\n", __func__);
	return retval;

err:
	clk_disable_unprepare(imx290_data.sensor_clk);
	imx290_power_down(&imx290_data);
	return retval;

}

/*!
 * imx290 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int imx290_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd;

	dev_dbg(&client->dev, "imx290_remove() - q\n");

	sd = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(sd);

	clk_disable_unprepare(imx290_data.sensor_clk);

	imx290_power_ctrl(&imx290_data, 0);

	oftree_gpio_release(&imx290_data);

	return 0;
}

module_i2c_driver(imx290_i2c_driver);

MODULE_AUTHOR("2Pi Adaptive Systems");
MODULE_AUTHOR("Hendrik Beijeman, hbeyeman@gmail.com");
MODULE_DESCRIPTION("imx290 MIPI Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
