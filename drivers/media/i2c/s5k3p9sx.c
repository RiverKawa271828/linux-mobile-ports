// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S5K3P9SX 16MP raw image sensor driver (Meizu 20 rear ultra-wide).
 *
 * Register tables and power sequencing decoded from the stock camx sensor
 * module binary (com.qti.sensormodule.shinetech_s5k3p9sx.bin, "Parameter
 * Parser V3.5.1" format) — see nura-meizu-m2381 camera round notes.
 * Ground truth cross-checked against the stock running FDT (sensor node
 * qcom,cam-sensor3: csiphy2 / cci0 / MCLK1=tlmm101 / RESET1=tlmm117) and
 * the ACK value 0x3109 @ reg 0x0000 (both vendor lib and stock camx dmesg).
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define S5K3P9SX_MCLK_FREQ_19MHZ	(19200000)
#define S5K3P9SX_DATA_LANES		4

#define S5K3P9SX_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5K3P9SX_CHIP_ID		0x3109

/* Samsung 3xx-family page select */
#define S5K3P9SX_REG_PAGE_SELECT	CCI_REG16(0x6028)

#define S5K3P9SX_REG_MODE_SELECT	CCI_REG16(0x0100)
#define S5K3P9SX_MODE_STREAMING		0x0103	/* stock stream-on value */
#define S5K3P9SX_MODE_STANDBY		0x0000

#define S5K3P9SX_REG_GROUP_HOLD		CCI_REG16(0x0104)
#define S5K3P9SX_GROUP_HOLD_ON		0x0001
#define S5K3P9SX_GROUP_HOLD_OFF		0x0000

#define S5K3P9SX_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5K3P9SX_EXPOSURE_MIN		8
#define S5K3P9SX_EXPOSURE_STEP		1
#define S5K3P9SX_EXPOSURE_MARGIN	8

#define S5K3P9SX_REG_VTS		CCI_REG16(0x0340)
#define S5K3P9SX_VTS_MAX		0xffff
#define S5K3P9SX_REG_HTS		CCI_REG16(0x0342)

#define to_s5k3p9sx(_sd)		container_of(_sd, struct s5k3p9sx, sd)

/*
 * Link frequencies estimated from the mode PLL register blocks
 * (VCO = 19.2MHz / 7 * M, out div 8) — UNVERIFIED against real CSI capture,
 * first-boot calibration item (csiphy settle timing derives from these).
 */
static const s64 s5k3p9sx_link_freq_menu[] = {
	657500000,	/* 4640x3488: pixel rate ~526 MHz */
	763000000,	/* 1920x1080: pixel rate ~610 MHz */
};

static const u32 s5k3p9sx_mbus_format = MEDIA_BUS_FMT_SRGGB10_1X10;

struct s5k3p9sx_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct s5k3p9sx_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts;
	u32 link_freq_idx;
	const struct s5k3p9sx_reg_list reg_list;
};

/*
 * Mode register sequences decoded verbatim from the vendor bin (74 records
 * each; the 0x602a/0x6f12 pairs are the page-0x2000 indirect addressing).
 * Vendor groups g0 (full-res) and g2 (1080p). g1/g3 are per-vendor variants
 * of the same two modes (extra 0xb32/0xb34 writes) — not exposed for now.
 */
static const struct cci_reg_sequence s5k3p9sx_4640x3488_mode[] = {
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x6214), 0x7970 },
	{ CCI_REG16(0x6218), 0x7150 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x0ed6 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x1cf0 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x0e58 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x602a), 0x1694 },
	{ CCI_REG16(0x6f12), 0x1b0f },
	{ CCI_REG16(0x602a), 0x16aa },
	{ CCI_REG16(0x6f12), 0x009d },
	{ CCI_REG16(0x6f12), 0x0007 },
	{ CCI_REG16(0x602a), 0x1098 },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x602a), 0x2690 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0055 },
	{ CCI_REG16(0x602a), 0x16a8 },
	{ CCI_REG16(0x6f12), 0x38cd },
	{ CCI_REG16(0x602a), 0x108c },
	{ CCI_REG16(0x6f12), 0x0003 },
	{ CCI_REG16(0x602a), 0x10cc },
	{ CCI_REG16(0x6f12), 0x0008 },
	{ CCI_REG16(0x602a), 0x10d0 },
	{ CCI_REG16(0x6f12), 0x000f },
	{ CCI_REG16(0x602a), 0x0f50 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x0c60 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0202 },
	{ CCI_REG16(0x602a), 0x1758 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x0344), 0x0000 },
	{ CCI_REG16(0x0346), 0x000c },
	{ CCI_REG16(0x0348), 0x122f },
	{ CCI_REG16(0x034a), 0x0da3 },
	{ CCI_REG16(0x034c), 0x1220 },
	{ CCI_REG16(0x034e), 0x0d98 },
	{ CCI_REG16(0x0350), 0x0008 },
	{ CCI_REG16(0x0900), 0x0011 },
	{ CCI_REG16(0x0380), 0x0001 },
	{ CCI_REG16(0x0382), 0x0001 },
	{ CCI_REG16(0x0384), 0x0001 },
	{ CCI_REG16(0x0386), 0x0001 },
	{ CCI_REG16(0x0404), 0x1000 },
	{ CCI_REG16(0x0402), 0x1010 },
	{ CCI_REG16(0x0400), 0x1010 },
	{ CCI_REG16(0x0114), 0x0301 },
	{ CCI_REG16(0x0110), 0x1002 },
	{ CCI_REG16(0x0136), 0x1333 },
	{ CCI_REG16(0x0300), 0x0007 },
	{ CCI_REG16(0x0302), 0x0001 },
	{ CCI_REG16(0x0304), 0x0005 },
	{ CCI_REG16(0x0306), 0x00ff },
	{ CCI_REG16(0x0308), 0x0008 },
	{ CCI_REG16(0x030a), 0x0001 },
	{ CCI_REG16(0x030c), 0x0000 },
	{ CCI_REG16(0x030e), 0x0003 },
	{ CCI_REG16(0x0310), 0x0073 },
	{ CCI_REG16(0x0312), 0x0000 },
	{ CCI_REG16(0x0340), 0x0e4f },
	{ CCI_REG16(0x0342), 0x13e0 },
	{ CCI_REG16(0x0202), 0x0100 },
	{ CCI_REG16(0x0200), 0x0100 },
	{ CCI_REG16(0x021e), 0x0000 },
	{ CCI_REG16(0x0d00), 0x0101 },
	{ CCI_REG16(0x0d02), 0x0101 },
	{ CCI_REG16(0x0d04), 0x0102 },
	{ CCI_REG16(0x0d06), 0x0120 },
	{ CCI_REG16(0x0d08), 0x0360 },
};

static const struct cci_reg_sequence s5k3p9sx_1920x1080_mode[] = {
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x6214), 0x7970 },
	{ CCI_REG16(0x6218), 0x7150 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x0ed6 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x1cf0 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x0e58 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x602a), 0x1694 },
	{ CCI_REG16(0x6f12), 0x1b0f },
	{ CCI_REG16(0x602a), 0x16aa },
	{ CCI_REG16(0x6f12), 0x009d },
	{ CCI_REG16(0x6f12), 0x0007 },
	{ CCI_REG16(0x602a), 0x1098 },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x602a), 0x2690 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0055 },
	{ CCI_REG16(0x602a), 0x16a8 },
	{ CCI_REG16(0x6f12), 0x38cd },
	{ CCI_REG16(0x602a), 0x108c },
	{ CCI_REG16(0x6f12), 0x0003 },
	{ CCI_REG16(0x602a), 0x10cc },
	{ CCI_REG16(0x6f12), 0x0008 },
	{ CCI_REG16(0x602a), 0x10d0 },
	{ CCI_REG16(0x6f12), 0x000f },
	{ CCI_REG16(0x602a), 0x0f50 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x0c60 },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x6f12), 0x0202 },
	{ CCI_REG16(0x602a), 0x1758 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x0344), 0x0190 },
	{ CCI_REG16(0x0346), 0x02a0 },
	{ CCI_REG16(0x0348), 0x109f },
	{ CCI_REG16(0x034a), 0x0b0f },
	{ CCI_REG16(0x034c), 0x0780 },
	{ CCI_REG16(0x034e), 0x0438 },
	{ CCI_REG16(0x0350), 0x0004 },
	{ CCI_REG16(0x0900), 0x0112 },
	{ CCI_REG16(0x0380), 0x0001 },
	{ CCI_REG16(0x0382), 0x0001 },
	{ CCI_REG16(0x0384), 0x0001 },
	{ CCI_REG16(0x0386), 0x0003 },
	{ CCI_REG16(0x0404), 0x2000 },
	{ CCI_REG16(0x0402), 0x1010 },
	{ CCI_REG16(0x0400), 0x1010 },
	{ CCI_REG16(0x0114), 0x0301 },
	{ CCI_REG16(0x0110), 0x1002 },
	{ CCI_REG16(0x0136), 0x1333 },
	{ CCI_REG16(0x0300), 0x0007 },
	{ CCI_REG16(0x0302), 0x0001 },
	{ CCI_REG16(0x0304), 0x0006 },
	{ CCI_REG16(0x0306), 0x00f5 },
	{ CCI_REG16(0x0308), 0x0008 },
	{ CCI_REG16(0x030a), 0x0001 },
	{ CCI_REG16(0x030c), 0x0000 },
	{ CCI_REG16(0x030e), 0x0003 },
	{ CCI_REG16(0x0310), 0x007a },
	{ CCI_REG16(0x0312), 0x0000 },
	{ CCI_REG16(0x0340), 0x05bb },
	{ CCI_REG16(0x0342), 0x13e0 },
	{ CCI_REG16(0x0202), 0x0100 },
	{ CCI_REG16(0x0200), 0x0100 },
	{ CCI_REG16(0x021e), 0x0000 },
	{ CCI_REG16(0x0d00), 0x0101 },
	{ CCI_REG16(0x0d02), 0x0101 },
	{ CCI_REG16(0x0d04), 0x0100 },
	{ CCI_REG16(0x0d06), 0x00f0 },
	{ CCI_REG16(0x0d08), 0x021c },
};

/*
 * Boot/init sequence decoded from vendor group g9 (1600 records): id-lock
 * writes, boot trigger 0x6010, then the configuration upload. Written once
 * per stream start, mirroring the in-tree s5kjn1 driver's flow.
 */
static const struct cci_reg_sequence s5k3p9sx_init_seq[] = {
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x0000), 0x101f },
	{ CCI_REG16(0x0000), 0x3109 },
	{ CCI_REG16(0x6010), 0x0001 },
};

static const struct s5k3p9sx_mode s5k3p9sx_supported_modes[] = {
	{
		.width = 4640,
		.height = 3488,
		.hts = 5088,
		.vts = 3663,
		.link_freq_idx = 0,
		.reg_list = {
			.regs = s5k3p9sx_4640x3488_mode,
			.num_regs = ARRAY_SIZE(s5k3p9sx_4640x3488_mode),
		},
	},
	{
		.width = 1920,
		.height = 1080,
		.hts = 5088,
		.vts = 1467,
		.link_freq_idx = 1,
		.reg_list = {
			.regs = s5k3p9sx_1920x1080_mode,
			.num_regs = ARRAY_SIZE(s5k3p9sx_1920x1080_mode),
		},
	},
};

struct s5k3p9sx {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator *vdda;		/* analog 2.8V  = pm8008j L4 */
	struct regulator *vddd;		/* digital 1.056V = pm8008j L2 */
	struct regulator *vddio;	/* I/O 1.8V  = pm8008j L5 */

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;

	const struct s5k3p9sx_mode *mode;
};

static int s5k3p9sx_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k3p9sx *s5k3p9sx = container_of(ctrl->handler, struct s5k3p9sx,
						 ctrl_handler);
	const struct s5k3p9sx_mode *mode = s5k3p9sx->mode;
	s64 exposure_max;
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		exposure_max = mode->height + ctrl->val -
			       S5K3P9SX_EXPOSURE_MARGIN;
		__v4l2_ctrl_modify_range(s5k3p9sx->exposure,
					 s5k3p9sx->exposure->minimum,
					 exposure_max,
					 s5k3p9sx->exposure->step,
					 s5k3p9sx->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_active(s5k3p9sx->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = cci_write(s5k3p9sx->regmap, S5K3P9SX_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(s5k3p9sx->regmap, S5K3P9SX_REG_VTS,
				ctrl->val + mode->height, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(s5k3p9sx->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k3p9sx_ctrl_ops = {
	.s_ctrl = s5k3p9sx_set_ctrl,
};

static inline u64 s5k3p9sx_freq_to_pixel_rate(const u64 freq)
{
	return div_u64(freq * 2 * S5K3P9SX_DATA_LANES, 10);
}

static int s5k3p9sx_init_controls(struct s5k3p9sx *s5k3p9sx)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &s5k3p9sx->ctrl_handler;
	const struct s5k3p9sx_mode *mode = s5k3p9sx->mode;
	s64 pixel_rate, hblank, vblank, exposure_max;

	v4l2_ctrl_handler_init(ctrl_hdlr, 5);

	s5k3p9sx->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr,
					&s5k3p9sx_ctrl_ops, V4L2_CID_LINK_FREQ,
					ARRAY_SIZE(s5k3p9sx_link_freq_menu) - 1,
					mode->link_freq_idx,
					s5k3p9sx_link_freq_menu);
	if (s5k3p9sx->link_freq)
		s5k3p9sx->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	pixel_rate = s5k3p9sx_freq_to_pixel_rate(
			s5k3p9sx_link_freq_menu[mode->link_freq_idx]);
	s5k3p9sx->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &s5k3p9sx_ctrl_ops,
						 V4L2_CID_PIXEL_RATE,
						 0, pixel_rate, 1, pixel_rate);
	if (s5k3p9sx->pixel_rate)
		s5k3p9sx->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	hblank = mode->hts - mode->width;
	s5k3p9sx->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k3p9sx_ctrl_ops,
					     V4L2_CID_HBLANK, hblank, hblank,
					     1, hblank);
	if (s5k3p9sx->hblank)
		s5k3p9sx->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = mode->vts - mode->height;
	s5k3p9sx->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k3p9sx_ctrl_ops,
					     V4L2_CID_VBLANK, vblank,
					     S5K3P9SX_VTS_MAX - mode->height,
					     1, vblank);

	exposure_max = mode->vts - S5K3P9SX_EXPOSURE_MARGIN;
	s5k3p9sx->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &s5k3p9sx_ctrl_ops,
					       V4L2_CID_EXPOSURE,
					       S5K3P9SX_EXPOSURE_MIN,
					       exposure_max,
					       S5K3P9SX_EXPOSURE_STEP,
					       mode->vts / 2);

	s5k3p9sx->sd.ctrl_handler = ctrl_hdlr;

	return 0;
}

static int s5k3p9sx_enable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state, u32 pad,
				   u64 streams_mask)
{
	struct s5k3p9sx *s5k3p9sx = to_s5k3p9sx(sd);
	const struct s5k3p9sx_reg_list *reg_list = &s5k3p9sx->mode->reg_list;
	int ret;

	ret = pm_runtime_resume_and_get(s5k3p9sx->dev);
	if (ret)
		return ret;

	/* Boot/init: id lock + trigger, then the full configuration upload */
	cci_multi_reg_write(s5k3p9sx->regmap, s5k3p9sx_init_seq,
			    ARRAY_SIZE(s5k3p9sx_init_seq), &ret);
	if (ret)
		goto error;

	usleep_range(5 * USEC_PER_MSEC, 6 * USEC_PER_MSEC);

	cci_multi_reg_write(s5k3p9sx->regmap, reg_list->regs,
			    reg_list->num_regs, &ret);
	if (ret)
		goto error;

	ret = __v4l2_ctrl_handler_setup(s5k3p9sx->sd.ctrl_handler);

	/* Stream on, page-select first (vendor startStreaming sequence) */
	cci_write(s5k3p9sx->regmap, S5K3P9SX_REG_PAGE_SELECT, 0x4000, &ret);
	cci_write(s5k3p9sx->regmap, S5K3P9SX_REG_MODE_SELECT,
		  S5K3P9SX_MODE_STREAMING, &ret);
	if (ret)
		goto error;

	usleep_range(5 * USEC_PER_MSEC, 6 * USEC_PER_MSEC);

	return 0;

error:
	dev_err(s5k3p9sx->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(s5k3p9sx->dev);

	return ret;
}

static int s5k3p9sx_disable_streams(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state, u32 pad,
				    u64 streams_mask)
{
	struct s5k3p9sx *s5k3p9sx = to_s5k3p9sx(sd);
	int ret;

	cci_write(s5k3p9sx->regmap, S5K3P9SX_REG_PAGE_SELECT, 0x4000, NULL);
	ret = cci_write(s5k3p9sx->regmap, S5K3P9SX_REG_MODE_SELECT,
			S5K3P9SX_MODE_STANDBY, NULL);
	if (ret)
		dev_err(s5k3p9sx->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(s5k3p9sx->dev);

	return ret;
}

static void s5k3p9sx_update_pad_format(struct s5k3p9sx *s5k3p9sx,
				       const struct s5k3p9sx_mode *mode,
				       struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = s5k3p9sx_mbus_format;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5k3p9sx_set_pad_format(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_format *fmt)
{
	struct s5k3p9sx *s5k3p9sx = to_s5k3p9sx(sd);
	s64 hblank, vblank, exposure_max;
	const struct s5k3p9sx_mode *mode;
	u64 new_pixel_rate;

	mode = v4l2_find_nearest_size(s5k3p9sx_supported_modes,
				      ARRAY_SIZE(s5k3p9sx_supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	s5k3p9sx_update_pad_format(s5k3p9sx, mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY || s5k3p9sx->mode == mode)
		goto set_format;

	hblank = mode->hts - mode->width;
	__v4l2_ctrl_modify_range(s5k3p9sx->hblank, hblank, hblank, 1, hblank);

	vblank = mode->vts - mode->height;
	__v4l2_ctrl_modify_range(s5k3p9sx->vblank, vblank,
				 S5K3P9SX_VTS_MAX - mode->height, 1, vblank);
	__v4l2_ctrl_s_ctrl(s5k3p9sx->vblank, vblank);

	exposure_max = mode->vts - S5K3P9SX_EXPOSURE_MARGIN;
	__v4l2_ctrl_modify_range(s5k3p9sx->exposure, S5K3P9SX_EXPOSURE_MIN,
				 exposure_max, S5K3P9SX_EXPOSURE_STEP,
				 mode->vts / 2);
	__v4l2_ctrl_s_ctrl(s5k3p9sx->exposure, mode->vts / 2);

	new_pixel_rate = s5k3p9sx_freq_to_pixel_rate(
				s5k3p9sx_link_freq_menu[mode->link_freq_idx]);
	__v4l2_ctrl_modify_range(s5k3p9sx->pixel_rate, 0, new_pixel_rate,
				 1, new_pixel_rate);
	__v4l2_ctrl_s_ctrl(s5k3p9sx->pixel_rate, new_pixel_rate);
	__v4l2_ctrl_s_ctrl(s5k3p9sx->link_freq, mode->link_freq_idx);

	if (s5k3p9sx->sd.ctrl_handler->error)
		return s5k3p9sx->sd.ctrl_handler->error;

	s5k3p9sx->mode = mode;

set_format:
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int s5k3p9sx_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = s5k3p9sx_mbus_format;

	return 0;
}

static int s5k3p9sx_enum_frame_size(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(s5k3p9sx_supported_modes))
		return -EINVAL;

	if (fse->code != s5k3p9sx_mbus_format)
		return -EINVAL;

	fse->min_width = s5k3p9sx_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5k3p9sx_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int s5k3p9sx_get_selection(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_selection *sel)
{
	struct s5k3p9sx *s5k3p9sx = to_s5k3p9sx(sd);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = s5k3p9sx->mode->width;
		sel->r.height = s5k3p9sx->mode->height;
		return 0;
	default:
		return -EINVAL;
	}
}

static int s5k3p9sx_init_state(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state)
{
	struct s5k3p9sx *s5k3p9sx = to_s5k3p9sx(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = s5k3p9sx->mode->width,
			.height = s5k3p9sx->mode->height,
		},
	};

	s5k3p9sx_set_pad_format(sd, state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops s5k3p9sx_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5k3p9sx_pad_ops = {
	.set_fmt = s5k3p9sx_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = s5k3p9sx_get_selection,
	.enum_mbus_code = s5k3p9sx_enum_mbus_code,
	.enum_frame_size = s5k3p9sx_enum_frame_size,
	.enable_streams = s5k3p9sx_enable_streams,
	.disable_streams = s5k3p9sx_disable_streams,
};

static const struct v4l2_subdev_ops s5k3p9sx_subdev_ops = {
	.video = &s5k3p9sx_video_ops,
	.pad = &s5k3p9sx_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k3p9sx_internal_ops = {
	.init_state = s5k3p9sx_init_state,
};

static const struct media_entity_operations s5k3p9sx_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int s5k3p9sx_identify_sensor(struct s5k3p9sx *s5k3p9sx)
{
	u64 val;
	int ret;

	ret = cci_read(s5k3p9sx->regmap, S5K3P9SX_REG_CHIP_ID, &val, NULL);
	if (ret) {
		dev_err(s5k3p9sx->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (val != S5K3P9SX_CHIP_ID) {
		dev_err(s5k3p9sx->dev, "chip id mismatch: %llx!=%x\n", val,
			S5K3P9SX_CHIP_ID);
		return -ENODEV;
	}

	return 0;
}

static int s5k3p9sx_check_hwcfg(struct s5k3p9sx *s5k3p9sx)
{
	struct fwnode_handle *fwnode = dev_fwnode(s5k3p9sx->dev), *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus = {
			.mipi_csi2 = {
				.num_data_lanes = S5K3P9SX_DATA_LANES,
			},
		},
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	unsigned long freq_bitmap;
	int ret;

	if (!fwnode)
		return -ENODEV;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != S5K3P9SX_DATA_LANES) {
		dev_err(s5k3p9sx->dev, "Invalid number of data lanes: %u\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto endpoint_free;
	}

	ret = v4l2_link_freq_to_bitmap(s5k3p9sx->dev,
				       bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       s5k3p9sx_link_freq_menu,
				       ARRAY_SIZE(s5k3p9sx_link_freq_menu),
				       &freq_bitmap);

endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int s5k3p9sx_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k3p9sx *s5k3p9sx = to_s5k3p9sx(sd);
	int ret;

	/*
	 * Stock powerSetting: vio -> vdig -> vana(+5ms) -> reset/release
	 * (+12ms) -> MCLK 19.2MHz (+10ms). Rails then clock then run.
	 */
	ret = regulator_enable(s5k3p9sx->vddio);
	if (ret)
		return ret;

	ret = regulator_enable(s5k3p9sx->vddd);
	if (ret)
		goto disable_vddio;

	ret = regulator_enable(s5k3p9sx->vdda);
	if (ret)
		goto disable_vddd;

	usleep_range(5000, 6000);

	ret = clk_prepare_enable(s5k3p9sx->mclk);
	if (ret)
		goto disable_vdda;

	gpiod_set_value_cansleep(s5k3p9sx->reset_gpio, 0);
	usleep_range(12000, 15000);

	return 0;

disable_vdda:
	regulator_disable(s5k3p9sx->vdda);
disable_vddd:
	regulator_disable(s5k3p9sx->vddd);
disable_vddio:
	regulator_disable(s5k3p9sx->vddio);

	return ret;
}

static int s5k3p9sx_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k3p9sx *s5k3p9sx = to_s5k3p9sx(sd);

	gpiod_set_value_cansleep(s5k3p9sx->reset_gpio, 1);

	clk_disable_unprepare(s5k3p9sx->mclk);

	regulator_disable(s5k3p9sx->vdda);
	regulator_disable(s5k3p9sx->vddd);
	regulator_disable(s5k3p9sx->vddio);

	return 0;
}

static int s5k3p9sx_probe(struct i2c_client *client)
{
	struct s5k3p9sx *s5k3p9sx;
	unsigned long freq;
	int ret;

	s5k3p9sx = devm_kzalloc(&client->dev, sizeof(*s5k3p9sx), GFP_KERNEL);
	if (!s5k3p9sx)
		return -ENOMEM;

	s5k3p9sx->dev = &client->dev;
	v4l2_i2c_subdev_init(&s5k3p9sx->sd, client, &s5k3p9sx_subdev_ops);

	s5k3p9sx->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5k3p9sx->regmap))
		return dev_err_probe(s5k3p9sx->dev, PTR_ERR(s5k3p9sx->regmap),
				     "failed to init CCI\n");

	s5k3p9sx->mclk = devm_v4l2_sensor_clk_get(s5k3p9sx->dev, NULL);
	if (IS_ERR(s5k3p9sx->mclk))
		return dev_err_probe(s5k3p9sx->dev, PTR_ERR(s5k3p9sx->mclk),
				     "failed to get MCLK clock\n");

	freq = clk_get_rate(s5k3p9sx->mclk);
	if (freq != S5K3P9SX_MCLK_FREQ_19MHZ)
		return dev_err_probe(s5k3p9sx->dev, -EINVAL,
				     "MCLK clock frequency %lu is not supported\n",
				     freq);

	ret = s5k3p9sx_check_hwcfg(s5k3p9sx);
	if (ret)
		return dev_err_probe(s5k3p9sx->dev, ret,
				     "failed to check HW configuration\n");

	s5k3p9sx->reset_gpio = devm_gpiod_get_optional(s5k3p9sx->dev, "reset",
						       GPIOD_OUT_HIGH);
	if (IS_ERR(s5k3p9sx->reset_gpio))
		return dev_err_probe(s5k3p9sx->dev,
				     PTR_ERR(s5k3p9sx->reset_gpio),
				     "cannot get reset GPIO\n");

	s5k3p9sx->vdda = devm_regulator_get_optional(s5k3p9sx->dev, "vdda");
	if (IS_ERR(s5k3p9sx->vdda))
		return dev_err_probe(s5k3p9sx->dev, PTR_ERR(s5k3p9sx->vdda),
				     "cannot get vdda regulator\n");

	s5k3p9sx->vddd = devm_regulator_get_optional(s5k3p9sx->dev, "vddd");
	if (IS_ERR(s5k3p9sx->vddd))
		return dev_err_probe(s5k3p9sx->dev, PTR_ERR(s5k3p9sx->vddd),
				     "cannot get vddd regulator\n");

	s5k3p9sx->vddio = devm_regulator_get_optional(s5k3p9sx->dev, "vddio");
	if (IS_ERR(s5k3p9sx->vddio))
		return dev_err_probe(s5k3p9sx->dev, PTR_ERR(s5k3p9sx->vddio),
				     "cannot get vddio regulator\n");

	/* The sensor must be powered on to read the CHIP_ID register */
	ret = s5k3p9sx_power_on(s5k3p9sx->dev);
	if (ret)
		return ret;

	ret = s5k3p9sx_identify_sensor(s5k3p9sx);
	if (ret) {
		dev_err_probe(s5k3p9sx->dev, ret, "failed to find sensor\n");
		goto power_off;
	}

	s5k3p9sx->mode = &s5k3p9sx_supported_modes[0];
	ret = s5k3p9sx_init_controls(s5k3p9sx);
	if (ret) {
		dev_err_probe(s5k3p9sx->dev, ret, "failed to init controls\n");
		goto power_off;
	}

	s5k3p9sx->sd.state_lock = s5k3p9sx->ctrl_handler.lock;
	s5k3p9sx->sd.internal_ops = &s5k3p9sx_internal_ops;
	s5k3p9sx->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5k3p9sx->sd.entity.ops = &s5k3p9sx_subdev_entity_ops;
	s5k3p9sx->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5k3p9sx->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&s5k3p9sx->sd.entity, 1, &s5k3p9sx->pad);
	if (ret) {
		dev_err_probe(s5k3p9sx->dev, ret,
			      "failed to init media entity pads\n");
		goto v4l2_ctrl_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&s5k3p9sx->sd);
	if (ret < 0) {
		dev_err_probe(s5k3p9sx->dev, ret,
			      "failed to init v4l2 subdev\n");
		goto media_entity_cleanup;
	}

	pm_runtime_set_active(s5k3p9sx->dev);
	pm_runtime_enable(s5k3p9sx->dev);

	ret = v4l2_async_register_subdev_sensor(&s5k3p9sx->sd);
	if (ret < 0) {
		dev_err_probe(s5k3p9sx->dev, ret,
			      "failed to register V4L2 subdev\n");
		goto subdev_cleanup;
	}

	pm_runtime_set_autosuspend_delay(s5k3p9sx->dev, 1000);
	pm_runtime_use_autosuspend(s5k3p9sx->dev);
	pm_runtime_idle(s5k3p9sx->dev);

	return 0;

subdev_cleanup:
	v4l2_subdev_cleanup(&s5k3p9sx->sd);
	pm_runtime_disable(s5k3p9sx->dev);
	pm_runtime_set_suspended(s5k3p9sx->dev);

media_entity_cleanup:
	media_entity_cleanup(&s5k3p9sx->sd.entity);

v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(s5k3p9sx->sd.ctrl_handler);

power_off:
	s5k3p9sx_power_off(s5k3p9sx->dev);

	return ret;
}

static void s5k3p9sx_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k3p9sx *s5k3p9sx = to_s5k3p9sx(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(s5k3p9sx->dev);

	if (!pm_runtime_status_suspended(s5k3p9sx->dev)) {
		s5k3p9sx_power_off(s5k3p9sx->dev);
		pm_runtime_set_suspended(s5k3p9sx->dev);
	}
}

static const struct dev_pm_ops s5k3p9sx_pm_ops = {
	SET_RUNTIME_PM_OPS(s5k3p9sx_power_off, s5k3p9sx_power_on, NULL)
};

static const struct of_device_id s5k3p9sx_of_match[] = {
	{ .compatible = "samsung,s5k3p9sx" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5k3p9sx_of_match);

static struct i2c_driver s5k3p9sx_i2c_driver = {
	.driver = {
		.name = "s5k3p9sx",
		.pm = &s5k3p9sx_pm_ops,
		.of_match_table = s5k3p9sx_of_match,
	},
	.probe = s5k3p9sx_probe,
	.remove = s5k3p9sx_remove,
};

module_i2c_driver(s5k3p9sx_i2c_driver);

MODULE_AUTHOR("Nura Meizu 20 porting crew");
MODULE_DESCRIPTION("Samsung S5K3P9SX image sensor driver");
MODULE_LICENSE("GPL");
