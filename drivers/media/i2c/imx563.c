// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sony IMX563 camera sensor driver
 *
 * Copyright (C) 2021 Intel Corporation
 */
#include <linux/unaligned.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* Streaming Mode */
#define IMX563_REG_MODE_SELECT	CCI_REG8(0x0100)
#define IMX563_MODE_STANDBY	0x00
#define IMX563_MODE_STREAMING	0x01

/* Lines per frame */
#define IMX563_REG_LPFR		CCI_REG16(0x0340)

/* Chip ID */
#define IMX563_REG_ID		CCI_REG16(0x0016)
#define IMX563_ID		0x563

/* Exposure control */
#define IMX563_REG_EXPOSURE_CIT	CCI_REG16(0x0202)
#define IMX563_EXPOSURE_MIN	8
#define IMX563_EXPOSURE_OFFSET	8
#define IMX563_EXPOSURE_STEP	1
#define IMX563_EXPOSURE_DEFAULT	0x0c04

/* Analog gain control */
#define IMX563_REG_AGAIN	CCI_REG16(0x0204)
#define IMX563_AGAIN_MIN	0
#define IMX563_AGAIN_MAX	896
#define IMX563_AGAIN_STEP	1
#define IMX563_AGAIN_DEFAULT	0

/* Group hold register */
#define IMX563_REG_HOLD		CCI_REG8(0x0104)

/* Input clock rate */
#define IMX563_INCLK_RATE	19200000

/* CSI2 HW configuration */
#define IMX563_LINK_FREQ	1507202083
#define IMX563_NUM_DATA_LANES	3

#define IMX563_REG_MIN		0x00
#define IMX563_REG_MAX		0xffff

/**
 * struct imx563_reg_list - imx563 sensor register list
 * @num_of_regs: Number of registers in the list
 * @regs: Pointer to register list
 */
struct imx563_reg_list {
	u32 num_of_regs;
	const struct cci_reg_sequence *regs;
};

/**
 * struct imx563_mode - imx563 sensor mode structure
 * @width: Frame width
 * @height: Frame height
 * @code: Format code
 * @hblank: Horizontal blanking in lines
 * @vblank: Vertical blanking in lines
 * @vblank_min: Minimum vertical blanking in lines
 * @vblank_max: Maximum vertical blanking in lines
 * @pclk: Sensor pixel clock
 * @link_freq_idx: Link frequency index
 * @reg_list: Register list for sensor mode
 */
struct imx563_mode {
	u32 width;
	u32 height;
	u32 code;
	u32 hblank;
	u32 vblank;
	u32 vblank_min;
	u32 vblank_max;
	u64 pclk;
	u32 link_freq_idx;
	struct imx563_reg_list reg_list;
};

static const char * const imx563_supply_names[] = {
	"avdd",		/* Analog power */
	"dvdd",		/* Digital core power */
	"dovdd",	/* Digital I/O power */
};

/**
 * struct imx563 - imx563 sensor device structure
 * @dev: Pointer to generic device
 * @cci: CCI register map
 * @client: Pointer to i2c client
 * @sd: V4L2 sub-device
 * @pad: Media pad. Only one pad supported
 * @reset_gpio: Sensor reset gpio
 * @inclk: Sensor input clock
 * @supplies: Regulator supplies
 * @ctrl_handler: V4L2 control handler
 * @link_freq_ctrl: Pointer to link frequency control
 * @pclk_ctrl: Pointer to pixel clock control
 * @hblank_ctrl: Pointer to horizontal blanking control
 * @vblank_ctrl: Pointer to vertical blanking control
 * @exp_ctrl: Pointer to exposure control
 * @again_ctrl: Pointer to analog gain control
 * @vblank: Vertical blanking in lines
 * @cur_mode: Pointer to current selected sensor mode
 */
struct imx563 {
	struct device *dev;
	struct regmap *cci;
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct gpio_desc *reset_gpio;
	struct clk *inclk;
	struct regulator_bulk_data supplies[ARRAY_SIZE(imx563_supply_names)];
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq_ctrl;
	struct v4l2_ctrl *pclk_ctrl;
	struct v4l2_ctrl *hblank_ctrl;
	struct v4l2_ctrl *vblank_ctrl;
	struct {
		struct v4l2_ctrl *exp_ctrl;
		struct v4l2_ctrl *again_ctrl;
	};
	u32 vblank;
	const struct imx563_mode *cur_mode;
};

static const s64 link_freq[] = {
	IMX563_LINK_FREQ,
};

/* Sensor mode registers */
static const struct cci_reg_sequence mode_4032x3024_regs[] = {
	{ CCI_REG8(0x0136), 0x13 },
	{ CCI_REG8(0x0137), 0x33 },
	{ CCI_REG8(0x33f0), 0x01 },
	{ CCI_REG8(0x33f1), 0x0a },
	{ CCI_REG8(0x0111), 0x03 },
	{ CCI_REG8(0x9f00), 0x01 },
	{ CCI_REG8(0x9f04), 0x02 },
	{ CCI_REG8(0x9f05), 0x3f },
	{ CCI_REG8(0xbc40), 0x03 },
	{ CCI_REG8(0xbc41), 0x03 },
	{ CCI_REG8(0xbc4c), 0x03 },
	{ CCI_REG8(0x0101), 0x00 },
	{ CCI_REG8(0x0106), 0x01 },
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x02 },
	{ CCI_REG8(0x0220), 0x00 },
	{ CCI_REG8(0x0221), 0x11 },
	{ CCI_REG8(0x0222), 0x01 },
	{ CCI_REG8(0x0340), 0x0c },
	{ CCI_REG8(0x0341), 0x64 },
	{ CCI_REG8(0x0342), 0x32 },
	{ CCI_REG8(0x0343), 0xd0 },
	{ CCI_REG8(0x0900), 0x00 },
	{ CCI_REG8(0x0901), 0x11 },
	{ CCI_REG8(0x0902), 0x00 },
	{ CCI_REG8(0x3004), 0x03 },
	{ CCI_REG8(0x30a5), 0x00 },
	{ CCI_REG8(0x30a8), 0x06 },
	{ CCI_REG8(0x3200), 0x01 },
	{ CCI_REG8(0x4060), 0x00 },
	{ CCI_REG8(0x4061), 0x00 },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x00 },
	{ CCI_REG8(0x0348), 0x0f },
	{ CCI_REG8(0x0349), 0xbf },
	{ CCI_REG8(0x034a), 0x0b },
	{ CCI_REG8(0x034b), 0xcf },
	{ CCI_REG8(0x034c), 0x0f },
	{ CCI_REG8(0x034d), 0xc0 },
	{ CCI_REG8(0x034e), 0x0b },
	{ CCI_REG8(0x034f), 0xd0 },
	{ CCI_REG8(0x0408), 0x00 },
	{ CCI_REG8(0x0409), 0x00 },
	{ CCI_REG8(0x040a), 0x00 },
	{ CCI_REG8(0x040b), 0x00 },
	{ CCI_REG8(0x040c), 0x0f },
	{ CCI_REG8(0x040d), 0xc0 },
	{ CCI_REG8(0x040e), 0x0b },
	{ CCI_REG8(0x040f), 0xd0 },
	{ CCI_REG8(0x0303), 0x04 },
	{ CCI_REG8(0x0305), 0x02 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0x81 },
	{ CCI_REG8(0x030b), 0x02 },
	{ CCI_REG8(0x030d), 0x02 },
	{ CCI_REG8(0x030e), 0x01 },
	{ CCI_REG8(0x030f), 0x3a },
	{ CCI_REG8(0x0310), 0x01 },
	{ CCI_REG8(0x3240), 0x00 },
	{ CCI_REG8(0x0860), 0x80 },
	{ CCI_REG8(0x0861), 0x28 },
	{ CCI_REG8(0x0862), 0x00 },
	{ CCI_REG8(0x0863), 0x28 },
	{ CCI_REG8(0x40b8), 0x03 },
	{ CCI_REG8(0x40b9), 0x84 },
	{ CCI_REG8(0x40bc), 0x00 },
	{ CCI_REG8(0x40bd), 0x78 },
	{ CCI_REG8(0x40be), 0x00 },
	{ CCI_REG8(0x40bf), 0x78 },
	{ CCI_REG8(0x0202), 0x0c },
	{ CCI_REG8(0x0203), 0x04 },
	{ CCI_REG8(0x0224), 0x01 },
	{ CCI_REG8(0x0225), 0xf4 },
	{ CCI_REG8(0x0204), 0x00 },
	{ CCI_REG8(0x0205), 0x00 },
	{ CCI_REG8(0x020e), 0x01 },
	{ CCI_REG8(0x020f), 0x00 },
	{ CCI_REG8(0x0216), 0x00 },
	{ CCI_REG8(0x0217), 0x00 },
	{ CCI_REG8(0x0218), 0x01 },
	{ CCI_REG8(0x0219), 0x00 },
	{ CCI_REG8(0xdf03), 0x00 },
};

/* Supported sensor mode configurations */
static const struct imx563_mode supported_mode = {
	.width = 4032,
	.height = 3024,
	.hblank = 8976,
	.vblank = 148,
	.vblank_min = 148,
	.vblank_max = 62511,
	.pclk = 1033510000,
	.link_freq_idx = 0,
	.code = MEDIA_BUS_FMT_SRGGB10_1X10,
	.reg_list = {
		.num_of_regs = ARRAY_SIZE(mode_4032x3024_regs),
		.regs = mode_4032x3024_regs,
	},
};

/**
 * to_imx563() - imx563 V4L2 sub-device to imx563 device.
 * @subdev: pointer to imx563 V4L2 sub-device
 *
 * Return: pointer to imx563 device
 */
static inline struct imx563 *to_imx563(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct imx563, sd);
}

/**
 * imx563_update_controls() - Update control ranges based on streaming mode
 * @imx563: pointer to imx563 device
 * @mode: pointer to imx563_mode sensor mode
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx563_update_controls(struct imx563 *imx563,
				  const struct imx563_mode *mode)
{
	int ret;

	ret = __v4l2_ctrl_s_ctrl(imx563->link_freq_ctrl, mode->link_freq_idx);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_s_ctrl(imx563->hblank_ctrl, mode->hblank);
	if (ret)
		return ret;

	return __v4l2_ctrl_modify_range(imx563->vblank_ctrl, mode->vblank_min,
					mode->vblank_max, 1, mode->vblank);
}

/**
 * imx563_update_exp_gain() - Set updated exposure and gain
 * @imx563: pointer to imx563 device
 * @exposure: updated exposure value
 * @gain: updated analog gain value
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx563_update_exp_gain(struct imx563 *imx563, u32 exposure, u32 gain)
{
	u32 lpfr;
	int ret = 0;
	int ret_hold;

	lpfr = imx563->vblank + imx563->cur_mode->height;

	dev_dbg(imx563->dev, "Set exp %u, analog gain %u, lpfr %u\n",
		exposure, gain, lpfr);

	cci_write(imx563->cci, IMX563_REG_HOLD, 1, &ret);

	cci_write(imx563->cci, IMX563_REG_LPFR, lpfr, &ret);

	cci_write(imx563->cci, IMX563_REG_EXPOSURE_CIT, exposure, &ret);

	cci_write(imx563->cci, IMX563_REG_AGAIN, gain, &ret);

	ret_hold = cci_write(imx563->cci, IMX563_REG_HOLD, 0, NULL);
	if (ret_hold)
		return ret_hold;

	return ret;
}

static int imx563_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx563 *imx563 =
		container_of(ctrl->handler, struct imx563, ctrl_handler);
	u32 analog_gain;
	u32 exposure;
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		imx563->vblank = imx563->vblank_ctrl->val;

		dev_dbg(imx563->dev, "Received vblank %u, new lpfr %u\n",
			imx563->vblank,
			imx563->vblank + imx563->cur_mode->height);

		ret = __v4l2_ctrl_modify_range(imx563->exp_ctrl,
					       IMX563_EXPOSURE_MIN,
					       imx563->vblank +
					       imx563->cur_mode->height -
					       IMX563_EXPOSURE_OFFSET,
					       1, IMX563_EXPOSURE_DEFAULT);
		break;
	case V4L2_CID_EXPOSURE:
		/* Set controls only if sensor is in power on state */
		if (!pm_runtime_get_if_in_use(imx563->dev))
			return 0;

		exposure = ctrl->val;
		analog_gain = imx563->again_ctrl->val;

		dev_dbg(imx563->dev, "Received exp %u, analog gain %u\n",
			exposure, analog_gain);

		ret = imx563_update_exp_gain(imx563, exposure, analog_gain);

		pm_runtime_put(imx563->dev);

		break;
	default:
		dev_err(imx563->dev, "Invalid control %d\n", ctrl->id);
		ret = -EINVAL;
	}

	return ret;
}

/* V4l2 subdevice control ops*/
static const struct v4l2_ctrl_ops imx563_ctrl_ops = {
	.s_ctrl = imx563_set_ctrl,
};

static int imx563_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = supported_mode.code;

	return 0;
}

static int imx563_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fsize)
{
	if (fsize->index > 0)
		return -EINVAL;

	if (fsize->code != supported_mode.code)
		return -EINVAL;

	fsize->min_width = supported_mode.width;
	fsize->max_width = fsize->min_width;
	fsize->min_height = supported_mode.height;
	fsize->max_height = fsize->min_height;

	return 0;
}

/**
 * imx563_fill_pad_format() - Fill subdevice pad format
 *                            from selected sensor mode
 * @imx563: pointer to imx563 device
 * @mode: pointer to imx563_mode sensor mode
 * @fmt: V4L2 sub-device format need to be filled
 */
static void imx563_fill_pad_format(struct imx563 *imx563,
				   const struct imx563_mode *mode,
				   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = mode->code;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	fmt->format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->format.quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->format.xfer_func = V4L2_XFER_FUNC_NONE;
}

static int imx563_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx563 *imx563 = to_imx563(sd);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		fmt->format = *framefmt;
	} else {
		imx563_fill_pad_format(imx563, imx563->cur_mode, fmt);
	}

	return 0;
}

static int imx563_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx563 *imx563 = to_imx563(sd);
	const struct imx563_mode *mode;
	int ret = 0;

	mode = &supported_mode;
	imx563_fill_pad_format(imx563, mode, fmt);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		*framefmt = fmt->format;
	} else {
		ret = imx563_update_controls(imx563, mode);
		if (!ret)
			imx563->cur_mode = mode;
	}

	return ret;
}

static int imx563_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state)
{
	struct imx563 *imx563 = to_imx563(sd);
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = sd_state ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	imx563_fill_pad_format(imx563, &supported_mode, &fmt);

	return imx563_set_pad_format(sd, sd_state, &fmt);
}

/**
 * imx563_enable_streams() - Enable specified streams for the sensor
 * @sd: pointer to the V4L2 subdevice
 * @state: pointer to the subdevice state
 * @pad: pad number for which streams are enabled
 * @streams_mask: bitmask specifying the streams to enable
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx563_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 u32 pad, u64 streams_mask)
{
	const struct imx563_reg_list *reg_list;
	struct imx563 *imx563 = to_imx563(sd);
	int ret;

	ret = pm_runtime_resume_and_get(imx563->dev);
	if (ret < 0)
		return ret;

	/* Write sensor mode registers */
	reg_list = &imx563->cur_mode->reg_list;
	ret = cci_multi_reg_write(imx563->cci, reg_list->regs,
				  reg_list->num_of_regs, NULL);
	if (ret) {
		dev_err(imx563->dev, "fail to write initial registers\n");
		goto err_rpm_put;
	}

	/* Setup handler will write actual exposure and gain */
	ret =  __v4l2_ctrl_handler_setup(imx563->sd.ctrl_handler);
	if (ret) {
		dev_err(imx563->dev, "fail to setup handler\n");
		goto err_rpm_put;
	}

	/* Delay is required before streaming*/
	usleep_range(7400, 8000);

	/* Start streaming */
	ret = cci_write(imx563->cci, IMX563_REG_MODE_SELECT,
			IMX563_MODE_STREAMING, NULL);
	if (ret) {
		dev_err(imx563->dev, "fail to start streaming\n");
		goto err_rpm_put;
	}

	return 0;

err_rpm_put:
	pm_runtime_put(imx563->dev);

	return ret;
}

/**
 * imx563_disable_streams() - Enable specified streams for the sensor
 * @sd: pointer to the V4L2 subdevice
 * @state: pointer to the subdevice state
 * @pad: pad number for which streams are disabled
 * @streams_mask: bitmask specifying the streams to disable
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx563_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  u32 pad, u64 streams_mask)
{
	struct imx563 *imx563 = to_imx563(sd);
	int ret;

	ret = cci_write(imx563->cci, IMX563_REG_MODE_SELECT,
			IMX563_MODE_STANDBY, NULL);

	if (ret)
		dev_err(imx563->dev, "failed to set stream off\n");

	pm_runtime_put(imx563->dev);

	return 0;
}

/**
 * imx563_detect() - Detect imx563 sensor
 * @imx563: pointer to imx563 device
 *
 * Return: 0 if successful, -EIO if sensor id does not match
 */
static int imx563_detect(struct imx563 *imx563)
{
	int ret;
	u64 val;

	ret = cci_read(imx563->cci, IMX563_REG_ID, &val, NULL);
	if (ret)
		return dev_err_probe(imx563->dev, ret,
				     "failed to read chip id %x\n",
				     IMX563_ID);

	if (val != IMX563_ID) {
		return dev_err_probe(imx563->dev, -ENODEV,
				     "chip id mismatch: %x!=%llx",
				     IMX563_ID, val);
	}

	return 0;
}

/**
 * imx563_parse_hw_config() - Parse HW configuration and check if supported
 * @imx563: pointer to imx563 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx563_parse_hw_config(struct imx563 *imx563)
{
	struct fwnode_handle *fwnode = dev_fwnode(imx563->dev);
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_CPHY
	};
	struct fwnode_handle *ep;
	unsigned long rate;
	unsigned int i;
	int ret;

	if (!fwnode)
		return -ENXIO;

	/* Request optional reset pin */
	imx563->reset_gpio = devm_gpiod_get_optional(imx563->dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(imx563->reset_gpio)) {
		dev_err(imx563->dev, "failed to get reset gpio %pe\n",
			imx563->reset_gpio);
		return PTR_ERR(imx563->reset_gpio);
	}

	/* Get sensor input clock */
	imx563->inclk = devm_v4l2_sensor_clk_get(imx563->dev, NULL);
	if (IS_ERR(imx563->inclk))
		return dev_err_probe(imx563->dev, PTR_ERR(imx563->inclk),
				     "could not get inclk\n");

	rate = clk_get_rate(imx563->inclk);
	if (rate != IMX563_INCLK_RATE) {
		dev_err(imx563->dev, "inclk frequency mismatch\n");
		return -EINVAL;
	}

	/* Get optional DT defined regulators */
	for (i = 0; i < ARRAY_SIZE(imx563_supply_names); i++)
		imx563->supplies[i].supply = imx563_supply_names[i];

	ret = devm_regulator_bulk_get(imx563->dev,
				      ARRAY_SIZE(imx563_supply_names),
				      imx563->supplies);
	if (ret)
		return ret;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -ENXIO;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != IMX563_NUM_DATA_LANES) {
		dev_err(imx563->dev,
			"number of CSI2 data lanes %d is not supported\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		dev_err(imx563->dev, "no link frequencies defined\n");
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++)
		if (bus_cfg.link_frequencies[i] == IMX563_LINK_FREQ)
			goto done_endpoint_free;

	ret = -EINVAL;

done_endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

/* V4l2 subdevice ops */
static const struct v4l2_subdev_video_ops imx563_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops imx563_pad_ops = {
	.enum_mbus_code = imx563_enum_mbus_code,
	.enum_frame_size = imx563_enum_frame_size,
	.get_fmt = imx563_get_pad_format,
	.set_fmt = imx563_set_pad_format,
	.enable_streams = imx563_enable_streams,
	.disable_streams = imx563_disable_streams,
};

static const struct v4l2_subdev_ops imx563_subdev_ops = {
	.video = &imx563_video_ops,
	.pad = &imx563_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx563_internal_ops = {
	.init_state = imx563_init_state,
};

static int imx563_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx563 *imx563 = to_imx563(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(imx563_supply_names),
				    imx563->supplies);
	if (ret < 0) {
		dev_err(dev, "failed to enable regulators\n");
		return ret;
	}

	usleep_range(1000, 2000);

	ret = clk_prepare_enable(imx563->inclk);
	if (ret) {
		dev_err(imx563->dev, "fail to enable inclk\n");
		goto error_reset;
	}

	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(imx563->reset_gpio, 0);
	usleep_range(10000, 12000);

	return 0;

error_reset:
	gpiod_set_value_cansleep(imx563->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(imx563_supply_names),
			       imx563->supplies);

	return ret;
}

static int imx563_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx563 *imx563 = to_imx563(sd);

	gpiod_set_value_cansleep(imx563->reset_gpio, 1);
	clk_disable_unprepare(imx563->inclk);

	regulator_bulk_disable(ARRAY_SIZE(imx563_supply_names),
			       imx563->supplies);

	return 0;
}

/**
 * imx563_init_controls() - Initialize sensor subdevice controls
 * @imx563: pointer to imx563 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx563_init_controls(struct imx563 *imx563)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &imx563->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	const struct imx563_mode *mode = imx563->cur_mode;
	u32 lpfr;
	int ret;

	ret = v4l2_fwnode_device_parse(imx563->dev, &props);
	if (ret)
		return dev_err_probe(imx563->dev, ret,
				     "failed to parse firmware properties\n");

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 8);
	if (ret)
		return ret;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx563_ctrl_ops,
					     &props);
	if (ret)
		goto error;

	/* Initialize exposure and gain */
	lpfr = mode->vblank + mode->height;
	imx563->exp_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
					     &imx563_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX563_EXPOSURE_MIN,
					     lpfr - IMX563_EXPOSURE_OFFSET,
					     IMX563_EXPOSURE_STEP,
					     IMX563_EXPOSURE_DEFAULT);

	imx563->again_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
					       &imx563_ctrl_ops,
					       V4L2_CID_ANALOGUE_GAIN,
					       IMX563_AGAIN_MIN,
					       IMX563_AGAIN_MAX,
					       IMX563_AGAIN_STEP,
					       IMX563_AGAIN_DEFAULT);

	v4l2_ctrl_cluster(2, &imx563->exp_ctrl);

	imx563->vblank_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						&imx563_ctrl_ops,
						V4L2_CID_VBLANK,
						mode->vblank_min,
						mode->vblank_max,
						1, mode->vblank);

	/* Read only controls */
	imx563->pclk_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
					      &imx563_ctrl_ops,
					      V4L2_CID_PIXEL_RATE,
					      mode->pclk, mode->pclk,
					      1, mode->pclk);

	imx563->link_freq_ctrl = v4l2_ctrl_new_int_menu(ctrl_hdlr,
							&imx563_ctrl_ops,
							V4L2_CID_LINK_FREQ,
							ARRAY_SIZE(link_freq) -
							1,
							mode->link_freq_idx,
							link_freq);
	if (imx563->link_freq_ctrl)
		imx563->link_freq_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx563->hblank_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						&imx563_ctrl_ops,
						V4L2_CID_HBLANK,
						IMX563_REG_MIN,
						IMX563_REG_MAX,
						1, mode->hblank);
	if (imx563->hblank_ctrl)
		imx563->hblank_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(imx563->dev, "control init failed: %d\n", ret);
		goto error;
	}

	imx563->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	return ret;
}

static int imx563_probe(struct i2c_client *client)
{
	struct imx563 *imx563;
	const char *name;
	int ret;

	imx563 = devm_kzalloc(&client->dev, sizeof(*imx563), GFP_KERNEL);
	if (!imx563)
		return -ENOMEM;

	imx563->dev = &client->dev;
	name = device_get_match_data(&client->dev);
	if (!name)
		return -ENODEV;

	imx563->cci = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx563->cci))
		return dev_err_probe(imx563->dev, PTR_ERR(imx563->cci),
				     "Failed to init CCI\n");

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&imx563->sd, client, &imx563_subdev_ops);
	imx563->sd.internal_ops = &imx563_internal_ops;

	ret = imx563_parse_hw_config(imx563);
	if (ret) {
		dev_err(imx563->dev, "HW configuration is not supported\n");
		return ret;
	}

	ret = imx563_power_on(imx563->dev);
	if (ret)
		return dev_err_probe(imx563->dev, ret,
				     "failed to power-on the sensor\n");

	/* Check module identity */
	ret = imx563_detect(imx563);
	if (ret) {
		dev_err(imx563->dev, "failed to find sensor: %d\n", ret);
		goto error_power_off;
	}

	/* Set default mode to max resolution */
	imx563->cur_mode = &supported_mode;
	imx563->vblank = imx563->cur_mode->vblank;

	ret = imx563_init_controls(imx563);
	if (ret) {
		dev_err(imx563->dev, "failed to init controls: %d\n", ret);
		goto error_power_off;
	}

	/* Initialize subdev */
	imx563->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx563->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	v4l2_i2c_subdev_set_name(&imx563->sd, client, name, NULL);

	/* Initialize source pad */
	imx563->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&imx563->sd.entity, 1, &imx563->pad);
	if (ret) {
		dev_err(imx563->dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	imx563->sd.state_lock = imx563->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&imx563->sd);
	if (ret < 0) {
		dev_err_probe(imx563->dev, ret, "subdev init error\n");
		goto error_media_entity;
	}

	pm_runtime_set_active(imx563->dev);
	pm_runtime_enable(imx563->dev);

	ret = v4l2_async_register_subdev_sensor(&imx563->sd);
	if (ret < 0) {
		dev_err_probe(imx563->dev, ret,
			      "failed to register sub-device\n");
		goto error_subdev_cleanup;
	}

	pm_runtime_idle(imx563->dev);

	return 0;

error_subdev_cleanup:
	v4l2_subdev_cleanup(&imx563->sd);
	pm_runtime_disable(imx563->dev);
	pm_runtime_set_suspended(imx563->dev);
error_media_entity:
	media_entity_cleanup(&imx563->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(imx563->sd.ctrl_handler);
error_power_off:
	imx563_power_off(imx563->dev);

	return ret;
}

static void imx563_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx563_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct dev_pm_ops imx563_pm_ops = {
	SET_RUNTIME_PM_OPS(imx563_power_off, imx563_power_on, NULL)
};

static const struct of_device_id imx563_of_match[] = {
	{ .compatible = "sony,imx563", .data = "imx563" },
	{ }
};

MODULE_DEVICE_TABLE(of, imx563_of_match);

static struct i2c_driver imx563_driver = {
	.probe = imx563_probe,
	.remove = imx563_remove,
	.driver = {
		.name = "imx563",
		.pm = &imx563_pm_ops,
		.of_match_table = imx563_of_match,
	},
};

module_i2c_driver(imx563_driver);

MODULE_DESCRIPTION("Sony IMX563 sensor driver");
MODULE_LICENSE("GPL");
