// SPDX-License-Identifier: GPL-2.0-only
/* DRM panel driver for the Meizu 20 (m2381) BOE AMOLED panel (PanelID
 * B5011A05, FHD+ 1080x2400, DSC command mode).
 *
 * Skeleton: panel-nt37290-boe.c (Chalkin, Meizu 20 Inf sister device).
 * Init sequence: vendor DTB panel-boe-cmd.dts on-command stream
 * (meizu20-m1/tools/panel-boe-meizu20-seq.h), decoded verbatim.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/workqueue.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_drv.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_modeset_lock.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct boe_meizu20 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct drm_connector *connector;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data *supplies;
	struct delayed_work recovery_work;
	bool recovery_in_progress;
	bool recovery_scheduled;
	bool keep_rails;
	bool panel_enabled;
};

static const struct regulator_bulk_data boe_meizu20_supplies[] = {
	{ .supply = "vddio" },
	{ .supply = "vdd" },
	{ .supply = "vci" },
};

enum boe_meizu20_supply_index {
	BOE_MEIZU20_VDDIO,
	BOE_MEIZU20_VDD,
	BOE_MEIZU20_VCI,
};

static const int boe_meizu20_supply_loads[] = {
	[BOE_MEIZU20_VDDIO] = 200000,
	[BOE_MEIZU20_VDD] = 299000,
	[BOE_MEIZU20_VCI] = 10000,
};


struct boe_meizu20_cmd {
	u8 dt;
	u16 delay;
	u8 len;
	u8 data[24];
};

/* auto-generated from tools/panel-boe-meizu20-seq.h (32 records, vendor panel-boe-cmd.dts) */
static const struct boe_meizu20_cmd boe_meizu20_init_cmds[] = {
	{ .dt = 0x39, .delay = 0, .len = 5, .data = { 0xff, 0xaa, 0x55, 0xa5, 0x80 } },
	{ .dt = 0x15, .delay = 0, .len = 2, .data = { 0x6f, 0x1d } },
	{ .dt = 0x15, .delay = 0, .len = 2, .data = { 0xf2, 0x05 } },
	{ .dt = 0x39, .delay = 0, .len = 5, .data = { 0xff, 0xaa, 0x55, 0xa5, 0x00 } },
	{ .dt = 0x05, .delay = 0, .len = 1, .data = { 0x35 } },
	{ .dt = 0x15, .delay = 0, .len = 2, .data = { 0x53, 0x20 } },
	{ .dt = 0x39, .delay = 0, .len = 5, .data = { 0x51, 0x00, 0x00, 0x00, 0x00 } },
	{ .dt = 0x39, .delay = 0, .len = 5, .data = { 0x2a, 0x00, 0x00, 0x04, 0x37 } },
	{ .dt = 0x39, .delay = 0, .len = 5, .data = { 0x2b, 0x00, 0x00, 0x09, 0x5f } },
	{ .dt = 0x15, .delay = 0, .len = 2, .data = { 0x82, 0xb0 } },
	{ .dt = 0x15, .delay = 0, .len = 2, .data = { 0x8b, 0x80 } },
	{ .dt = 0x15, .delay = 0, .len = 2, .data = { 0x03, 0x01 } },
	{ .dt = 0x15, .delay = 0, .len = 2, .data = { 0x90, 0x11 } },
	{ .dt = 0x39, .delay = 0, .len = 19, .data = { 0x91, 0xab, 0x28, 0x00, 0x14, 0xc2, 0x00, 0x03, 0x1c, 0x02, 0x8c, 0x00, 0x0f, 0x05, 0x0e, 0x02, 0x8b, 0x10, 0xf0 } },
	{ .dt = 0x05, .delay = 0, .len = 1, .data = { 0x2c } },
	{ .dt = 0x39, .delay = 0, .len = 6, .data = { 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x01 } },
	{ .dt = 0x39, .delay = 0, .len = 3, .data = { 0xcd, 0x05, 0x51 } },
	{ .dt = 0x39, .delay = 0, .len = 6, .data = { 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x07 } },
	{ .dt = 0x39, .delay = 0, .len = 2, .data = { 0xc0, 0x00 } },
	{ .dt = 0x39, .delay = 0, .len = 6, .data = { 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x04 } },
	{ .dt = 0x39, .delay = 0, .len = 2, .data = { 0xd0, 0xa3 } },
	{ .dt = 0x15, .delay = 0, .len = 2, .data = { 0x2f, 0x02 } },
	{ .dt = 0x15, .delay = 120, .len = 2, .data = { 0x11, 0x00 } },
	{ .dt = 0x15, .delay = 0, .len = 2, .data = { 0x29, 0x00 } },
	{ .dt = 0x39, .delay = 0, .len = 6, .data = { 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x08 } },
	{ .dt = 0x39, .delay = 0, .len = 6, .data = { 0xe1, 0xe1, 0x73, 0x73, 0x73, 0x00 } },
	{ .dt = 0x39, .delay = 0, .len = 21, .data = { 0xe2, 0x45, 0x31, 0x34, 0xa4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00 } },
	{ .dt = 0x39, .delay = 0, .len = 21, .data = { 0xe3, 0x45, 0x32, 0x33, 0xa6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00 } },
	{ .dt = 0x39, .delay = 0, .len = 21, .data = { 0xe4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
	{ .dt = 0x39, .delay = 0, .len = 19, .data = { 0xe5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
	{ .dt = 0x39, .delay = 0, .len = 19, .data = { 0xe6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
	{ .dt = 0x39, .delay = 0, .len = 19, .data = { 0xe7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
};

static inline struct boe_meizu20 *to_boe_meizu20(struct drm_panel *panel)
{
	return container_of(panel, struct boe_meizu20, panel);
}

static int boe_meizu20_power_on(struct boe_meizu20 *ctx)
{
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(boe_meizu20_supplies); i++) {
		ret = regulator_set_load(ctx->supplies[i].consumer,
					 boe_meizu20_supply_loads[i]);
		if (ret < 0)
			goto clear_loads;
	}

	ret = regulator_enable(ctx->supplies[BOE_MEIZU20_VDDIO].consumer);
	if (ret)
		goto clear_loads;
	mdelay(2);

	ret = regulator_enable(ctx->supplies[BOE_MEIZU20_VDD].consumer);
	if (ret)
		goto disable_vddio;
	mdelay(5);

	ret = regulator_enable(ctx->supplies[BOE_MEIZU20_VCI].consumer);
	if (ret)
		goto disable_vdd;

	return 0;

disable_vdd:
	regulator_disable(ctx->supplies[BOE_MEIZU20_VDD].consumer);
disable_vddio:
	regulator_disable(ctx->supplies[BOE_MEIZU20_VDDIO].consumer);
	clear_loads:
	while (--i >= 0)
		regulator_set_load(ctx->supplies[i].consumer, 0);
	return ret;
}

static void boe_meizu20_power_off(struct boe_meizu20 *ctx)
{
	regulator_disable(ctx->supplies[BOE_MEIZU20_VCI].consumer);
	regulator_disable(ctx->supplies[BOE_MEIZU20_VDD].consumer);
	regulator_disable(ctx->supplies[BOE_MEIZU20_VDDIO].consumer);
	regulator_set_load(ctx->supplies[BOE_MEIZU20_VCI].consumer, 0);
	regulator_set_load(ctx->supplies[BOE_MEIZU20_VDD].consumer, 0);
	regulator_set_load(ctx->supplies[BOE_MEIZU20_VDDIO].consumer, 0);
}

static void boe_meizu20_reset(struct boe_meizu20 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	mdelay(5);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	mdelay(2);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	mdelay(10);
}

static void boe_meizu20_recovery_work(struct work_struct *work)
{
	struct boe_meizu20 *ctx =
		container_of(to_delayed_work(work), struct boe_meizu20,
			     recovery_work);
	struct drm_connector *connector = ctx->connector;
	struct drm_modeset_acquire_ctx acquire_ctx;
	struct drm_atomic_commit *state;
	struct drm_device *drm;
	int ret;

	if (!connector)
		return;

	drm = connector->dev;
	DRM_MODESET_LOCK_ALL_BEGIN(drm, acquire_ctx, 0, ret);

	state = drm_atomic_helper_duplicate_state(drm, &acquire_ctx);
	if (IS_ERR(state)) {
		ret = PTR_ERR(state);
	} else {
		WRITE_ONCE(ctx->recovery_in_progress, true);
		ret = drm_atomic_helper_disable_all(drm, &acquire_ctx);
		if (ret)
			goto stop_recovery;

		msleep(20);
		ret = drm_atomic_helper_commit_duplicated_state(state,
							 &acquire_ctx);
stop_recovery:
		WRITE_ONCE(ctx->recovery_in_progress, false);
		drm_atomic_commit_put(state);
	}

	DRM_MODESET_LOCK_ALL_END(drm, acquire_ctx, ret);

	if (ret)
		dev_err(&ctx->dsi->dev, "failed to restart display after cold enable: %d\n",
			ret);
	else
		dev_info(&ctx->dsi->dev, "restarted display after failed cold enable\n");
}

/*
 * Index of the sleep-out entry (0x11, 120 ms) in the vendor table below; the
 * entry right after it is display-on (0x29).
 *
 * The DDIC has to be told about DSC (PPS + compression mode) *before* it
 * leaves sleep and starts scanning out: if display-on comes first it decodes
 * the compressed stream with the default (uncompressed) setup, which shows up
 * as screen-wide speckle noise after every power-cycle re-init.  Cold boot only
 * survived because the recovery path re-ran the whole sequence.  Same ordering
 * as the upstream DSC command-mode reference (panel-visionox-r66451).
 */
#define BOE_MEIZU20_SEQ_DSC_SPLIT	22

/* Send vendor table entries [first, last). */
static int boe_meizu20_send_seq(struct boe_meizu20 *ctx, unsigned int first,
				unsigned int last)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	unsigned int i;

	for (i = first; i < last && i < ARRAY_SIZE(boe_meizu20_init_cmds); i++) {
		const struct boe_meizu20_cmd *c = &boe_meizu20_init_cmds[i];
		int ret;

		ret = mipi_dsi_dcs_write(dsi, c->data[0],
					 c->len > 1 ? &c->data[1] : NULL,
					 c->len - 1);
		if (ret < 0) {
			dev_err(&dsi->dev, "init cmd %u (0x%02x) failed: %d\n",
				i, c->data[0], ret);
			return ret;
		}
		if (c->delay)
			msleep(c->delay);
	}

	return 0;
}

static int boe_meizu20_prepare(struct drm_panel *panel)
{
	struct boe_meizu20 *ctx = to_boe_meizu20(panel);
	int ret;

	if (READ_ONCE(ctx->recovery_in_progress))
		return 0;

	/*
	 * M3: the previous unprepare() left the rails up and the panel in
	 * sleep, so the DDIC session state is intact.  Skip the reset /
	 * power-cycle entirely - that cycle is what loses the DDIC's
	 * block-compensation state (fixed-position mosaic, see patch 0010).
	 * enable() replays the full init sequence over the live panel, the
	 * same shape the recovery path already proves is fine.
	 */
	if (READ_ONCE(ctx->keep_rails)) {
		WRITE_ONCE(ctx->keep_rails, false);
		return 0;
	}

	/* Match unprepare(): hold reset asserted while bringing up the rails. */
	ret = gpiod_direction_output(ctx->reset_gpio, 1);
	if (ret)
		return ret;
	usleep_range(2000, 3000);

	ret = boe_meizu20_power_on(ctx);
	if (ret < 0)
		return ret;

	boe_meizu20_reset(ctx);

	return 0;
}

static int boe_meizu20_enable(struct drm_panel *panel)
{
	struct boe_meizu20 *ctx = to_boe_meizu20(panel);
	struct device *dev = &ctx->dsi->dev;
	struct drm_dsc_picture_parameter_set pps;
	unsigned long mode_flags;
	u8 power_mode = 0;
	int ret;

	ret = boe_meizu20_send_seq(ctx, 0, BOE_MEIZU20_SEQ_DSC_SPLIT);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel (pre-DSC part): %d\n", ret);
		return ret;
	}

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);

	ret = mipi_dsi_picture_parameter_set(ctx->dsi, &pps);
	if (ret < 0) {
		dev_err(dev, "failed to transmit PPS: %d\n", ret);
		return ret;
	}
	ret = mipi_dsi_compression_mode(ctx->dsi, true);
	if (ret < 0) {
		dev_err(dev, "failed to enable compression mode: %d\n", ret);
		return ret;
	}
	/* Give the DDIC a moment to latch the PPS before it leaves sleep. */
	mdelay(5);

	ret = boe_meizu20_send_seq(ctx, BOE_MEIZU20_SEQ_DSC_SPLIT,
				   ARRAY_SIZE(boe_meizu20_init_cmds));
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel (post-DSC part): %d\n", ret);
		return ret;
	}
	WRITE_ONCE(ctx->panel_enabled, true);

	/* The stream ends with 0x29 (display on); give the DDIC its settle. */
	mdelay(20);

	/* Cold-enable sanity check: the panel must report on+normal state. */
	mode_flags = ctx->dsi->mode_flags;
	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_get_power_mode(ctx->dsi, &power_mode);
	ctx->dsi->mode_flags = mode_flags;
	if (!ctx->recovery_scheduled &&
	    (ret < 0 ||
	     (power_mode & (MIPI_DSI_DCS_POWER_MODE_DISPLAY |
			    MIPI_DSI_DCS_POWER_MODE_NORMAL |
			    MIPI_DSI_DCS_POWER_MODE_SLEEP)) !=
	     (MIPI_DSI_DCS_POWER_MODE_DISPLAY |
	      MIPI_DSI_DCS_POWER_MODE_NORMAL |
	      MIPI_DSI_DCS_POWER_MODE_SLEEP))) {
		ctx->recovery_scheduled = true;
		dev_warn(dev,
			 "cold enable power mode invalid (ret=%d, mode=0x%02x), scheduling display restart\n",
			 ret, power_mode);
		schedule_delayed_work(&ctx->recovery_work, msecs_to_jiffies(100));
	}

	return 0;
}

static int boe_meizu20_unprepare(struct drm_panel *panel)
{
	struct boe_meizu20 *ctx = to_boe_meizu20(panel);
	struct device *dev = &ctx->dsi->dev;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	int ret;

	if (READ_ONCE(ctx->recovery_in_progress))
		return 0;

	WRITE_ONCE(ctx->panel_enabled, false);

	ret = mipi_dsi_dcs_set_display_off(ctx->dsi);
	if (ret < 0)
		dev_err(dev, "Failed to disable panel: %d\n", ret);
	else
		usleep_range(5000, 6000);

	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	/*
	 * M3: leave the rails up and reset untouched.  Reset or power-cut
	 * clears the DDIC's block-compensation (demura/IRC class) session
	 * state; the next enable() then shows a fixed-position block mosaic
	 * that does not scroll with content (2026-09-29, E1 round).  Sleep
	 * mode alone preserves it - both known-good paths (bootloader
	 * session, our recovery path) keep the panel powered.  Cost: the
	 * panel keeps its sleep-mode current while the screen is off.
	 */
	WRITE_ONCE(ctx->keep_rails, true);

	return 0;
}

static const struct drm_display_mode boe_meizu20_modes[] = {
	{
		/* qcom,mdss-dsi timing@0: h/v porches 50/16/50, 20/4/12 */
		.clock = (1080 + 50 + 16 + 50) * (2400 + 20 + 4 + 12) * 120 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 50,
		.hsync_end = 1080 + 50 + 16,
		.htotal = 1080 + 50 + 16 + 50,
		.vdisplay = 2400,
		.vsync_start = 2400 + 20,
		.vsync_end = 2400 + 20 + 4,
		.vtotal = 2400 + 20 + 4 + 12,
		.width_mm = 68,
		.height_mm = 152,
		.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
	},
};

static int boe_meizu20_get_modes(struct drm_panel *panel,
				 struct drm_connector *connector)
{
	struct boe_meizu20 *ctx = to_boe_meizu20(panel);
	unsigned int i;
	int count = 0;

	ctx->connector = connector;
	connector->display_info.width_mm = 68;
	connector->display_info.height_mm = 152;

	for (i = 0; i < ARRAY_SIZE(boe_meizu20_modes); i++) {
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, &boe_meizu20_modes[i]);
		if (!mode)
			continue;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
		count++;
	}

	return count;
}

static const struct drm_panel_funcs boe_meizu20_panel_funcs = {
	.prepare = boe_meizu20_prepare,
	.enable = boe_meizu20_enable,
	.unprepare = boe_meizu20_unprepare,
	.get_modes = boe_meizu20_get_modes,
};

static int boe_meizu20_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct boe_meizu20 *ctx = mipi_dsi_get_drvdata(dsi);
	unsigned long mode_flags = dsi->mode_flags;
	u16 brightness = backlight_get_brightness(bl);
	unsigned int i;
	int ret;

	/* The panel is powered off by DPMS blanking; a DCS write over a
	 * dead link wedges the DSI host.  Backlight state is restored by
	 * enable() instead. */
	if (!READ_ONCE(ctx->panel_enabled))
		return 0;

	/* Our DSI host times out HS command writes early in the enable
	 * path; command-mode panels accept DCS in LP.  The first write after
	 * a DPMS cycle can additionally race the DPU's first command-mode
	 * frame transfer, so retry a couple of times before giving up. */
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	for (i = 0; i < 3; i++) {
		ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
		if (ret != -ETIMEDOUT)
			break;
		msleep(20);
	}

	dsi->mode_flags = mode_flags;

	return ret;
}

static const struct backlight_ops boe_meizu20_bl_ops = {
	.update_status = boe_meizu20_bl_update_status,
};

static struct backlight_device *
boe_meizu20_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 1200,
		.max_brightness = 3515, /* 0x0dbb, inf sibling panel max */
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &boe_meizu20_bl_ops, &props);
}

static int boe_meizu20_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct boe_meizu20 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct boe_meizu20, panel,
				   &boe_meizu20_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = devm_regulator_bulk_get_const(&dsi->dev,
					    ARRAY_SIZE(boe_meizu20_supplies),
					    boe_meizu20_supplies,
					    &ctx->supplies);
	if (ret < 0)
		return ret;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	INIT_DELAYED_WORK(&ctx->recovery_work, boe_meizu20_recovery_work);
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB101010;
	dsi->mode_flags = MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS |
			  MIPI_DSI_MODE_DSC_ALL_SLICES_IN_PKT;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = boe_meizu20_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	/* This panel only supports DSC; unconditionally enable it */
	dsi->dsc = &ctx->dsc;

	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 2;

	ctx->dsc.slice_height = 20;
	ctx->dsc.slice_width = 540;
	WARN_ON(1080 % ctx->dsc.slice_width);
	ctx->dsc.slice_count = 1080 / ctx->dsc.slice_width;
	ctx->dsc.bits_per_component = 10;
	ctx->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	ctx->dsc.block_pred_enable = true;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void boe_meizu20_remove(struct mipi_dsi_device *dsi)
{
	struct boe_meizu20 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	cancel_delayed_work_sync(&ctx->recovery_work);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id boe_meizu20_of_match[] = {
	{ .compatible = "boe,meizu20-amoled-fhd" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, boe_meizu20_of_match);

static struct mipi_dsi_driver boe_meizu20_driver = {
	.probe = boe_meizu20_probe,
	.remove = boe_meizu20_remove,
	.driver = {
		.name = "panel-boe-meizu20",
		.of_match_table = boe_meizu20_of_match,
	},
};
module_mipi_dsi_driver(boe_meizu20_driver);

MODULE_AUTHOR("ri");
MODULE_DESCRIPTION("DRM panel driver for BOE B5011A05 (Meizu 20 m2381)");
MODULE_LICENSE("GPL");
