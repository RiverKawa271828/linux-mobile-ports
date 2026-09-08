// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026

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
#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct nt37290_boe {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct drm_connector *connector;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data *supplies;
	struct delayed_work recovery_work;
	bool recovery_scheduled;
	bool firmware_handoff_pending;
	bool firmware_handoff_active;
};

static const struct regulator_bulk_data nt37290_boe_supplies[] = {
	{ .supply = "vddio" },
	{ .supply = "vdd" },
	{ .supply = "vci" },
};

enum nt37290_boe_supply_index {
	NT37290_BOE_VDDIO,
	NT37290_BOE_VDD,
	NT37290_BOE_VCI,
};

/* qcom,panel-supply-entries 0x709 from the Android device tree. */
static const int nt37290_boe_supply_loads[] = {
	[NT37290_BOE_VDDIO] = 200000,
	[NT37290_BOE_VDD] = 299000,
	[NT37290_BOE_VCI] = 10000,
};

static inline struct nt37290_boe *to_nt37290_boe(struct drm_panel *panel)
{
	return container_of(panel, struct nt37290_boe, panel);
}

static int nt37290_boe_power_on(struct nt37290_boe *ctx)
{
	int i;
	int ret;

	/* Match dsi_panel_pwr_supply (phandle 0x709) in the Android DTS. */
	for (i = 0; i < ARRAY_SIZE(nt37290_boe_supplies); i++) {
		ret = regulator_set_load(ctx->supplies[i].consumer,
					 nt37290_boe_supply_loads[i]);
		if (ret < 0)
			goto clear_loads;
	}

	ret = regulator_enable(ctx->supplies[NT37290_BOE_VDDIO].consumer);
	if (ret)
		goto clear_loads;
	mdelay(2);

	mdelay(2);
	ret = regulator_enable(ctx->supplies[NT37290_BOE_VDD].consumer);
	if (ret)
		goto disable_vddio;
	mdelay(5);

	ret = regulator_enable(ctx->supplies[NT37290_BOE_VCI].consumer);
	if (ret)
		goto disable_vdd;

	return 0;

disable_vdd:
	regulator_disable(ctx->supplies[NT37290_BOE_VDD].consumer);
disable_vddio:
	regulator_disable(ctx->supplies[NT37290_BOE_VDDIO].consumer);
	clear_loads:
	while (--i >= 0)
		regulator_set_load(ctx->supplies[i].consumer, 0);
	return ret;
}

static void nt37290_boe_power_off(struct nt37290_boe *ctx)
{
	regulator_disable(ctx->supplies[NT37290_BOE_VCI].consumer);
	regulator_disable(ctx->supplies[NT37290_BOE_VDD].consumer);
	regulator_disable(ctx->supplies[NT37290_BOE_VDDIO].consumer);
	regulator_set_load(ctx->supplies[NT37290_BOE_VCI].consumer, 0);
	regulator_set_load(ctx->supplies[NT37290_BOE_VDD].consumer, 0);
	regulator_set_load(ctx->supplies[NT37290_BOE_VDDIO].consumer, 0);
}

static void nt37290_boe_reset(struct nt37290_boe *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	mdelay(5);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	mdelay(2);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	mdelay(10);
}

static unsigned int nt37290_boe_get_refresh_rate(struct nt37290_boe *ctx)
{
	const struct drm_connector_state *conn_state;

	if (!ctx->connector)
		return 120;

	conn_state = ctx->connector->state;
	if (!conn_state || !conn_state->crtc || !conn_state->crtc->state)
		return 120;

	return drm_mode_vrefresh(&conn_state->crtc->state->adjusted_mode);
}

static int nt37290_boe_on(struct nt37290_boe *ctx, unsigned int refresh_rate)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0,
				     0x55, 0xaa, 0x52, 0x08, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x00, 0x0b, 0x0b, 0x0b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0,
				     0x55, 0xaa, 0x52, 0x08, 0x07);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb4, 0x78, 0x43, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0,
				     0x55, 0xaa, 0x52, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0xa5, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x1b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf4, 0x55);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0xa5, 0x81);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x07);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0,
				     0x55, 0xaa, 0x52, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x35, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x53, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x8f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2a, 0x00, 0x00, 0x05, 0x57);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2b, 0x00, 0x00, 0x0c, 0x77);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x26, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1f, 0xf0);
	if (refresh_rate < 120) {
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0,
					     0x55, 0xaa, 0x52, 0x08, 0x00);
		if (refresh_rate >= 60)
			mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbc, 0x20);
		else
			mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbc, 0x60);
	}
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5a, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x51, 0x0d, 0xbb, 0x0d, 0xbb);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x81, 0x01, 0x19);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x90, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x91,
				     0xab, 0xa8, 0x00, 0x18, 0xd2, 0x00, 0x02,
				     0x73, 0x02, 0x75, 0x00, 0x09, 0x04, 0x86,
				     0x03, 0x59, 0x10, 0xf0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x93,
				     0xab, 0xa8, 0x00, 0x10, 0xc2, 0x00, 0x01,
				     0xe4, 0x01, 0x63, 0x00, 0x06, 0x06, 0x67,
				     0x07, 0x87, 0x10, 0xf0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9c, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2f, 0x00);
	if (refresh_rate < 120) {
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0,
					     0x55, 0xaa, 0x52, 0x08, 0x00);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x1c);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xba,
					     0x91, 0x01, 0x01, 0x00,
					     0x01, 0x01, 0x01, 0x00);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2c);
	}
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x03, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0,
				     0x55, 0xaa, 0x52, 0x08, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbe, 0x47, 0x4f, 0x4a, 0x49);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x09);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbe, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0,
				     0x55, 0xaa, 0x52, 0x08, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc4,
				     0x00, 0x00, 0x00, 0x00, 0x70, 0x10, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70,
				     0x10, 0x00, 0x00, 0x70, 0x10, 0x00, 0x00,
				     0x70, 0x10, 0x00, 0x00, 0x70, 0x10, 0x00,
				     0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0,
				     0x55, 0xaa, 0x52, 0x08, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x87, 0x27, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb6,
				     0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x86,
				     0x82, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0xa5, 0x84);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x1b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf8, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0,
				     0x55, 0xaa, 0x52, 0x08, 0x00);
	if (refresh_rate >= 120)
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbc, 0x00);
	else if (refresh_rate >= 60)
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbc, 0x20);
	else
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbc, 0x60);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5a, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0,
				     0x55, 0xaa, 0x52, 0x08, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3, 0x82);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3, 0x82);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0,
				     0x55, 0xaa, 0x52, 0x08, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3,
				     0x99, 0x04, 0x20, 0x15, 0xff, 0x00, 0x04,
				     0x20, 0x0e, 0xff, 0x00, 0x06, 0x39, 0x1f,
				     0x06, 0x45, 0x19, 0x06, 0x39, 0x1f, 0x06,
				     0x45, 0x19, 0x06, 0x39, 0x1f, 0x06, 0x45,
				     0x19, 0x06, 0x39, 0x1f, 0x06, 0x45, 0x19,
				     0x06, 0x39, 0x1f, 0x06, 0x45, 0x19);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0,
				     0x55, 0xaa, 0x52, 0x08, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x06);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xec, 0xcc);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0xa5, 0x81);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf5, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0,
				     0x55, 0xaa, 0x52, 0x08, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x37);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe2, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x38);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe2, 0xff, 0xff, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x57);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe2,
				     0x2a, 0x2a, 0x2a, 0x2a, 0x3a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0,
				     0x55, 0xaa, 0x52, 0x08, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x07);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb7,
				     0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x51, 0x00, 0x00);

	return dsi_ctx.accum_err;
}

static int nt37290_boe_sleep_in(struct nt37290_boe *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int nt37290_boe_prepare(struct drm_panel *panel)
{
	struct nt37290_boe *ctx = to_nt37290_boe(panel);
	int ret;

	/* Match unprepare(): hold reset asserted while bringing up the rails. */
	if (!ctx->firmware_handoff_pending) {
		ret = gpiod_direction_output(ctx->reset_gpio, 1);
		if (ret)
			return ret;
		usleep_range(2000, 3000);
	}

	ret = nt37290_boe_power_on(ctx);
	if (ret < 0)
		return ret;

	/* Keep the firmware scanout alive until the DSI host can send DCS. */
	if (ctx->firmware_handoff_pending)
		return 0;

	nt37290_boe_reset(ctx);

	return 0;
}

static int nt37290_boe_enable(struct drm_panel *panel)
{
	struct nt37290_boe *ctx = to_nt37290_boe(panel);
	struct device *dev = &ctx->dsi->dev;
	struct drm_dsc_picture_parameter_set pps;
	unsigned int refresh_rate = nt37290_boe_get_refresh_rate(ctx);
	unsigned long mode_flags;
	u8 power_mode = 0;
	int ret;

	if (ctx->firmware_handoff_pending) {
		/*
		 * UEFI left this command-mode panel enabled and displaying the
		 * splash buffer.  The first atomic commit only has to replace the
		 * DPU scanout; resetting or reinitializing the panel here destroys
		 * that working state before Linux has completed the takeover.
		 *
		 * Drop the handoff flag now so the first real DPMS cycle uses the
		 * normal cold prepare/enable sequence below.
		 */
		ctx->firmware_handoff_pending = false;
		ctx->firmware_handoff_active = true;
		return 0;
	}

	ret = nt37290_boe_on(ctx, refresh_rate);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		return ret;
	}
	/* Downstream sends the panel on sequence in HS mode. */
	ret = mipi_dsi_dcs_exit_sleep_mode(ctx->dsi);
	if (ret < 0) {
		dev_err(dev, "failed to exit sleep mode: %d\n", ret);
		return ret;
	}

	/* Cold DRM takeover can run before schedulable sleeps are dependable. */
	mdelay(120);

	ret = mipi_dsi_dcs_set_display_on(ctx->dsi);
	if (ret < 0) {
		dev_err(dev, "failed to enable display: %d\n", ret);
		return ret;
	}

	mdelay(20);

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);

	ret = mipi_dsi_picture_parameter_set(ctx->dsi, &pps);
	if (ret < 0) {
		dev_err(panel->dev, "failed to transmit PPS: %d\n", ret);
		return ret;
	}
	ret = mipi_dsi_compression_mode(ctx->dsi, true);
	if (ret < 0) {
		dev_err(dev, "failed to enable compression mode: %d\n", ret);
		return ret;
	}
	mdelay(28);

	mode_flags = ctx->dsi->mode_flags;
	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	power_mode = 0;
	ret = mipi_dsi_dcs_get_power_mode(ctx->dsi, &power_mode);
	ctx->dsi->mode_flags = mode_flags;

	if (ret < 0 && !ctx->recovery_scheduled) {
		ctx->recovery_scheduled = true;
		schedule_delayed_work(&ctx->recovery_work,
				      msecs_to_jiffies(250));
	}

	return 0;
}

static void nt37290_boe_recovery_work(struct work_struct *work)
{
	struct nt37290_boe *ctx = container_of(to_delayed_work(work),
						struct nt37290_boe, recovery_work);
	struct drm_device *drm;
	struct drm_atomic_state *state;
	int ret;

	if (!ctx->connector)
		return;

	drm = ctx->connector->dev;
	state = drm_atomic_helper_suspend(drm);
	if (IS_ERR(state)) {
		dev_err(&ctx->dsi->dev,
			"failed to suspend atomic state for recovery: %ld\n",
			PTR_ERR(state));
		return;
	}

	msleep(20);
	ret = drm_atomic_helper_resume(drm, state);
	if (ret)
		dev_err(&ctx->dsi->dev,
			"failed to resume atomic state during recovery: %d\n", ret);
}

static int nt37290_boe_unprepare(struct drm_panel *panel)
{
	struct nt37290_boe *ctx = to_nt37290_boe(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ctx->firmware_handoff_active = false;

	ret = mipi_dsi_dcs_set_display_off(ctx->dsi);
	if (ret < 0)
		dev_err(dev, "Failed to disable panel: %d\n", ret);
	else
		usleep_range(5000, 6000);

	ret = nt37290_boe_sleep_in(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	nt37290_boe_power_off(ctx);

	return 0;
}

static const struct drm_display_mode nt37290_boe_modes[] = {
	{
		.clock = (1368 + 50 + 2 + 50) * (3192 + 28 + 2 + 32) * 120 / 1000,
		.hdisplay = 1368,
		.hsync_start = 1368 + 50,
		.hsync_end = 1368 + 50 + 2,
		.htotal = 1368 + 50 + 2 + 50,
		.vdisplay = 3192,
		.vsync_start = 3192 + 28,
		.vsync_end = 3192 + 28 + 2,
		.vtotal = 3192 + 28 + 2 + 32,
		.width_mm = 68,
		.height_mm = 158,
		.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
	}, {
		.clock = (1368 + 50 + 2 + 48) * (3192 + 28 + 2 + 32) * 60 / 1000,
		.hdisplay = 1368,
		.hsync_start = 1368 + 50,
		.hsync_end = 1368 + 50 + 2,
		.htotal = 1368 + 50 + 2 + 48,
		.vdisplay = 3192,
		.vsync_start = 3192 + 28,
		.vsync_end = 3192 + 28 + 2,
		.vtotal = 3192 + 28 + 2 + 32,
		.width_mm = 68,
		.height_mm = 158,
		.type = DRM_MODE_TYPE_DRIVER,
	}, {
		.clock = (1368 + 50 + 2 + 48) * (3192 + 30 + 2 + 32) * 30 / 1000,
		.hdisplay = 1368,
		.hsync_start = 1368 + 50,
		.hsync_end = 1368 + 50 + 2,
		.htotal = 1368 + 50 + 2 + 48,
		.vdisplay = 3192,
		.vsync_start = 3192 + 30,
		.vsync_end = 3192 + 30 + 2,
		.vtotal = 3192 + 30 + 2 + 32,
		.width_mm = 68,
		.height_mm = 158,
		.type = DRM_MODE_TYPE_DRIVER,
	},
};

static int nt37290_boe_get_modes(struct drm_panel *panel,
				 struct drm_connector *connector)
{
	struct nt37290_boe *ctx = to_nt37290_boe(panel);
	unsigned int i;
	int count = 0;

	ctx->connector = connector;
	connector->display_info.width_mm = 68;
	connector->display_info.height_mm = 158;

	for (i = 0; i < ARRAY_SIZE(nt37290_boe_modes); i++) {
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, &nt37290_boe_modes[i]);
		if (!mode)
			continue;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
		count++;
	}

	return count;
}

static const struct drm_panel_funcs nt37290_boe_panel_funcs = {
	.prepare = nt37290_boe_prepare,
	.enable = nt37290_boe_enable,
	.unprepare = nt37290_boe_unprepare,
	.get_modes = nt37290_boe_get_modes,
};

static int nt37290_boe_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct nt37290_boe *ctx = mipi_dsi_get_drvdata(dsi);
	unsigned long mode_flags = dsi->mode_flags;
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	if (ctx->firmware_handoff_active)
		return 0;

	/* Downstream only marks the brightness-off command as LP mode. */
	if (brightness)
		dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	else
		dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	dsi->mode_flags = mode_flags;

	return ret;
}
static const struct backlight_ops nt37290_boe_bl_ops = {
	.update_status = nt37290_boe_bl_update_status,
};

static struct backlight_device *
nt37290_boe_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 4095,
		.max_brightness = 4095,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &nt37290_boe_bl_ops, &props);
}

static int nt37290_boe_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct nt37290_boe *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct nt37290_boe, panel,
				   &nt37290_boe_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = devm_regulator_bulk_get_const(&dsi->dev,
					    ARRAY_SIZE(nt37290_boe_supplies),
					    nt37290_boe_supplies,
					    &ctx->supplies);
	if (ret < 0)
		return ret;
	/* Do not assert reset while UEFI is still scanning out the splash. */
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	INIT_DELAYED_WORK(&ctx->recovery_work, nt37290_boe_recovery_work);
	/* Linux performs a complete cold panel initialization on first commit. */
	ctx->firmware_handoff_pending = false;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB101010;
	dsi->mode_flags = MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS |
			  MIPI_DSI_MODE_DSC_ALL_SLICES_IN_PKT;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = nt37290_boe_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	/* This panel only supports DSC; unconditionally enable it */
	dsi->dsc = &ctx->dsc;

	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 2;

	ctx->dsc.slice_height = 24;
	ctx->dsc.slice_width = 684;
	/*
	 * TODO: hdisplay should be read from the selected mode once
	 * it is passed back to drm_panel (in prepare?)
	 */
	WARN_ON(1368 % ctx->dsc.slice_width);
	ctx->dsc.slice_count = 1368 / ctx->dsc.slice_width;
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

static void nt37290_boe_remove(struct mipi_dsi_device *dsi)
{
	struct nt37290_boe *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	cancel_delayed_work_sync(&ctx->recovery_work);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id nt37290_boe_of_match[] = {
	{ .compatible = "boe,nt37290" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nt37290_boe_of_match);

static struct mipi_dsi_driver nt37290_boe_driver = {
	.probe = nt37290_boe_probe,
	.remove = nt37290_boe_remove,
	.driver = {
		.name = "panel-nt37290-boe",
		.of_match_table = nt37290_boe_of_match,
	},
};
module_mipi_dsi_driver(nt37290_boe_driver);

MODULE_AUTHOR("Chalkin");
MODULE_DESCRIPTION("DRM driver for NT37290_BOE");
MODULE_LICENSE("GPL");
