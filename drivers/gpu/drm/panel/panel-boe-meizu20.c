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
#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/seq_buf.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
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

	/* ---- r22 diagnostics (experiment round, do not ship) ---- */
	struct dentry *debugfs;
	struct mutex diag_lock;	/* serializes cycle_work vs snapshots */
	struct work_struct cycle_work;
	bool diag_cycle;	/* debugfs cycle in flight: suppress auto-recovery */
	/* wake knobs: bit i of wake_mask = send init-table entry i;
	 * bit30 = PPS, bit31 = compression-mode.  ALL_ONES = stock behaviour. */
	u32 wake_mask;
	bool wake_e5;		/* run the whole enable body twice (E5) */
	bool wake_lp;		/* whole enable body in LP (E2) */
	bool wake_pps_late;	/* PPS after the full ON table (stock shape) */
	bool wake_bloff;	/* screen-off = 0x51=0 only, never sleep */
	bool wake_d28;		/* screen-off = 0x28+0x51=0, no sleep-in */
	bool bl_off_live;	/* last off was bloff/d28: panel never slept */
	int cur_profile;	/* last replayed refresh-rate profile */
	int profile_override;	/* <0: follow mode; >=0: force profile (H2) */
	/* session snapshots: scheduled good-state capture + on-demand */
	struct delayed_work snap_work;
	bool snap_scheduled;
	bool snap_done;
	int snap_retries;
	char *snap_probe;	/* auto-captured good state ~20s after boot */
	size_t snap_probe_len;
	char *snap_now;		/* capture-on-read */
	size_t snap_now_len;
	/* dcs io scratch (ported old-chain probe) */
	u8 rd_buf[192];
	int rd_len;
	u8 rd_off;
	u8 pps_buf[128];
	bool pps_valid;
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

/*
 * Per-refresh-rate profile data, verbatim from the stock runtime FDT
 * (vendor-dtb-analysis/runtime-boe-cmd-node.dts).  The four on-command
 * streams are byte-identical except the 0x91 vendor block (entry 13) and
 * the 0x2F profile selector (entry 21); stock switches rate by sending
 * `0x2F <sel>` alone.  The DDIC reloads Gamma/Demura per profile at
 * sleep-out (EP4089666A1) - this table is both the refresh-rate-switching
 * feature and the H2 (profile-mismatch) experiment instrument.
 * NOTE: the DDIC's live 0x91 readback in our session is the 60Hz block
 * with byte1 bit7 set, i.e. the seated profile content does not match the
 * 120Hz profile we command.
 */
#define BOE_MEIZU20_NPROFILES	4
#define BOE_MEIZU20_IDX_60	0
#define BOE_MEIZU20_IDX_120	1
#define BOE_MEIZU20_IDX_90	2
#define BOE_MEIZU20_IDX_144	3

static const u8 boe_meizu20_prof_0x91[BOE_MEIZU20_NPROFILES][18] = {
	[BOE_MEIZU20_IDX_60] = {
		0xab, 0x28, 0x00, 0x14, 0xc2, 0x00, 0x02, 0x0e,
		0x01, 0xe8, 0x00, 0x07, 0x05, 0x0e, 0x05, 0x16,
		0x10, 0xf0,
	},
	[BOE_MEIZU20_IDX_120] = {
		0xab, 0x28, 0x00, 0x14, 0xc2, 0x00, 0x03, 0x1c,
		0x02, 0x8c, 0x00, 0x0f, 0x05, 0x0e, 0x02, 0x8b,
		0x10, 0xf0,
	},
	[BOE_MEIZU20_IDX_90] = {
		0xab, 0xa8, 0x00, 0x14, 0xd2, 0x00, 0x02, 0x25,
		0x01, 0xd5, 0x00, 0x07, 0x05, 0x7a, 0x05, 0x16,
		0x10, 0xf0,
	},
	[BOE_MEIZU20_IDX_144] = {
		0xab, 0xa8, 0x00, 0x14, 0xd2, 0x00, 0x02, 0x25,
		0x01, 0xd5, 0x00, 0x07, 0x05, 0x7a, 0x05, 0x16,
		0x10, 0xf0,
	},
};

static const u8 boe_meizu20_prof_0x2f[BOE_MEIZU20_NPROFILES] = {
	0x01, 0x02, 0x03, 0x04,
};

/* profile index for a refresh rate */
static int boe_meizu20_profile_for_vrefresh(int vrefresh)
{
	switch (vrefresh) {
	case 60:
		return BOE_MEIZU20_IDX_60;
	case 90:
		return BOE_MEIZU20_IDX_90;
	case 144:
		return BOE_MEIZU20_IDX_144;
	default:
		return BOE_MEIZU20_IDX_120;
	}
}

/* Send vendor table entries [first, last), skipping entries whose bit in
 * @mask is clear.  mask == ALL_ONES reproduces the stock sequence, with
 * entries 13 (0x91) / 21 (0x2F) taken from @profile. */
static int boe_meizu20_send_seq_masked(struct boe_meizu20 *ctx, u32 mask,
				       unsigned int first, unsigned int last,
				       int profile)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	unsigned int i;

	for (i = first; i < last && i < ARRAY_SIZE(boe_meizu20_init_cmds); i++) {
		const struct boe_meizu20_cmd *c = &boe_meizu20_init_cmds[i];
		int ret;

		if (!(mask & BIT(i)))
			continue;

		if (i == 13 && c->data[0] == 0x91) {
			ret = mipi_dsi_dcs_write(dsi, 0x91,
						 boe_meizu20_prof_0x91[profile],
						 sizeof(boe_meizu20_prof_0x91[0]));
		} else if (i == 21 && c->data[0] == 0x2f) {
			ret = mipi_dsi_dcs_write(dsi, 0x2f,
						 &boe_meizu20_prof_0x2f[profile], 1);
		} else {
			ret = mipi_dsi_dcs_write(dsi, c->data[0],
						 c->len > 1 ? &c->data[1] : NULL,
						 c->len - 1);
		}
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

/* ==================================================================
 * r22 diagnostics: DDIC session snapshots + wake-path knobs.
 * Verdicts so far (r22a-d device rounds): good vs bad states are
 * byte-identical across the FULL 373-register sweep; every wake flavour
 * (full replay, minimal 0x11/0x29, double pass, all-LP, with/without
 * reset/rails) shows the fixed mosaic while cold boot is clean - the
 * poison event is the DDIC's own sleep transition (sleep-out self-reinit
 * of its demura/OTP-class compensation, beyond any DCS reach).  What
 * remains: never let it sleep - knob bloff (0x51=0 only) / d28 (0x28
 * without 0x10); AMOLED black frames are visually off at near-zero power.
 * All knobs live under /sys/kernel/debug/<dsi-dev>/.
 * ================================================================== */

#define BOE_SNAP_MAX		(64 * 1024)
#define BOE_MEIZU20_MASK_ALL	0xffffffffu
#define BOE_MEIZU20_BIT_PPS	0x40000000u
#define BOE_MEIZU20_BIT_COMP	0x80000000u
/* E6 minimal wake: sleep-out (0x11, entry 22) + display-on (0x29, entry 23). */
#define BOE_MEIZU20_MASK_E6	(BIT(22) | BIT(23))

/*
 * Transitional default (r22 round verdict): the DDIC's own sleep transition
 * mis-reloads its demura/OTP-class compensation in our session - every wake
 * flavour shows the fixed mosaic, cold boot is clean, and the full
 * 373-register good/bad sweep is byte-identical.  So screen-off defaults to
 * never sleeping (0x51=0 black frames).  Disable with
 * boe_meizu20.bloff_default=0, or "echo default > .../knob" at runtime.
 */
static bool bloff_default = true;
module_param(bloff_default, bool, 0444);
MODULE_PARM_DESC(bloff_default, "screen-off without sleep (0x51=0 only) by default");

/* One LP DCS read of @addr, appended to the snapshot buffer.  @page selects
 * the vendor register page first (0xf0 55 aa 52 08 <page>), 0xff = none.
 * Returns 0 on success (including empty reads), negative on the first DSI
 * error - the caller must stop: one failed transaction poisons the command
 * DMA and every further one grinds 200 ms timeouts. */
static int boe_meizu20_snap_one(struct seq_buf *sb,
				struct mipi_dsi_device *dsi,
				unsigned int page, u8 addr, int want)
{
	unsigned long mode_flags = dsi->mode_flags;
	u8 buf[32];
	int ret, i;

	if (page != 0xff) {
		u8 sel[5] = { 0x55, 0xaa, 0x52, 0x08, (u8)page };

		dsi->mode_flags |= MIPI_DSI_MODE_LPM;
		ret = mipi_dsi_dcs_write(dsi, 0xf0, sel, sizeof(sel));
		dsi->mode_flags = mode_flags;
		if (ret < 0) {
			seq_buf_printf(sb, "p%02x select ERR %d\n", page, ret);
			return ret;
		}
		usleep_range(1500, 2500);
	}

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_read(dsi, addr, buf, min_t(size_t, sizeof(buf), want));
	dsi->mode_flags = mode_flags;
	if (ret < 0) {
		seq_buf_printf(sb, "p%02x %02x: ERR %d\n", page, addr, ret);
		return ret;
	}
	if (ret == 0) {
		seq_buf_printf(sb, "p%02x %02x: (0 bytes)\n", page, addr);
		return 0;
	}

	seq_buf_printf(sb, "p%02x %02x:", page, addr);
	for (i = 0; i < ret; i++)
		seq_buf_printf(sb, " %02x", buf[i]);
	seq_buf_putc(sb, '\n');
	return 0;
}

/* Conservative per-page set: only addresses proven read-safe on this panel
 * (std DCS status + the 0x91 vendor block + bank marker 0x00).  The r22a
 * round showed a read of page-0 reg 0x01 can wedge the command path. */
static const u8 boe_meizu20_snap_std[] = { 0x09, 0x51, 0x53, 0x91, 0x9c };
static const u8 boe_meizu20_snap_safe[] = { 0x00, 0x91 };
/* Full sweep (opt-in via the session_full debugfs file only).  Reg 0x01 is
 * blacklisted: reading it deterministically wedges the command path (-110
 * on both r22a and r22b device rounds, any state). */
static const u8 boe_meizu20_snap_full[] = {
	0x00, 0x03, 0x06, 0x08, 0x2f, 0x6f, 0x82,
	0x8b, 0x8d, 0x90, 0x91, 0x92, 0xa0, 0xa2, 0xa5,
	0xa8, 0xad, 0xcd, 0xd0, 0xd6, 0xe1, 0xe2, 0xf2,
};

/* Snapshot the DDIC session: standard DCS status (page-0 context), then the
 * vendor page register banks 0x00..0x0f, then restore the vendor end-of-
 * sequence page (08).  Read-only apart from page selects.  Aborts at the
 * first DSI error.  Returns the number of successful reads. */
static int boe_meizu20_capture(struct boe_meizu20 *ctx, char **outp,
			       size_t *lenp, const char *why, bool full)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	unsigned long mode_flags = dsi->mode_flags;
	static const u8 page_sel_end[5] = { 0x55, 0xaa, 0x52, 0x08, 0x08 };
	const u8 *addrs = full ? boe_meizu20_snap_full : boe_meizu20_snap_safe;
	unsigned int n_addrs = full ? ARRAY_SIZE(boe_meizu20_snap_full)
				    : ARRAY_SIZE(boe_meizu20_snap_safe);
	struct seq_buf sb;
	char *buf;
	int ok = 0, ret = 0;
	unsigned int page, i;

	if (!*outp) {
		buf = kzalloc(BOE_SNAP_MAX, GFP_KERNEL);
		if (!buf)
			return 0;
		*outp = buf;
	} else {
		buf = *outp;
	}

	seq_buf_init(&sb, buf, BOE_SNAP_MAX - 1);
	seq_buf_printf(&sb,
		       "# meizu20 ddic session why=%s full=%d mask=0x%08x e5=%d lp=%d enabled=%d rails=%d\n",
		       why, full, ctx->wake_mask, ctx->wake_e5, ctx->wake_lp,
		       ctx->panel_enabled, ctx->keep_rails);

	for (i = 0; i < ARRAY_SIZE(boe_meizu20_snap_std); i++) {
		ret = boe_meizu20_snap_one(&sb, dsi, 0xff, boe_meizu20_snap_std[i],
					   boe_meizu20_snap_std[i] == 0x91 ? 24 : 8);
		if (ret == 0)
			ok++;
		else
			goto aborted;
		usleep_range(1500, 2500);
	}

	for (page = 0; page <= 0x0f; page++) {
		for (i = 0; i < n_addrs; i++) {
			ret = boe_meizu20_snap_one(&sb, dsi, page, addrs[i],
						   addrs[i] == 0x91 ? 24 : 12);
			if (ret == 0)
				ok++;
			else
				goto aborted;
			usleep_range(1500, 2500);
		}
	}

aborted:
	if (ret < 0)
		seq_buf_printf(&sb,
			       "# ABORTED at first DSI error (ret=%d) - link poisoned, not grinding further\n",
			       ret);

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	mipi_dsi_dcs_write(dsi, 0xf0, page_sel_end, sizeof(page_sel_end));
	dsi->mode_flags = mode_flags;

	seq_buf_printf(&sb, "# end ok=%d\n", ok);
	*lenp = seq_buf_used(&sb);
	return ok;
}

/* Good-state reference capture, ~20 s after the first successful enable:
 * plasma/fbcon is streaming, the panel is awake and calm - the M1-proven
 * safe regime for DCS reads. */
static void boe_meizu20_snap_work(struct work_struct *work)
{
	struct boe_meizu20 *ctx = container_of(work, struct boe_meizu20,
					       snap_work.work);
	int ok;

	/* Not up yet (early boot or a cycle in flight): retry a few times. */
	if ((!ctx->panel_enabled || READ_ONCE(ctx->diag_cycle)) &&
	    ctx->snap_retries < 3) {
		ctx->snap_retries++;
		schedule_delayed_work(&ctx->snap_work, msecs_to_jiffies(10000));
		return;
	}
	if (!ctx->panel_enabled)
		return;

	mutex_lock(&ctx->diag_lock);
	ok = boe_meizu20_capture(ctx, &ctx->snap_probe, &ctx->snap_probe_len,
				 "boot-good", false);
	mutex_unlock(&ctx->diag_lock);

	ctx->snap_done = true;
	dev_info(&ctx->dsi->dev, "r22: good-state snapshot captured (ok=%d)\n", ok);
}

static void boe_meizu20_cycle_work(struct work_struct *work)
{
	struct boe_meizu20 *ctx = container_of(work, struct boe_meizu20,
					       cycle_work);
	struct drm_connector *connector = ctx->connector;
	struct drm_modeset_acquire_ctx acquire_ctx;
	struct drm_atomic_commit *state;
	struct drm_device *drm;
	int ret = 0;

	if (!connector) {
		dev_warn(&ctx->dsi->dev,
			 "r22: cycle before first modeset ignored\n");
		WRITE_ONCE(ctx->diag_cycle, false);
		return;
	}

	mutex_lock(&ctx->diag_lock);
	drm = connector->dev;
	DRM_MODESET_LOCK_ALL_BEGIN(drm, acquire_ctx, 0, ret);

	state = drm_atomic_helper_duplicate_state(drm, &acquire_ctx);
	if (IS_ERR(state)) {
		ret = PTR_ERR(state);
	} else {
		/* disable_all runs our unprepare (M3: sleep only, rails kept);
		 * the commit runs prepare (skipped) + enable() per the knobs. */
		ret = drm_atomic_helper_disable_all(drm, &acquire_ctx);
		if (!ret) {
			msleep(30);
			ret = drm_atomic_helper_commit_duplicated_state(state,
								 &acquire_ctx);
		}
		drm_atomic_commit_put(state);
	}

	DRM_MODESET_LOCK_ALL_END(drm, acquire_ctx, ret);
	mutex_unlock(&ctx->diag_lock);

	if (ret)
		dev_err(&ctx->dsi->dev, "r22: cycle failed: %d\n", ret);
	else
		dev_info(&ctx->dsi->dev,
			 "r22: sleep/wake cycle done (mask=0x%08x e5=%d lp=%d)\n",
			 ctx->wake_mask, ctx->wake_e5, ctx->wake_lp);

	/* enable() replays 0x51=0; restore brightness so the result is
	 * visible to the eye. */
	if (ctx->panel.backlight)
		backlight_update_status(ctx->panel.backlight);

	WRITE_ONCE(ctx->diag_cycle, false);
}

/* ------------------------------------------------------------------
 * dcs io (ported from the old-chain r16 probe; forced LP like stock reads)
 * ------------------------------------------------------------------ */
static int boe_meizu20_dcs_read_reg(struct boe_meizu20 *ctx, u8 off)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	unsigned long mode_flags = dsi->mode_flags;
	int ret, i;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_read(dsi, off, ctx->rd_buf, sizeof(ctx->rd_buf));
	dsi->mode_flags = mode_flags;
	if (ret < 0) {
		dev_err(&dsi->dev, "r22 dcs_read 0x%02x: failed %d\n", off, ret);
		return ret;
	}

	ctx->rd_off = off;
	ctx->rd_len = ret;
	dev_info(&dsi->dev, "r22 dcs_read 0x%02x: len=%d:", off, ret);
	for (i = 0; i < ret; i++)
		pr_cont("%s%02x", i ? " " : " ", ctx->rd_buf[i]);
	pr_cont("\n");
	return ret;
}

static ssize_t boe_meizu20_dcs_read_write(struct file *file,
					  const char __user *ubuf,
					  size_t count, loff_t *ppos)
{
	struct boe_meizu20 *ctx = file_inode(file)->i_private;
	char kbuf[8];
	u8 off;
	int ret;

	if (!count || count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, ubuf, count))
		return -EFAULT;
	kbuf[count] = '\0';
	ret = kstrtou8(strstrip(kbuf), 0, &off);
	if (ret)
		return ret;

	ret = boe_meizu20_dcs_read_reg(ctx, off);
	return ret < 0 ? ret : count;
}

static int boe_meizu20_dcs_read_show(struct seq_file *m, void *v)
{
	struct boe_meizu20 *ctx = m->private;
	int i;

	if (ctx->rd_len <= 0) {
		seq_puts(m, "(no read yet; echo 0x91 > dcs_read)\n");
		return 0;
	}
	seq_printf(m, "dcs 0x%02x len=%d:", ctx->rd_off, ctx->rd_len);
	for (i = 0; i < ctx->rd_len; i++)
		seq_printf(m, " %02x", ctx->rd_buf[i]);
	seq_putc(m, '\n');
	return 0;
}

static int boe_meizu20_dcs_read_open(struct inode *inode, struct file *file)
{
	return single_open(file, boe_meizu20_dcs_read_show, inode->i_private);
}

static const struct file_operations boe_meizu20_dcs_read_fops = {
	.owner = THIS_MODULE,
	.open = boe_meizu20_dcs_read_open,
	.read = seq_read,
	.write = boe_meizu20_dcs_read_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t boe_meizu20_dcs_write_write(struct file *file,
					   const char __user *ubuf,
					   size_t count, loff_t *ppos)
{
	struct boe_meizu20 *ctx = file_inode(file)->i_private;
	struct mipi_dsi_device *dsi = ctx->dsi;
	unsigned long mode_flags = dsi->mode_flags;
	u8 payload[128];
	char *kbuf, *tok, *p;
	u8 reg = 0;
	int plen = 0, ret;

	if (!count || count > 600)
		return -EINVAL;
	kbuf = memdup_user_nul(ubuf, count);
	if (IS_ERR(kbuf))
		return PTR_ERR(kbuf);

	p = strstrip(kbuf);
	while ((tok = strsep(&p, " \t,")) != NULL) {
		if (!*tok)
			continue;
		if (plen == 0) {
			if (kstrtou8(tok, 16, &reg))
				goto out_free;
		} else {
			if (plen > (int)sizeof(payload))
				goto out_free;
			if (kstrtou8(tok, 16, &payload[plen - 1]))
				goto out_free;
		}
		plen++;
	}
	if (plen == 0)
		goto out_free;
	plen--; /* first token = register */

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_write(dsi, reg, plen ? payload : NULL, plen);
	dsi->mode_flags = mode_flags;
	if (ret < 0)
		dev_err(&dsi->dev, "r22 dcs_write 0x%02x len=%d: failed %d\n",
			reg, plen, ret);
	else
		dev_info(&dsi->dev, "r22 dcs_write 0x%02x len=%d: ok\n",
			 reg, plen);
	kfree(kbuf);
	return ret < 0 ? ret : count;

out_free:
	kfree(kbuf);
	return -EINVAL;
}

static const struct file_operations boe_meizu20_dcs_write_fops = {
	.owner = THIS_MODULE,
	.write = boe_meizu20_dcs_write_write,
};

static int boe_meizu20_pps_show(struct seq_file *m, void *v)
{
	struct boe_meizu20 *ctx = m->private;
	int i;

	if (!ctx->pps_valid) {
		seq_puts(m, "(no PPS packed yet)\n");
		return 0;
	}
	for (i = 0; i < (int)sizeof(ctx->pps_buf); i++)
		seq_printf(m, "%02x%s", ctx->pps_buf[i],
			   (i % 16 == 15) ? "\n" : " ");
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(boe_meizu20_pps);

static ssize_t boe_meizu20_comp_sel_write(struct file *file,
					  const char __user *ubuf,
					  size_t count, loff_t *ppos)
{
	struct boe_meizu20 *ctx = file_inode(file)->i_private;
	struct mipi_dsi_device *dsi = ctx->dsi;
	unsigned long mode_flags = dsi->mode_flags;
	char kbuf[8];
	u8 sel;
	int ret;

	if (!count || count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, ubuf, count))
		return -EFAULT;
	kbuf[count] = '\0';
	if (kstrtou8(strstrip(kbuf), 0, &sel) || sel > 3)
		return -EINVAL;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_compression_mode_ext(dsi, true, MIPI_DSI_COMPRESSION_DSC,
					    sel);
	dsi->mode_flags = mode_flags;
	if (ret < 0)
		dev_err(&dsi->dev, "r22 comp_sel %u: failed %d\n", sel, ret);
	else
		dev_info(&dsi->dev, "r22 comp_sel %u: ok (LP, profile %u)\n",
			 sel, sel);
	return ret < 0 ? ret : count;
}

static const struct file_operations boe_meizu20_comp_sel_fops = {
	.owner = THIS_MODULE,
	.write = boe_meizu20_comp_sel_write,
};

static int boe_meizu20_session_probe_show(struct seq_file *m, void *v)
{
	struct boe_meizu20 *ctx = m->private;

	if (!ctx->snap_probe_len) {
		seq_puts(m, "(no probe-time snapshot yet)\n");
		return 0;
	}
	seq_write(m, ctx->snap_probe, ctx->snap_probe_len);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(boe_meizu20_session_probe);

static int boe_meizu20_session_now_show(struct seq_file *m, void *v)
{
	struct boe_meizu20 *ctx = m->private;
	int ok;

	if (!ctx->panel_enabled) {
		seq_puts(m, "(panel asleep or never enabled; refusing to read)\n");
		return 0;
	}

	if (mutex_lock_interruptible(&ctx->diag_lock))
		return -ERESTARTSYS;
	ok = boe_meizu20_capture(ctx, &ctx->snap_now, &ctx->snap_now_len,
				 "now", false);
	mutex_unlock(&ctx->diag_lock);

	dev_info(&ctx->dsi->dev, "r22: on-demand snapshot (ok=%d)\n", ok);
	if (ctx->snap_now_len)
		seq_write(m, ctx->snap_now, ctx->snap_now_len);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(boe_meizu20_session_now);

/* Full 24-address page sweep, opt-in only: the r22a round showed reads of
 * unknown registers can wedge the DDIC command path.  Expect to need a
 * reboot if this one aborts. */
static int boe_meizu20_session_full_show(struct seq_file *m, void *v)
{
	struct boe_meizu20 *ctx = m->private;
	int ok;

	if (!ctx->panel_enabled) {
		seq_puts(m, "(panel asleep or never enabled; refusing to read)\n");
		return 0;
	}

	if (mutex_lock_interruptible(&ctx->diag_lock))
		return -ERESTARTSYS;
	ok = boe_meizu20_capture(ctx, &ctx->snap_now, &ctx->snap_now_len,
				 "full", true);
	mutex_unlock(&ctx->diag_lock);

	dev_info(&ctx->dsi->dev, "r22: full sweep snapshot (ok=%d)\n", ok);
	if (ctx->snap_now_len)
		seq_write(m, ctx->snap_now, ctx->snap_now_len);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(boe_meizu20_session_full);

static int boe_meizu20_knob_show(struct seq_file *m, void *v)
{
	struct boe_meizu20 *ctx = m->private;

	seq_printf(m, "mask=0x%08x e5=%d lp=%d pps_late=%d bloff=%d d28=%d profile=%d override=%d\n",
		   ctx->wake_mask, ctx->wake_e5, ctx->wake_lp,
		   ctx->wake_pps_late, ctx->wake_bloff, ctx->wake_d28,
		   ctx->cur_profile, ctx->profile_override);
	return 0;
}

static void boe_meizu20_knob_clear(struct boe_meizu20 *ctx)
{
	ctx->wake_bloff = false;
	ctx->wake_d28 = false;
	ctx->wake_e5 = false;
	ctx->wake_lp = false;
	ctx->wake_pps_late = false;
	ctx->profile_override = -1;
}

static ssize_t boe_meizu20_knob_write(struct file *file,
				      const char __user *ubuf,
				      size_t count, loff_t *ppos)
{
	struct boe_meizu20 *ctx = file_inode(file)->i_private;
	char kbuf[16];

	if (!count || count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, ubuf, count))
		return -EFAULT;
	kbuf[count] = '\0';
	(void)strstrip(kbuf);

	boe_meizu20_knob_clear(ctx);
	if (!strcmp(kbuf, "default")) {
		ctx->wake_mask = BOE_MEIZU20_MASK_ALL;
	} else if (!strcmp(kbuf, "e6")) {
		ctx->wake_mask = BOE_MEIZU20_MASK_E6;
	} else if (!strcmp(kbuf, "e5")) {
		ctx->wake_mask = BOE_MEIZU20_MASK_ALL;
		ctx->wake_e5 = true;
	} else if (!strcmp(kbuf, "e2")) {
		ctx->wake_mask = BOE_MEIZU20_MASK_ALL;
		ctx->wake_lp = true;
	} else if (!strcmp(kbuf, "e6+e5")) {
		ctx->wake_mask = BOE_MEIZU20_MASK_E6;
		ctx->wake_e5 = true;
	} else if (!strcmp(kbuf, "e6+lp")) {
		ctx->wake_mask = BOE_MEIZU20_MASK_E6;
		ctx->wake_lp = true;
	} else if (!strcmp(kbuf, "profile60") || !strcmp(kbuf, "profile90") ||
		   !strcmp(kbuf, "profile120") || !strcmp(kbuf, "profile144")) {
		/* H2: real sleep + full replay at 120Hz timing with a
		 * FOREIGN profile's 0x91/0x2F.  If the mask pattern changes
		 * with the profile, the sleep-out reload consumes the
		 * commanded profile - if not, H2 is dead too. */
		ctx->wake_mask = BOE_MEIZU20_MASK_ALL;
		if (!strcmp(kbuf, "profile60"))
			ctx->profile_override = BOE_MEIZU20_IDX_60;
		else if (!strcmp(kbuf, "profile120"))
			ctx->profile_override = BOE_MEIZU20_IDX_120;
		else if (!strcmp(kbuf, "profile90"))
			ctx->profile_override = BOE_MEIZU20_IDX_90;
		else
			ctx->profile_override = BOE_MEIZU20_IDX_144;
	} else if (!strcmp(kbuf, "stock")) {
		/* S1 P1+P2+P3: the exact downstream wake shape - no 0x07
		 * compression packet, PPS after the full ON table, all LP. */
		ctx->wake_mask = BOE_MEIZU20_MASK_ALL & ~BOE_MEIZU20_BIT_COMP;
		ctx->wake_lp = true;
		ctx->wake_pps_late = true;
	} else if (!strcmp(kbuf, "bloff")) {
		/* screen-off = 0x51=0 only; the panel never sleeps */
		ctx->wake_bloff = true;
	} else if (!strcmp(kbuf, "d28")) {
		/* screen-off = 0x28 + 0x51=0; no sleep-in (0x10) */
		ctx->wake_d28 = true;
	} else {
		return -EINVAL;
	}

	dev_info(&ctx->dsi->dev,
		 "r22: knob '%s' (mask=0x%08x e5=%d lp=%d bloff=%d d28=%d)\n",
		 kbuf, ctx->wake_mask, ctx->wake_e5, ctx->wake_lp,
		 ctx->wake_bloff, ctx->wake_d28);
	return count;
}

static int boe_meizu20_knob_open(struct inode *inode, struct file *file)
{
	return single_open(file, boe_meizu20_knob_show, inode->i_private);
}

static const struct file_operations boe_meizu20_knob_fops = {
	.owner = THIS_MODULE,
	.open = boe_meizu20_knob_open,
	.read = seq_read,
	.write = boe_meizu20_knob_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t boe_meizu20_wake_mask_write(struct file *file,
					   const char __user *ubuf,
					   size_t count, loff_t *ppos)
{
	struct boe_meizu20 *ctx = file_inode(file)->i_private;
	char kbuf[12];
	int ret;

	if (!count || count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, ubuf, count))
		return -EFAULT;
	kbuf[count] = '\0';
	ret = kstrtou32(strstrip(kbuf), 0, &ctx->wake_mask);
	if (ret)
		return ret;
	dev_info(&ctx->dsi->dev, "r22: wake_mask=0x%08x\n", ctx->wake_mask);
	return count;
}

static const struct file_operations boe_meizu20_wake_mask_fops = {
	.owner = THIS_MODULE,
	.write = boe_meizu20_wake_mask_write,
};

static ssize_t boe_meizu20_cycle_write(struct file *file,
				       const char __user *ubuf,
				       size_t count, loff_t *ppos)
{
	struct boe_meizu20 *ctx = file_inode(file)->i_private;

	if (READ_ONCE(ctx->diag_cycle))
		return -EBUSY;

	WRITE_ONCE(ctx->diag_cycle, true);
	schedule_work(&ctx->cycle_work);
	return count;
}

static const struct file_operations boe_meizu20_cycle_fops = {
	.owner = THIS_MODULE,
	.write = boe_meizu20_cycle_write,
};

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

/* refresh rate of the mode currently being committed (0 = unknown) */
static int boe_meizu20_current_vrefresh(struct boe_meizu20 *ctx)
{
	struct drm_crtc *crtc;

	if (!ctx->connector || !ctx->connector->state)
		return 0;
	crtc = ctx->connector->state->crtc;
	if (!crtc || !crtc->state)
		return 0;
	if (!crtc->state->mode.clock)
		return 0;
	return drm_mode_vrefresh(&crtc->state->mode);
}

static int boe_meizu20_enable_once(struct boe_meizu20 *ctx)
{
	struct device *dev = &ctx->dsi->dev;
	struct drm_dsc_picture_parameter_set pps;
	unsigned long mode_flags;
	u32 mask = READ_ONCE(ctx->wake_mask);
	bool pps_late = READ_ONCE(ctx->wake_pps_late);
	int profile = READ_ONCE(ctx->profile_override);
	int ret;

	if (profile < 0)
		profile = boe_meizu20_profile_for_vrefresh(
					boe_meizu20_current_vrefresh(ctx));

	mode_flags = ctx->dsi->mode_flags;
	if (READ_ONCE(ctx->wake_lp))
		ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = boe_meizu20_send_seq_masked(ctx, mask, 0, BOE_MEIZU20_SEQ_DSC_SPLIT,
					  profile);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel (pre-DSC part): %d\n", ret);
		goto out_flags;
	}

	if ((mask & BOE_MEIZU20_BIT_PPS) && !pps_late) {
		drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
		memcpy(ctx->pps_buf, &pps, sizeof(pps));
		ctx->pps_valid = true;

		ret = mipi_dsi_picture_parameter_set(ctx->dsi, &pps);
		if (ret < 0) {
			dev_err(dev, "failed to transmit PPS: %d\n", ret);
			goto out_flags;
		}
	}
	if (mask & BOE_MEIZU20_BIT_COMP) {
		ret = mipi_dsi_compression_mode(ctx->dsi, true);
		if (ret < 0) {
			dev_err(dev, "failed to enable compression mode: %d\n", ret);
			goto out_flags;
		}
	}
	if ((mask & (BOE_MEIZU20_BIT_PPS | BOE_MEIZU20_BIT_COMP)) && !pps_late) {
		/* Give the DDIC a moment to latch the PPS before it leaves sleep. */
		mdelay(5);
	}

	ret = boe_meizu20_send_seq_masked(ctx, mask, BOE_MEIZU20_SEQ_DSC_SPLIT,
					  ARRAY_SIZE(boe_meizu20_init_cmds),
					  profile);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel (post-DSC part): %d\n", ret);
		goto out_flags;
	}

	/* P2 (knob stock): downstream transmits the PPS only after the full
	 * ON table (dsi_display.c dsi_display_enable path), LP, every wake. */
	if (mask & BOE_MEIZU20_BIT_PPS) {
		drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
		memcpy(ctx->pps_buf, &pps, sizeof(pps));
		ctx->pps_valid = true;

		ret = mipi_dsi_picture_parameter_set(ctx->dsi, &pps);
		if (ret < 0) {
			dev_err(dev, "failed to transmit late PPS: %d\n", ret);
			goto out_flags;
		}
	}
	WRITE_ONCE(ctx->panel_enabled, true);
	WRITE_ONCE(ctx->cur_profile, profile);

	/* The stock stream ends with 0x29 (display on); give the DDIC its settle. */
	mdelay(20);

out_flags:
	ctx->dsi->mode_flags = mode_flags;
	return ret;
}

/* LP backlight write with the usual first-write retry. */
static int boe_meizu20_bl_write(struct mipi_dsi_device *dsi, u16 brightness)
{
	unsigned long mode_flags = dsi->mode_flags;
	unsigned int i;
	int ret;

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

static int boe_meizu20_enable(struct drm_panel *panel)
{
	struct boe_meizu20 *ctx = to_boe_meizu20(panel);
	struct device *dev = &ctx->dsi->dev;
	unsigned long mode_flags;
	u8 power_mode = 0;
	int ret;

	/* bloff/d28: the panel never slept - no replay, just display-on
	 * (d28 only) and brightness restore.  Only valid when the last off
	 * was itself a bloff/d28 off AND the refresh-rate profile did not
	 * change (a profile switch needs a full replay to send the mode's
	 * 0x91/0x2F).  After a cold boot the full replay runs anyway. */
	if ((READ_ONCE(ctx->wake_bloff) || READ_ONCE(ctx->wake_d28)) &&
	    READ_ONCE(ctx->bl_off_live) &&
	    ctx->cur_profile == boe_meizu20_profile_for_vrefresh(
					boe_meizu20_current_vrefresh(ctx))) {
		ctx->bl_off_live = false;
		if (READ_ONCE(ctx->wake_d28))
			mipi_dsi_dcs_set_display_on(ctx->dsi);
		WRITE_ONCE(ctx->panel_enabled, true);
		if (ctx->panel.backlight)
			backlight_update_status(ctx->panel.backlight);
		return 0;
	}
	ctx->bl_off_live = false;

	ret = boe_meizu20_enable_once(ctx);

	/* E5: the recovery path proves a second pass over the live panel
	 * is safe; test whether the sequence simply needs to run twice. */
	if (!ret && READ_ONCE(ctx->wake_e5)) {
		msleep(60);
		ret = boe_meizu20_enable_once(ctx);
	}
	if (ret)
		return ret;

	/* One good-state reference snapshot, taken well after the enable
	 * storm has settled (never inside the commit path). */
	if (!ctx->snap_scheduled) {
		ctx->snap_scheduled = true;
		schedule_delayed_work(&ctx->snap_work, msecs_to_jiffies(20000));
	}

	/* Cold-enable sanity check.  The first pass after a panel reset is
	 * never answered by the DDIC (-ENODATA); the recovery cycle and its
	 * second full pass are what actually establishes the working session
	 * (old-chain r6/r7 evidence).  It must therefore run on every real
	 * DPMS enable; only the debugfs diag cycle suppresses it. */
	if (READ_ONCE(ctx->diag_cycle))
		return 0;

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

	/*
	 * bloff/d28: screen-off WITHOUT the sleep transition.  The r22b/c/d
	 * device rounds proved the poison event is the DDIC's own sleep
	 * cycle: every wake flavour (full replay, minimal 0x11/0x29, double
	 * pass, all-LP, with/without reset/rails) shows the fixed mosaic,
	 * while cold boot and the boot-time recovery are clean, and the
	 * 373-register good/bad sweep is byte-identical - the damage happens
	 * inside the DDIC's sleep-out self-reinit (demura/OTP class), beyond
	 * any DCS reach.  On AMOLED 0x51=0 is black frames: visually off at
	 * near-zero power, and the compensation state is never reloaded.
	 */
	if (READ_ONCE(ctx->wake_bloff) || READ_ONCE(ctx->wake_d28)) {
		if (READ_ONCE(ctx->wake_d28)) {
			ret = mipi_dsi_dcs_set_display_off(ctx->dsi);
			if (ret < 0)
				dev_err(dev, "display-off failed: %d\n", ret);
			else
				usleep_range(5000, 6000);
		}
		boe_meizu20_bl_write(ctx->dsi, 0);
		WRITE_ONCE(ctx->keep_rails, true);
		WRITE_ONCE(ctx->panel_enabled, false);
		WRITE_ONCE(ctx->bl_off_live, true);
		return 0;
	}

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
		/* qcom,mdss-dsi timing@0: h/v porches 50/16/50, 20/4/12.
		 * Multi-rate (60/90/144) modes reverted: exposing them moved
		 * the session-start timing and correlated 2/2 boot crashes in
		 * the geni_i2c_resume window (r22g).  Rate switching is parked
		 * until the mask is fixed; the profile tables stay for the
		 * science knobs. */
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
	INIT_DELAYED_WORK(&ctx->snap_work, boe_meizu20_snap_work);
	INIT_WORK(&ctx->cycle_work, boe_meizu20_cycle_work);
	mutex_init(&ctx->diag_lock);
	ctx->wake_mask = BOE_MEIZU20_MASK_ALL;
	ctx->cur_profile = BOE_MEIZU20_IDX_120;
	ctx->profile_override = -1;
	if (bloff_default)
		ctx->wake_bloff = true;
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

	ctx->debugfs = debugfs_create_dir(dev_name(&dsi->dev), NULL);
	if (!IS_ERR_OR_NULL(ctx->debugfs)) {
		debugfs_create_file("knob", 0644, ctx->debugfs, ctx,
				    &boe_meizu20_knob_fops);
		debugfs_create_file("wake_mask", 0200, ctx->debugfs, ctx,
				    &boe_meizu20_wake_mask_fops);
		debugfs_create_file("cycle", 0200, ctx->debugfs, ctx,
				    &boe_meizu20_cycle_fops);
		debugfs_create_file("session_probe", 0444, ctx->debugfs, ctx,
				    &boe_meizu20_session_probe_fops);
		debugfs_create_file("session_now", 0444, ctx->debugfs, ctx,
				    &boe_meizu20_session_now_fops);
		debugfs_create_file("session_full", 0444, ctx->debugfs, ctx,
				    &boe_meizu20_session_full_fops);
		debugfs_create_file("dcs_read", 0644, ctx->debugfs, ctx,
				    &boe_meizu20_dcs_read_fops);
		debugfs_create_file("dcs_write", 0200, ctx->debugfs, ctx,
				    &boe_meizu20_dcs_write_fops);
		debugfs_create_file("pps", 0444, ctx->debugfs, ctx,
				    &boe_meizu20_pps_fops);
		debugfs_create_file("comp_sel", 0200, ctx->debugfs, ctx,
				    &boe_meizu20_comp_sel_fops);
	}

	return 0;
}

static void boe_meizu20_remove(struct mipi_dsi_device *dsi)
{
	struct boe_meizu20 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	cancel_delayed_work_sync(&ctx->recovery_work);
	cancel_delayed_work_sync(&ctx->snap_work);
	cancel_work_sync(&ctx->cycle_work);
	debugfs_remove_recursive(ctx->debugfs);
	kfree(ctx->snap_probe);
	kfree(ctx->snap_now);

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
