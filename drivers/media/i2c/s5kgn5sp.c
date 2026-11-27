// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S5KGN5SP03 50MP raw image sensor driver (Meizu 20 rear main/wide).
 *
 * Register tables and power sequencing decoded from the stock camx sensor
 * module binary (com.qti.sensormodule.shinetech_s5kgn5sp.bin) - see
 * nura-meizu-m2381 GN5 main-camera round notes.
 * Ground truth cross-checked against stock running dmesg (2026-11-27 Flyme
 * forensics, camera-rear session: ACQUIRE/START_DEV s5kgn5sp sensor_id
 * 0x8e5 @ slave 0x7a, res index 1 = 4080x3060 2PD 30fps, csiphy3 CPHY,
 * Datarate 5661696000, Settletime 7200000000 -> settle_cnt 36), the stock
 * FDT sensor node (cci1 master-1 AON bus, MCLK2 = tlmm102, RESET = tlmm118,
 * rails pm8008i l5/l2/l7) and the ACK value 0x8e5 @ reg 0x0000.
 *
 * C-PHY notes: 3 data lanes, no clock lane (DT: data-lanes = <0 1 2>, no
 * clock-lanes). link_freq below is the stock per-lane Datarate/2 following
 * the DPHY V4L2 half-rate convention; on C-PHY the physical meaning is
 * per-trio payload rate / 2 (symbol rate x 16/7 packing already included
 * in the Datarate). csiphy settle counter for this link is stock truth 36
 * (module param cphy_settle_cnt in camss-csiphy-3ph-1-0.c).
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define S5KGN5SP_MCLK_FREQ_19MHZ	(19200000)
#define S5KGN5SP_DATA_LANES		3	/* C-PHY trio, no clock lane */

#define S5KGN5SP_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5KGN5SP_CHIP_ID		0x8e5


#define S5KGN5SP_REG_PAGE_SELECT	CCI_REG16(0xfcfc)	/* GN5 page select */

#define S5KGN5SP_REG_MODE_SELECT	CCI_REG16(0x0100)
#define S5KGN5SP_MODE_STREAMING		0x0103	/* stock stream-on value */
#define S5KGN5SP_MODE_STANDBY		0x0000

#define S5KGN5SP_REG_GROUP_HOLD		CCI_REG16(0x0104)
#define S5KGN5SP_GROUP_HOLD_ON		0x0001
#define S5KGN5SP_GROUP_HOLD_OFF		0x0000

#define S5KGN5SP_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5KGN5SP_EXPOSURE_MIN		8
#define S5KGN5SP_EXPOSURE_STEP		1
#define S5KGN5SP_EXPOSURE_MARGIN	8

/* Analog gain: code/1024 = multiplier, 1x = 1024 (Volla s5kgd2sp
 * ana_gain_table; same Samsung ISOCELL code semantics for this family). */
#define S5KGN5SP_REG_ANALOG_GAIN	CCI_REG16(0x0204)
#define S5KGN5SP_ANALOG_GAIN_MIN	1024
#define S5KGN5SP_ANALOG_GAIN_MAX	16384
#define S5KGN5SP_ANALOG_GAIN_STEP	32

#define S5KGN5SP_REG_VTS		CCI_REG16(0x0340)
#define S5KGN5SP_VTS_MAX		0xffff
#define S5KGN5SP_REG_HTS		CCI_REG16(0x0342)

#define to_s5kgn5sp(_sd)		container_of(_sd, struct s5kgn5sp, sd)

/* Stock truth: CAM_START_PHYDEV Datarate = 5661696000 (per-lane wire rate,
 * C-PHY 2.5GSpS table band). V4L2 link_freq = Datarate/2 (DDR half-rate
 * convention carried over from the DPHY drivers; on C-PHY there is no
 * clock lane, this is a per-trio payload-rate/2 label). */
static const s64 s5kgn5sp_link_freq_menu[] = {
	2830848000,
};
#define S5KGN5SP_LF_IDX_FULL	0

/* Measured truth (2026-11-20 colour round, red-box eye verdict): real
 * line order is GBRG (first row G) — the SRGGB label produced R/B-swapped
 * colours; grey-path decode cannot tell, colour did. */
static const u32 s5kgn5sp_mbus_format = MEDIA_BUS_FMT_SGBRG10_1X10;

struct s5kgn5sp_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct s5kgn5sp_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts;
	u32 link_freq_idx;
	const struct s5kgn5sp_reg_list reg_list;
};

/*
 * Mode register sequences decoded verbatim from the vendor bin (74 records
 * each; the 0x602a/0x6f12 pairs are the page-0x2000 indirect addressing).
 * Vendor groups g0 (full-res) and g2 (1080p). g1/g3 are per-vendor variants
 * of the same two modes (extra 0xb32/0xb34 writes) — not exposed for now.
 */
/*
 * Vendor bin group 15 (initSettings, 1213 records) verbatim: id-lock
 * 0x0000<-0x0015, sensor FW patch upload (page 0x2001 block), then mode
 * independent setup. The two vendor delays (0x6018 +1ms, 0x6014 +20ms) are
 * realised as array splits with explicit usleep_range() between segments.
 */
static const struct cci_reg_sequence s5kgn5sp_init_a[] = {
	{ CCI_REG16(0xfcfc), 0x4000 },
	{ CCI_REG16(0x0000), 0x0015 },
	{ CCI_REG16(0x6018), 0x0001 },
};

static const struct cci_reg_sequence s5kgn5sp_init_b[] = {
	{ CCI_REG16(0xfcfc), 0x2002 },
	{ CCI_REG16(0xb7f0), 0x0000 },
	{ CCI_REG16(0xfcfc), 0x4000 },
	{ CCI_REG16(0x7002), 0x0008 },
	{ CCI_REG16(0x7004), 0x12c0 },
	{ CCI_REG16(0x70b8), 0x48c8 },
	{ CCI_REG16(0x6014), 0x0001 },
};

static const struct cci_reg_sequence s5kgn5sp_init_c[] = {
	{ CCI_REG16(0x0a02), 0x03d8 },
	{ CCI_REG16(0xfcfc), 0x2001 },
	{ CCI_REG16(0xdc84), 0xc5fa },
	{ CCI_REG16(0xdc96), 0xe2c0 },
	{ CCI_REG16(0xdca2), 0x2de9 },
	{ CCI_REG16(0xdca4), 0xf047 },
	{ CCI_REG16(0xdca6), 0xfc4d },
	{ CCI_REG16(0xdca8), 0x8246 },
	{ CCI_REG16(0xdcaa), 0x0120 },
	{ CCI_REG16(0xdcac), 0x15f8 },
	{ CCI_REG16(0xdcae), 0xce6f },
	{ CCI_REG16(0xdcb0), 0x9046 },
	{ CCI_REG16(0xdcb2), 0x2870 },
	{ CCI_REG16(0xdcb4), 0xf948 },
	{ CCI_REG16(0xdcb6), 0x8946 },
	{ CCI_REG16(0xdcb8), 0x0022 },
	{ CCI_REG16(0xdcba), 0xd0f8 },
	{ CCI_REG16(0xdcbc), 0x9040 },
	{ CCI_REG16(0xdcbe), 0x200c },
	{ CCI_REG16(0xdcc0), 0xa1b2 },
	{ CCI_REG16(0xdcc2), 0x0746 },
	{ CCI_REG16(0xdcc4), 0xfff7 },
	{ CCI_REG16(0xdcc6), 0x88fe },
	{ CCI_REG16(0xdcc8), 0x4246 },
	{ CCI_REG16(0xdcca), 0x4946 },
	{ CCI_REG16(0xdccc), 0x5046 },
	{ CCI_REG16(0xdcce), 0x00f0 },
	{ CCI_REG16(0xdcd0), 0xa4fa },
	{ CCI_REG16(0xdcd2), 0xa1b2 },
	{ CCI_REG16(0xdcd4), 0x0122 },
	{ CCI_REG16(0xdcd6), 0x3846 },
	{ CCI_REG16(0xdcd8), 0xfff7 },
	{ CCI_REG16(0xdcda), 0x7efe },
	{ CCI_REG16(0xdcdc), 0x2e70 },
	{ CCI_REG16(0xdcde), 0xbde8 },
	{ CCI_REG16(0xdce0), 0xf087 },
	{ CCI_REG16(0xdce2), 0xef49 },
	{ CCI_REG16(0xdce4), 0x10b5 },
	{ CCI_REG16(0xdce6), 0x0b6f },
	{ CCI_REG16(0xdce8), 0xb1f8 },
	{ CCI_REG16(0xdcea), 0xaa14 },
	{ CCI_REG16(0xdcec), 0x0022 },
	{ CCI_REG16(0xdcee), 0xa0fb },
	{ CCI_REG16(0xdcf0), 0x0110 },
	{ CCI_REG16(0xdcf2), 0xfff7 },
	{ CCI_REG16(0xdcf4), 0x57ff },
	{ CCI_REG16(0xdcf6), 0x0846 },
	{ CCI_REG16(0xdcf8), 0x10bd },
	{ CCI_REG16(0xdcfa), 0x2de9 },
	{ CCI_REG16(0xdcfc), 0xfc5f },
	{ CCI_REG16(0xdcfe), 0x0446 },
	{ CCI_REG16(0xdd00), 0x9846 },
	{ CCI_REG16(0xdd02), 0x9246 },
	{ CCI_REG16(0xdd04), 0x8b46 },
	{ CCI_REG16(0xdd06), 0x1846 },
	{ CCI_REG16(0xdd08), 0xddf8 },
	{ CCI_REG16(0xdd0a), 0x3090 },
	{ CCI_REG16(0xdd0c), 0xfff7 },
	{ CCI_REG16(0xdd0e), 0xe9ff },
	{ CCI_REG16(0xdd10), 0x0026 },
	{ CCI_REG16(0xdd12), 0x0246 },
	{ CCI_REG16(0xdd14), 0x5328 },
	{ CCI_REG16(0xdd16), 0x3ed9 },
	{ CCI_REG16(0xdd18), 0xe24f },
	{ CCI_REG16(0xdd1a), 0x3888 },
	{ CCI_REG16(0xdd1c), 0xd8b3 },
	{ CCI_REG16(0xdd1e), 0x0020 },
	{ CCI_REG16(0xdd20), 0xdf49 },
	{ CCI_REG16(0xdd22), 0x0190 },
	{ CCI_REG16(0xdd24), 0xa08b },
	{ CCI_REG16(0xdd26), 0xd1f8 },
	{ CCI_REG16(0xdd28), 0x8016 },
	{ CCI_REG16(0xdd2a), 0x0823 },
	{ CCI_REG16(0xdd2c), 0x4843 },
	{ CCI_REG16(0xdd2e), 0x050b },
	{ CCI_REG16(0xdd30), 0x01a8 },
	{ CCI_REG16(0xdd32), 0x0090 },
	{ CCI_REG16(0xdd34), 0x0e21 },
	{ CCI_REG16(0xdd36), 0xdc48 },
	{ CCI_REG16(0xdd38), 0x00f0 },
	{ CCI_REG16(0xdd3a), 0x74fa },
	{ CCI_REG16(0xdd3c), 0xc0b2 },
	{ CCI_REG16(0xdd3e), 0x411c },
	{ CCI_REG16(0xdd40), 0x0e29 },
	{ CCI_REG16(0xdd42), 0x00d3 },
	{ CCI_REG16(0xdd44), 0x0146 },
	{ CCI_REG16(0xdd46), 0xd84a },
	{ CCI_REG16(0xdd48), 0xc9b2 },
	{ CCI_REG16(0xdd4a), 0x1c32 },
	{ CCI_REG16(0xdd4c), 0x0928 },
	{ CCI_REG16(0xdd4e), 0x03d2 },
	{ CCI_REG16(0xdd50), 0x07eb },
	{ CCI_REG16(0xdd52), 0x4000 },
	{ CCI_REG16(0xdd54), 0x4088 },
	{ CCI_REG16(0xdd56), 0x03e0 },
	{ CCI_REG16(0xdd58), 0x02eb },
	{ CCI_REG16(0xdd5a), 0x4000 },
	{ CCI_REG16(0xdd5c), 0x30f8 },
	{ CCI_REG16(0xdd5e), 0x120c },
	{ CCI_REG16(0xdd60), 0x0929 },
	{ CCI_REG16(0xdd62), 0x03d2 },
	{ CCI_REG16(0xdd64), 0x07eb },
	{ CCI_REG16(0xdd66), 0x4101 },
	{ CCI_REG16(0xdd68), 0x4988 },
	{ CCI_REG16(0xdd6a), 0x03e0 },
	{ CCI_REG16(0xdd6c), 0x02eb },
	{ CCI_REG16(0xdd6e), 0x4101 },
	{ CCI_REG16(0xdd70), 0x31f8 },
	{ CCI_REG16(0xdd72), 0x121c },
	{ CCI_REG16(0xdd74), 0x019a },
	{ CCI_REG16(0xdd76), 0x091a },
	{ CCI_REG16(0xdd78), 0x5143 },
	{ CCI_REG16(0xdd7a), 0x8031 },
	{ CCI_REG16(0xdd7c), 0x00eb },
	{ CCI_REG16(0xdd7e), 0x2120 },
	{ CCI_REG16(0xdd80), 0x80b2 },
	{ CCI_REG16(0xdd82), 0xa842 },
	{ CCI_REG16(0xdd84), 0x07d9 },
	{ CCI_REG16(0xdd86), 0x401b },
	{ CCI_REG16(0xdd88), 0x0601 },
	{ CCI_REG16(0xdd8a), 0x4fea },
	{ CCI_REG16(0xdd8c), 0x0821 },
	{ CCI_REG16(0xdd8e), 0x00f5 },
	{ CCI_REG16(0xdd90), 0x8070 },
	{ CCI_REG16(0xdd92), 0xb1fb },
	{ CCI_REG16(0xdd94), 0xf0f8 },
	{ CCI_REG16(0xdd96), 0xc148 },
	{ CCI_REG16(0xdd98), 0x0022 },
	{ CCI_REG16(0xdd9a), 0xd0f8 },
	{ CCI_REG16(0xdd9c), 0x9450 },
	{ CCI_REG16(0xdd9e), 0x280c },
	{ CCI_REG16(0xdda0), 0xa9b2 },
	{ CCI_REG16(0xdda2), 0x0746 },
	{ CCI_REG16(0xdda4), 0xfff7 },
	{ CCI_REG16(0xdda6), 0x18fe },
	{ CCI_REG16(0xdda8), 0x4346 },
	{ CCI_REG16(0xddaa), 0x5246 },
	{ CCI_REG16(0xddac), 0x5946 },
	{ CCI_REG16(0xddae), 0x2046 },
	{ CCI_REG16(0xddb0), 0xcdf8 },
	{ CCI_REG16(0xddb2), 0x0090 },
	{ CCI_REG16(0xddb4), 0x00f0 },
	{ CCI_REG16(0xddb6), 0x3bfa },
	{ CCI_REG16(0xddb8), 0xa9b2 },
	{ CCI_REG16(0xddba), 0x0122 },
	{ CCI_REG16(0xddbc), 0x3846 },
	{ CCI_REG16(0xddbe), 0xfff7 },
	{ CCI_REG16(0xddc0), 0x0bfe },
	{ CCI_REG16(0xddc2), 0x6069 },
	{ CCI_REG16(0xddc4), 0x3044 },
	{ CCI_REG16(0xddc6), 0x6061 },
	{ CCI_REG16(0xddc8), 0xbde8 },
	{ CCI_REG16(0xddca), 0xfc9f },
	{ CCI_REG16(0xddcc), 0x2de9 },
	{ CCI_REG16(0xddce), 0xf041 },
	{ CCI_REG16(0xddd0), 0x0d46 },
	{ CCI_REG16(0xddd2), 0x0646 },
	{ CCI_REG16(0xddd4), 0xfff7 },
	{ CCI_REG16(0xddd6), 0x87fe },
	{ CCI_REG16(0xddd8), 0xdff8 },
	{ CCI_REG16(0xddda), 0xd082 },
	{ CCI_REG16(0xdddc), 0x0528 },
	{ CCI_REG16(0xddde), 0x1ed0 },
	{ CCI_REG16(0xdde0), 0xae48 },
	{ CCI_REG16(0xdde2), 0x0022 },
	{ CCI_REG16(0xdde4), 0x446d },
	{ CCI_REG16(0xdde6), 0x200c },
	{ CCI_REG16(0xdde8), 0xa1b2 },
	{ CCI_REG16(0xddea), 0x0746 },
	{ CCI_REG16(0xddec), 0xfff7 },
	{ CCI_REG16(0xddee), 0xf4fd },
	{ CCI_REG16(0xddf0), 0x2946 },
	{ CCI_REG16(0xddf2), 0x3046 },
	{ CCI_REG16(0xddf4), 0xfff7 },
	{ CCI_REG16(0xddf6), 0x7cfe },
	{ CCI_REG16(0xddf8), 0xa1b2 },
	{ CCI_REG16(0xddfa), 0x0122 },
	{ CCI_REG16(0xddfc), 0x3846 },
	{ CCI_REG16(0xddfe), 0xfff7 },
	{ CCI_REG16(0xde00), 0xebfd },
	{ CCI_REG16(0xde02), 0xa848 },
	{ CCI_REG16(0xde04), 0x808a },
	{ CCI_REG16(0xde06), 0x0028 },
	{ CCI_REG16(0xde08), 0x07d0 },
	{ CCI_REG16(0xde0a), 0xd8f8 },
	{ CCI_REG16(0xde0c), 0x2c10 },
	{ CCI_REG16(0xde0e), 0x8140 },
	{ CCI_REG16(0xde10), 0xc8f8 },
	{ CCI_REG16(0xde12), 0x2c10 },
	{ CCI_REG16(0xde14), 0x2988 },
	{ CCI_REG16(0xde16), 0x8140 },
	{ CCI_REG16(0xde18), 0x2980 },
	{ CCI_REG16(0xde1a), 0xbde8 },
	{ CCI_REG16(0xde1c), 0xf081 },
	{ CCI_REG16(0xde1e), 0x98f8 },
	{ CCI_REG16(0xde20), 0x0300 },
	{ CCI_REG16(0xde22), 0x0022 },
	{ CCI_REG16(0xde24), 0x10b1 },
	{ CCI_REG16(0xde26), 0x0228 },
	{ CCI_REG16(0xde28), 0x04d1 },
	{ CCI_REG16(0xde2a), 0x01e0 },
	{ CCI_REG16(0xde2c), 0x3068 },
	{ CCI_REG16(0xde2e), 0x00e0 },
	{ CCI_REG16(0xde30), 0x3069 },
	{ CCI_REG16(0xde32), 0x8208 },
	{ CCI_REG16(0xde34), 0x9e48 },
	{ CCI_REG16(0xde36), 0x4079 },
	{ CCI_REG16(0xde38), 0x00b1 },
	{ CCI_REG16(0xde3a), 0x5200 },
	{ CCI_REG16(0xde3c), 0x90b2 },
	{ CCI_REG16(0xde3e), 0x2880 },
	{ CCI_REG16(0xde40), 0xc8f8 },
	{ CCI_REG16(0xde42), 0x2c00 },
	{ CCI_REG16(0xde44), 0xdde7 },
	{ CCI_REG16(0xde46), 0x2de9 },
	{ CCI_REG16(0xde48), 0xf041 },
	{ CCI_REG16(0xde4a), 0x9a4e },
	{ CCI_REG16(0xde4c), 0x8046 },
	{ CCI_REG16(0xde4e), 0x9a4c },
	{ CCI_REG16(0xde50), 0xb6f8 },
	{ CCI_REG16(0xde52), 0x6600 },
	{ CCI_REG16(0xde54), 0x40b9 },
	{ CCI_REG16(0xde56), 0xb6f8 },
	{ CCI_REG16(0xde58), 0x6800 },
	{ CCI_REG16(0xde5a), 0x28b9 },
	{ CCI_REG16(0xde5c), 0xb6f8 },
	{ CCI_REG16(0xde5e), 0x6200 },
	{ CCI_REG16(0xde60), 0x10b9 },
	{ CCI_REG16(0xde62), 0xb6f8 },
	{ CCI_REG16(0xde64), 0x6400 },
	{ CCI_REG16(0xde66), 0x88b1 },
	{ CCI_REG16(0xde68), 0xfff7 },
	{ CCI_REG16(0xde6a), 0x1ffe },
	{ CCI_REG16(0xde6c), 0x30b1 },
	{ CCI_REG16(0xde6e), 0xb6f8 },
	{ CCI_REG16(0xde70), 0x6600 },
	{ CCI_REG16(0xde72), 0xa4f8 },
	{ CCI_REG16(0xde74), 0xfc0b },
	{ CCI_REG16(0xde76), 0xb6f8 },
	{ CCI_REG16(0xde78), 0x6800 },
	{ CCI_REG16(0xde7a), 0x05e0 },
	{ CCI_REG16(0xde7c), 0xb6f8 },
	{ CCI_REG16(0xde7e), 0x6200 },
	{ CCI_REG16(0xde80), 0xa4f8 },
	{ CCI_REG16(0xde82), 0xfc0b },
	{ CCI_REG16(0xde84), 0xb6f8 },
	{ CCI_REG16(0xde86), 0x6400 },
	{ CCI_REG16(0xde88), 0xa4f8 },
	{ CCI_REG16(0xde8a), 0xfe0b },
	{ CCI_REG16(0xde8c), 0x00f0 },
	{ CCI_REG16(0xde8e), 0xd4f9 },
	{ CCI_REG16(0xde90), 0x0746 },
	{ CCI_REG16(0xde92), 0x00f0 },
	{ CCI_REG16(0xde94), 0xd6f9 },
	{ CCI_REG16(0xde96), 0x791e },
	{ CCI_REG16(0xde98), 0x824d },
	{ CCI_REG16(0xde9a), 0x0329 },
	{ CCI_REG16(0xde9c), 0x1cd2 },
	{ CCI_REG16(0xde9e), 0x0128 },
	{ CCI_REG16(0xdea0), 0x06d0 },
	{ CCI_REG16(0xdea2), 0x0228 },
	{ CCI_REG16(0xdea4), 0x10d0 },
	{ CCI_REG16(0xdea6), 0xe88a },
	{ CCI_REG16(0xdea8), 0xa4f8 },
	{ CCI_REG16(0xdeaa), 0xf20b },
	{ CCI_REG16(0xdeac), 0x288b },
	{ CCI_REG16(0xdeae), 0x11e0 },
	{ CCI_REG16(0xdeb0), 0x012f },
	{ CCI_REG16(0xdeb2), 0x04d0 },
	{ CCI_REG16(0xdeb4), 0xa88b },
	{ CCI_REG16(0xdeb6), 0xa4f8 },
	{ CCI_REG16(0xdeb8), 0xf20b },
	{ CCI_REG16(0xdeba), 0x288c },
	{ CCI_REG16(0xdebc), 0x0ae0 },
	{ CCI_REG16(0xdebe), 0x688b },
	{ CCI_REG16(0xdec0), 0xa4f8 },
	{ CCI_REG16(0xdec2), 0xf20b },
	{ CCI_REG16(0xdec4), 0xe88b },
	{ CCI_REG16(0xdec6), 0x05e0 },
	{ CCI_REG16(0xdec8), 0x012f },
	{ CCI_REG16(0xdeca), 0x2bd0 },
	{ CCI_REG16(0xdecc), 0xa88c },
	{ CCI_REG16(0xdece), 0xa4f8 },
	{ CCI_REG16(0xded0), 0xf20b },
	{ CCI_REG16(0xded2), 0x288d },
	{ CCI_REG16(0xded4), 0xa4f8 },
	{ CCI_REG16(0xded6), 0xf80b },
	{ CCI_REG16(0xded8), 0x7048 },
	{ CCI_REG16(0xdeda), 0x0022 },
	{ CCI_REG16(0xdedc), 0xd0f8 },
	{ CCI_REG16(0xdede), 0x8040 },
	{ CCI_REG16(0xdee0), 0x200c },
	{ CCI_REG16(0xdee2), 0xa1b2 },
	{ CCI_REG16(0xdee4), 0x0746 },
	{ CCI_REG16(0xdee6), 0xfff7 },
	{ CCI_REG16(0xdee8), 0x77fd },
	{ CCI_REG16(0xdeea), 0x4046 },
	{ CCI_REG16(0xdeec), 0xfff7 },
	{ CCI_REG16(0xdeee), 0x82fe },
	{ CCI_REG16(0xdef0), 0xa1b2 },
	{ CCI_REG16(0xdef2), 0x0122 },
	{ CCI_REG16(0xdef4), 0x3846 },
	{ CCI_REG16(0xdef6), 0xfff7 },
	{ CCI_REG16(0xdef8), 0x6ffd },
	{ CCI_REG16(0xdefa), 0x7048 },
	{ CCI_REG16(0xdefc), 0x90f8 },
	{ CCI_REG16(0xdefe), 0xf214 },
	{ CCI_REG16(0xdf00), 0x21b1 },
	{ CCI_REG16(0xdf02), 0x6549 },
	{ CCI_REG16(0xdf04), 0x90f8 },
	{ CCI_REG16(0xdf06), 0xf304 },
	{ CCI_REG16(0xdf08), 0x81f8 },
	{ CCI_REG16(0xdf0a), 0xc400 },
	{ CCI_REG16(0xdf0c), 0xb8f1 },
	{ CCI_REG16(0xdf0e), 0x000f },
	{ CCI_REG16(0xdf10), 0x14d0 },
	{ CCI_REG16(0xdf12), 0x96f8 },
	{ CCI_REG16(0xdf14), 0xc600 },
	{ CCI_REG16(0xdf16), 0x88b1 },
	{ CCI_REG16(0xdf18), 0x6948 },
	{ CCI_REG16(0xdf1a), 0x0068 },
	{ CCI_REG16(0xdf1c), 0x8078 },
	{ CCI_REG16(0xdf1e), 0xa028 },
	{ CCI_REG16(0xdf20), 0x05d0 },
	{ CCI_REG16(0xdf22), 0x08e0 },
	{ CCI_REG16(0xdf24), 0x688c },
	{ CCI_REG16(0xdf26), 0xa4f8 },
	{ CCI_REG16(0xdf28), 0xf20b },
	{ CCI_REG16(0xdf2a), 0xe88c },
	{ CCI_REG16(0xdf2c), 0xd2e7 },
	{ CCI_REG16(0xdf2e), 0x6549 },
	{ CCI_REG16(0xdf30), 0x0320 },
	{ CCI_REG16(0xdf32), 0x81f8 },
	{ CCI_REG16(0xdf34), 0x9700 },
	{ CCI_REG16(0xdf36), 0x0120 },
	{ CCI_REG16(0xdf38), 0xfff7 },
	{ CCI_REG16(0xdf3a), 0x61fe },
	{ CCI_REG16(0xdf3c), 0xfff7 },
	{ CCI_REG16(0xdf3e), 0xe7fd },
	{ CCI_REG16(0xdf40), 0x28b1 },
	{ CCI_REG16(0xdf42), 0xfff7 },
	{ CCI_REG16(0xdf44), 0xa8fd },
	{ CCI_REG16(0xdf46), 0x10b1 },
	{ CCI_REG16(0xdf48), 0x95f8 },
	{ CCI_REG16(0xdf4a), 0xac00 },
	{ CCI_REG16(0xdf4c), 0x05e0 },
	{ CCI_REG16(0xdf4e), 0xfff7 },
	{ CCI_REG16(0xdf50), 0xa2fd },
	{ CCI_REG16(0xdf52), 0x08b1 },
	{ CCI_REG16(0xdf54), 0x0b20 },
	{ CCI_REG16(0xdf56), 0x00e0 },
	{ CCI_REG16(0xdf58), 0x0a20 },
	{ CCI_REG16(0xdf5a), 0xbde8 },
	{ CCI_REG16(0xdf5c), 0xf041 },
	{ CCI_REG16(0xdf5e), 0x4ff6 },
	{ CCI_REG16(0xdf60), 0xff71 },
	{ CCI_REG16(0xdf62), 0x01ea },
	{ CCI_REG16(0xdf64), 0x0012 },
	{ CCI_REG16(0xdf66), 0xf021 },
	{ CCI_REG16(0xdf68), 0x4ff6 },
	{ CCI_REG16(0xdf6a), 0x4a60 },
	{ CCI_REG16(0xdf6c), 0xfff7 },
	{ CCI_REG16(0xdf6e), 0x6fbe },
	{ CCI_REG16(0xdf70), 0x38b5 },
	{ CCI_REG16(0xdf72), 0x0024 },
	{ CCI_REG16(0xdf74), 0x8df8 },
	{ CCI_REG16(0xdf76), 0x0040 },
	{ CCI_REG16(0xdf78), 0x0122 },
	{ CCI_REG16(0xdf7a), 0x6946 },
	{ CCI_REG16(0xdf7c), 0x4ff4 },
	{ CCI_REG16(0xdf7e), 0xa070 },
	{ CCI_REG16(0xdf80), 0xfff7 },
	{ CCI_REG16(0xdf82), 0x5bfe },
	{ CCI_REG16(0xdf84), 0x9df8 },
	{ CCI_REG16(0xdf86), 0x0000 },
	{ CCI_REG16(0xdf88), 0x0328 },
	{ CCI_REG16(0xdf8a), 0x01d3 },
	{ CCI_REG16(0xdf8c), 0x0121 },
	{ CCI_REG16(0xdf8e), 0x00e0 },
	{ CCI_REG16(0xdf90), 0x0021 },
	{ CCI_REG16(0xdf92), 0x444a },
	{ CCI_REG16(0xdf94), 0x4c4d },
	{ CCI_REG16(0xdf96), 0x82f8 },
	{ CCI_REG16(0xdf98), 0xd610 },
	{ CCI_REG16(0xdf9a), 0x2978 },
	{ CCI_REG16(0xdf9c), 0x0029 },
	{ CCI_REG16(0xdf9e), 0x51d0 },
	{ CCI_REG16(0xdfa0), 0x0328 },
	{ CCI_REG16(0xdfa2), 0x4ed3 },
	{ CCI_REG16(0xdfa4), 0x2022 },
	{ CCI_REG16(0xdfa6), 0x4949 },
	{ CCI_REG16(0xdfa8), 0x4948 },
	{ CCI_REG16(0xdfaa), 0xfff7 },
	{ CCI_REG16(0xdfac), 0x24fd },
	{ CCI_REG16(0xdfae), 0x4848 },
	{ CCI_REG16(0xdfb0), 0x2022 },
	{ CCI_REG16(0xdfb2), 0x4849 },
	{ CCI_REG16(0xdfb4), 0x8030 },
	{ CCI_REG16(0xdfb6), 0xfff7 },
	{ CCI_REG16(0xdfb8), 0x1efd },
	{ CCI_REG16(0xdfba), 0x4748 },
	{ CCI_REG16(0xdfbc), 0x4749 },
	{ CCI_REG16(0xdfbe), 0x0288 },
	{ CCI_REG16(0xdfc0), 0xa1f8 },
	{ CCI_REG16(0xdfc2), 0xfc24 },
	{ CCI_REG16(0xdfc4), 0x4288 },
	{ CCI_REG16(0xdfc6), 0xa1f8 },
	{ CCI_REG16(0xdfc8), 0x0025 },
	{ CCI_REG16(0xdfca), 0x8288 },
	{ CCI_REG16(0xdfcc), 0xa1f8 },
	{ CCI_REG16(0xdfce), 0x0c25 },
	{ CCI_REG16(0xdfd0), 0xc088 },
	{ CCI_REG16(0xdfd2), 0xa1f8 },
	{ CCI_REG16(0xdfd4), 0x1005 },
	{ CCI_REG16(0xdfd6), 0x1822 },
	{ CCI_REG16(0xdfd8), 0x4149 },
	{ CCI_REG16(0xdfda), 0x4248 },
	{ CCI_REG16(0xdfdc), 0xfff7 },
	{ CCI_REG16(0xdfde), 0x0bfd },
	{ CCI_REG16(0xdfe0), 0x3f49 },
	{ CCI_REG16(0xdfe2), 0x4048 },
	{ CCI_REG16(0xdfe4), 0x1822 },
	{ CCI_REG16(0xdfe6), 0x1831 },
	{ CCI_REG16(0xdfe8), 0x2830 },
	{ CCI_REG16(0xdfea), 0xfff7 },
	{ CCI_REG16(0xdfec), 0x04fd },
	{ CCI_REG16(0xdfee), 0x3c49 },
	{ CCI_REG16(0xdff0), 0x3c48 },
	{ CCI_REG16(0xdff2), 0x1822 },
	{ CCI_REG16(0xdff4), 0x3031 },
	{ CCI_REG16(0xdff6), 0x5030 },
	{ CCI_REG16(0xdff8), 0xfff7 },
	{ CCI_REG16(0xdffa), 0xfdfc },
	{ CCI_REG16(0xdffc), 0x3849 },
	{ CCI_REG16(0xdffe), 0x3948 },
	{ CCI_REG16(0xe000), 0x1822 },
	{ CCI_REG16(0xe002), 0x4831 },
	{ CCI_REG16(0xe004), 0x7830 },
	{ CCI_REG16(0xe006), 0xfff7 },
	{ CCI_REG16(0xe008), 0xf6fc },
	{ CCI_REG16(0xe00a), 0x3549 },
	{ CCI_REG16(0xe00c), 0x3548 },
	{ CCI_REG16(0xe00e), 0x1822 },
	{ CCI_REG16(0xe010), 0x6031 },
	{ CCI_REG16(0xe012), 0xa030 },
	{ CCI_REG16(0xe014), 0xfff7 },
	{ CCI_REG16(0xe016), 0xeffc },
	{ CCI_REG16(0xe018), 0x3149 },
	{ CCI_REG16(0xe01a), 0x3248 },
	{ CCI_REG16(0xe01c), 0x1822 },
	{ CCI_REG16(0xe01e), 0x7831 },
	{ CCI_REG16(0xe020), 0xc830 },
	{ CCI_REG16(0xe022), 0xfff7 },
	{ CCI_REG16(0xe024), 0xe8fc },
	{ CCI_REG16(0xe026), 0x2e49 },
	{ CCI_REG16(0xe028), 0x2e48 },
	{ CCI_REG16(0xe02a), 0x1822 },
	{ CCI_REG16(0xe02c), 0x9031 },
	{ CCI_REG16(0xe02e), 0xf030 },
	{ CCI_REG16(0xe030), 0xfff7 },
	{ CCI_REG16(0xe032), 0xe1fc },
	{ CCI_REG16(0xe034), 0x2a49 },
	{ CCI_REG16(0xe036), 0x2648 },
	{ CCI_REG16(0xe038), 0x1822 },
	{ CCI_REG16(0xe03a), 0xa831 },
	{ CCI_REG16(0xe03c), 0xd238 },
	{ CCI_REG16(0xe03e), 0xfff7 },
	{ CCI_REG16(0xe040), 0xdafc },
	{ CCI_REG16(0xe042), 0x2c70 },
	{ CCI_REG16(0xe044), 0x1548 },
	{ CCI_REG16(0xe046), 0x0022 },
	{ CCI_REG16(0xe048), 0xd0f8 },
	{ CCI_REG16(0xe04a), 0x9840 },
	{ CCI_REG16(0xe04c), 0x200c },
	{ CCI_REG16(0xe04e), 0xa1b2 },
	{ CCI_REG16(0xe050), 0x0546 },
	{ CCI_REG16(0xe052), 0xfff7 },
	{ CCI_REG16(0xe054), 0xc1fc },
	{ CCI_REG16(0xe056), 0x00f0 },
	{ CCI_REG16(0xe058), 0xf9f8 },
	{ CCI_REG16(0xe05a), 0xa1b2 },
	{ CCI_REG16(0xe05c), 0x0122 },
	{ CCI_REG16(0xe05e), 0x2846 },
	{ CCI_REG16(0xe060), 0xfff7 },
	{ CCI_REG16(0xe062), 0xbafc },
	{ CCI_REG16(0xe064), 0x38bd },
	{ CCI_REG16(0xe066), 0x70b5 },
	{ CCI_REG16(0xe068), 0x0646 },
	{ CCI_REG16(0xe06a), 0x0d48 },
	{ CCI_REG16(0xe06c), 0x406f },
	{ CCI_REG16(0xe06e), 0x00f0 },
	{ CCI_REG16(0xe070), 0xf2f8 },
	{ CCI_REG16(0xe072), 0x0a48 },
	{ CCI_REG16(0xe074), 0x0022 },
	{ CCI_REG16(0xe076), 0xd0f8 },
	{ CCI_REG16(0xe078), 0xa040 },
	{ CCI_REG16(0xe07a), 0x250c },
	{ CCI_REG16(0xe07c), 0xa1b2 },
	{ CCI_REG16(0xe07e), 0x2846 },
	{ CCI_REG16(0xe080), 0xfff7 },
	{ CCI_REG16(0xe082), 0xaafc },
	{ CCI_REG16(0xe084), 0x3046 },
	{ CCI_REG16(0xe086), 0x00f0 },
	{ CCI_REG16(0xe088), 0xebf8 },
	{ CCI_REG16(0xe08a), 0xa1b2 },
	{ CCI_REG16(0xe08c), 0x2846 },
	{ CCI_REG16(0xe08e), 0xbde8 },
	{ CCI_REG16(0xe090), 0x7040 },
	{ CCI_REG16(0xe092), 0x0122 },
	{ CCI_REG16(0xe094), 0xfff7 },
	{ CCI_REG16(0xe096), 0xa0bc },
	{ CCI_REG16(0xe098), 0x2001 },
	{ CCI_REG16(0xe09a), 0x5700 },
	{ CCI_REG16(0xe09c), 0x2001 },
	{ CCI_REG16(0xe09e), 0x5540 },
	{ CCI_REG16(0xe0a0), 0x2001 },
	{ CCI_REG16(0xe0a2), 0x3a50 },
	{ CCI_REG16(0xe0a4), 0x2002 },
	{ CCI_REG16(0xe0a6), 0x8220 },
	{ CCI_REG16(0xe0a8), 0x2001 },
	{ CCI_REG16(0xe0aa), 0xe280 },
	{ CCI_REG16(0xe0ac), 0x2001 },
	{ CCI_REG16(0xe0ae), 0x3080 },
	{ CCI_REG16(0xe0b0), 0x2001 },
	{ CCI_REG16(0xe0b2), 0x2ff0 },
	{ CCI_REG16(0xe0b4), 0x2002 },
	{ CCI_REG16(0xe0b6), 0x8120 },
	{ CCI_REG16(0xe0b8), 0x2000 },
	{ CCI_REG16(0xe0ba), 0xa1f0 },
	{ CCI_REG16(0xe0bc), 0x2000 },
	{ CCI_REG16(0xe0be), 0xb1f0 },
	{ CCI_REG16(0xe0c0), 0x2000 },
	{ CCI_REG16(0xe0c2), 0x0d90 },
	{ CCI_REG16(0xe0c4), 0x2000 },
	{ CCI_REG16(0xe0c6), 0x0e00 },
	{ CCI_REG16(0xe0c8), 0x2002 },
	{ CCI_REG16(0xe0ca), 0x8000 },
	{ CCI_REG16(0xe0cc), 0x2002 },
	{ CCI_REG16(0xe0ce), 0x8002 },
	{ CCI_REG16(0xe0d0), 0x2000 },
	{ CCI_REG16(0xe0d2), 0xd15a },
	{ CCI_REG16(0xe0d4), 0x2002 },
	{ CCI_REG16(0xe0d6), 0x8022 },
	{ CCI_REG16(0xe0d8), 0x2002 },
	{ CCI_REG16(0xe0da), 0x8042 },
	{ CCI_REG16(0xe0dc), 0x2000 },
	{ CCI_REG16(0xe0de), 0xcde0 },
	{ CCI_REG16(0xe0e0), 0x2002 },
	{ CCI_REG16(0xe0e2), 0x804a },
	{ CCI_REG16(0xe0e4), 0x2000 },
	{ CCI_REG16(0xe0e6), 0xcf70 },
	{ CCI_REG16(0xe0e8), 0x2de9 },
	{ CCI_REG16(0xe0ea), 0xfc5f },
	{ CCI_REG16(0xe0ec), 0x414e },
	{ CCI_REG16(0xe0ee), 0x8346 },
	{ CCI_REG16(0xe0f0), 0x0024 },
	{ CCI_REG16(0xe0f2), 0x96f8 },
	{ CCI_REG16(0xe0f4), 0xad00 },
	{ CCI_REG16(0xe0f6), 0xdff8 },
	{ CCI_REG16(0xe0f8), 0x0091 },
	{ CCI_REG16(0xe0fa), 0x2546 },
	{ CCI_REG16(0xe0fc), 0x38b9 },
	{ CCI_REG16(0xe0fe), 0x96f8 },
	{ CCI_REG16(0xe100), 0x2a00 },
	{ CCI_REG16(0xe102), 0x0028 },
	{ CCI_REG16(0xe104), 0x5fd0 },
	{ CCI_REG16(0xe106), 0x00f0 },
	{ CCI_REG16(0xe108), 0xb0f8 },
	{ CCI_REG16(0xe10a), 0x0328 },
	{ CCI_REG16(0xe10c), 0x12d0 },
	{ CCI_REG16(0xe10e), 0xfff7 },
	{ CCI_REG16(0xe110), 0xc2fc },
	{ CCI_REG16(0xe112), 0x00b1 },
	{ CCI_REG16(0xe114), 0x0124 },
	{ CCI_REG16(0xe116), 0x3948 },
	{ CCI_REG16(0xe118), 0x90f8 },
	{ CCI_REG16(0xe11a), 0x190c },
	{ CCI_REG16(0xe11c), 0x38b9 },
	{ CCI_REG16(0xe11e), 0x99f8 },
	{ CCI_REG16(0xe120), 0x4d09 },
	{ CCI_REG16(0xe122), 0x10b9 },
	{ CCI_REG16(0xe124), 0x99f8 },
	{ CCI_REG16(0xe126), 0x4e09 },
	{ CCI_REG16(0xe128), 0x18b1 },
	{ CCI_REG16(0xe12a), 0x401e },
	{ CCI_REG16(0xe12c), 0xc0b2 },
	{ CCI_REG16(0xe12e), 0x0228 },
	{ CCI_REG16(0xe130), 0x5dd0 },
	{ CCI_REG16(0xe132), 0x0025 },
	{ CCI_REG16(0xe134), 0x96f8 },
	{ CCI_REG16(0xe136), 0x2a00 },
	{ CCI_REG16(0xe138), 0x20b3 },
	{ CCI_REG16(0xe13a), 0x00f0 },
	{ CCI_REG16(0xe13c), 0x96f8 },
	{ CCI_REG16(0xe13e), 0x0328 },
	{ CCI_REG16(0xe140), 0x20d0 },
	{ CCI_REG16(0xe142), 0xdff8 },
	{ CCI_REG16(0xe144), 0xbca0 },
	{ CCI_REG16(0xe146), 0x0023 },
	{ CCI_REG16(0xe148), 0x06eb },
	{ CCI_REG16(0xe14a), 0x8311 },
	{ CCI_REG16(0xe14c), 0x03eb },
	{ CCI_REG16(0xe14e), 0x4302 },
	{ CCI_REG16(0xe150), 0x01eb },
	{ CCI_REG16(0xe152), 0x4411 },
	{ CCI_REG16(0xe154), 0x0020 },
	{ CCI_REG16(0xe156), 0x0aeb },
	{ CCI_REG16(0xe158), 0x0218 },
	{ CCI_REG16(0xe15a), 0x01eb },
	{ CCI_REG16(0xe15c), 0x051c },
	{ CCI_REG16(0xe15e), 0x0ceb },
	{ CCI_REG16(0xe160), 0x4002 },
	{ CCI_REG16(0xe162), 0x08eb },
	{ CCI_REG16(0xe164), 0x4001 },
	{ CCI_REG16(0xe166), 0x978d },
	{ CCI_REG16(0xe168), 0x21f8 },
	{ CCI_REG16(0xe16a), 0xaa7f },
	{ CCI_REG16(0xe16c), 0x178e },
	{ CCI_REG16(0xe16e), 0x8f80 },
	{ CCI_REG16(0xe170), 0x978e },
	{ CCI_REG16(0xe172), 0x0f81 },
	{ CCI_REG16(0xe174), 0x128f },
	{ CCI_REG16(0xe176), 0x401c },
	{ CCI_REG16(0xe178), 0x8a81 },
	{ CCI_REG16(0xe17a), 0x0228 },
	{ CCI_REG16(0xe17c), 0xefdb },
	{ CCI_REG16(0xe17e), 0x5b1c },
	{ CCI_REG16(0xe180), 0x022b },
	{ CCI_REG16(0xe182), 0xe1db },
	{ CCI_REG16(0xe184), 0x96f8 },
	{ CCI_REG16(0xe186), 0xad00 },
	{ CCI_REG16(0xe188), 0xe8b1 },
	{ CCI_REG16(0xe18a), 0x0020 },
	{ CCI_REG16(0xe18c), 0x0190 },
	{ CCI_REG16(0xe18e), 0xb9f8 },
	{ CCI_REG16(0xe190), 0x2021 },
	{ CCI_REG16(0xe192), 0x01a8 },
	{ CCI_REG16(0xe194), 0x0090 },
	{ CCI_REG16(0xe196), 0x0023 },
	{ CCI_REG16(0xe198), 0x0621 },
	{ CCI_REG16(0xe19a), 0x1a48 },
	{ CCI_REG16(0xe19c), 0x00f0 },
	{ CCI_REG16(0xe19e), 0x42f8 },
	{ CCI_REG16(0xe1a0), 0x81b2 },
	{ CCI_REG16(0xe1a2), 0x96f8 },
	{ CCI_REG16(0xe1a4), 0xd600 },
	{ CCI_REG16(0xe1a6), 0x04eb },
	{ CCI_REG16(0xe1a8), 0x8402 },
	{ CCI_REG16(0xe1aa), 0x00eb },
	{ CCI_REG16(0xe1ac), 0x8000 },
	{ CCI_REG16(0xe1ae), 0x06eb },
	{ CCI_REG16(0xe1b0), 0x8000 },
	{ CCI_REG16(0xe1b2), 0x00eb },
	{ CCI_REG16(0xe1b4), 0x4200 },
	{ CCI_REG16(0xe1b6), 0x05eb },
	{ CCI_REG16(0xe1b8), 0x8502 },
	{ CCI_REG16(0xe1ba), 0x1044 },
	{ CCI_REG16(0xe1bc), 0x0844 },
	{ CCI_REG16(0xe1be), 0x1249 },
	{ CCI_REG16(0xe1c0), 0x90f8 },
	{ CCI_REG16(0xe1c2), 0xae00 },
	{ CCI_REG16(0xe1c4), 0x0880 },
	{ CCI_REG16(0xe1c6), 0x1148 },
	{ CCI_REG16(0xe1c8), 0x0022 },
	{ CCI_REG16(0xe1ca), 0xd0f8 },
	{ CCI_REG16(0xe1cc), 0x9c40 },
	{ CCI_REG16(0xe1ce), 0x200c },
	{ CCI_REG16(0xe1d0), 0xa1b2 },
	{ CCI_REG16(0xe1d2), 0x0546 },
	{ CCI_REG16(0xe1d4), 0xfff7 },
	{ CCI_REG16(0xe1d6), 0x00fc },
	{ CCI_REG16(0xe1d8), 0x5846 },
	{ CCI_REG16(0xe1da), 0x00f0 },
	{ CCI_REG16(0xe1dc), 0x4bf8 },
	{ CCI_REG16(0xe1de), 0x02b0 },
	{ CCI_REG16(0xe1e0), 0xa1b2 },
	{ CCI_REG16(0xe1e2), 0x2846 },
	{ CCI_REG16(0xe1e4), 0xbde8 },
	{ CCI_REG16(0xe1e6), 0xf05f },
	{ CCI_REG16(0xe1e8), 0x0122 },
	{ CCI_REG16(0xe1ea), 0xfff7 },
	{ CCI_REG16(0xe1ec), 0xf5bb },
	{ CCI_REG16(0xe1ee), 0x0125 },
	{ CCI_REG16(0xe1f0), 0xa0e7 },
	{ CCI_REG16(0xe1f2), 0x0000 },
	{ CCI_REG16(0xe1f4), 0x2002 },
	{ CCI_REG16(0xe1f6), 0x8220 },
	{ CCI_REG16(0xe1f8), 0x2001 },
	{ CCI_REG16(0xe1fa), 0x5700 },
	{ CCI_REG16(0xe1fc), 0x2000 },
	{ CCI_REG16(0xe1fe), 0xa1f0 },
	{ CCI_REG16(0xe200), 0x2001 },
	{ CCI_REG16(0xe202), 0x09e0 },
	{ CCI_REG16(0xe204), 0x2000 },
	{ CCI_REG16(0xe206), 0xae0a },
	{ CCI_REG16(0xe208), 0x4000 },
	{ CCI_REG16(0xe20a), 0xee0c },
	{ CCI_REG16(0xe20c), 0x2001 },
	{ CCI_REG16(0xe20e), 0x5540 },
	{ CCI_REG16(0xe210), 0x45f6 },
	{ CCI_REG16(0xe212), 0xf14c },
	{ CCI_REG16(0xe214), 0xc0f2 },
	{ CCI_REG16(0xe216), 0x020c },
	{ CCI_REG16(0xe218), 0x6047 },
	{ CCI_REG16(0xe21a), 0x4cf2 },
	{ CCI_REG16(0xe21c), 0x030c },
	{ CCI_REG16(0xe21e), 0xc0f2 },
	{ CCI_REG16(0xe220), 0x020c },
	{ CCI_REG16(0xe222), 0x6047 },
	{ CCI_REG16(0xe224), 0x4bf2 },
	{ CCI_REG16(0xe226), 0x872c },
	{ CCI_REG16(0xe228), 0xc0f2 },
	{ CCI_REG16(0xe22a), 0x010c },
	{ CCI_REG16(0xe22c), 0x6047 },
	{ CCI_REG16(0xe22e), 0x49f6 },
	{ CCI_REG16(0xe230), 0x232c },
	{ CCI_REG16(0xe232), 0xc0f2 },
	{ CCI_REG16(0xe234), 0x020c },
	{ CCI_REG16(0xe236), 0x6047 },
	{ CCI_REG16(0xe238), 0x48f6 },
	{ CCI_REG16(0xe23a), 0x655c },
	{ CCI_REG16(0xe23c), 0xc0f2 },
	{ CCI_REG16(0xe23e), 0x020c },
	{ CCI_REG16(0xe240), 0x6047 },
	{ CCI_REG16(0xe242), 0x48f6 },
	{ CCI_REG16(0xe244), 0x6d5c },
	{ CCI_REG16(0xe246), 0xc0f2 },
	{ CCI_REG16(0xe248), 0x020c },
	{ CCI_REG16(0xe24a), 0x6047 },
	{ CCI_REG16(0xe24c), 0x49f2 },
	{ CCI_REG16(0xe24e), 0x132c },
	{ CCI_REG16(0xe250), 0xc0f2 },
	{ CCI_REG16(0xe252), 0x000c },
	{ CCI_REG16(0xe254), 0x6047 },
	{ CCI_REG16(0xe256), 0x47f2 },
	{ CCI_REG16(0xe258), 0xb10c },
	{ CCI_REG16(0xe25a), 0xc0f2 },
	{ CCI_REG16(0xe25c), 0x020c },
	{ CCI_REG16(0xe25e), 0x6047 },
	{ CCI_REG16(0xe260), 0x48f2 },
	{ CCI_REG16(0xe262), 0x5b4c },
	{ CCI_REG16(0xe264), 0xc0f2 },
	{ CCI_REG16(0xe266), 0x010c },
	{ CCI_REG16(0xe268), 0x6047 },
	{ CCI_REG16(0xe26a), 0x42f6 },
	{ CCI_REG16(0xe26c), 0x5d3c },
	{ CCI_REG16(0xe26e), 0xc0f2 },
	{ CCI_REG16(0xe270), 0x010c },
	{ CCI_REG16(0xe272), 0x6047 },
	{ CCI_REG16(0xe274), 0x4cf2 },
	{ CCI_REG16(0xe276), 0x512c },
	{ CCI_REG16(0xe278), 0xc0f2 },
	{ CCI_REG16(0xe27a), 0x000c },
	{ CCI_REG16(0xe27c), 0x6047 },
	{ CCI_REG16(0xe27e), 0x0000 },
	{ CCI_REG16(0xe280), 0x0053 },
	{ CCI_REG16(0xe282), 0x0064 },
	{ CCI_REG16(0xe284), 0x0071 },
	{ CCI_REG16(0xe286), 0x007d },
	{ CCI_REG16(0xe288), 0x00be },
	{ CCI_REG16(0xe28a), 0x00c8 },
	{ CCI_REG16(0xe28c), 0x00fa },
	{ CCI_REG16(0xe28e), 0x01f4 },
	{ CCI_REG16(0xe290), 0x03e8 },
	{ CCI_REG16(0xe292), 0x07d0 },
	{ CCI_REG16(0xe294), 0x0fa0 },
	{ CCI_REG16(0xe296), 0x1f40 },
	{ CCI_REG16(0xe298), 0x3a98 },
	{ CCI_REG16(0xe29a), 0x7530 },
	{ CCI_REG16(0xe29c), 0x0159 },
	{ CCI_REG16(0xe29e), 0x016e },
	{ CCI_REG16(0xe2a0), 0x0182 },
	{ CCI_REG16(0xe2a2), 0x0197 },
	{ CCI_REG16(0xe2a4), 0x01b0 },
	{ CCI_REG16(0xe2a6), 0x0000 },
	{ CCI_REG16(0xe2a8), 0x0000 },
	{ CCI_REG16(0xe2aa), 0x0000 },
	{ CCI_REG16(0xe2ac), 0x0000 },
	{ CCI_REG16(0xe2ae), 0x0000 },
	{ CCI_REG16(0xe2b0), 0x0000 },
	{ CCI_REG16(0xe2b2), 0x0000 },
	{ CCI_REG16(0xe2b4), 0x08e3 },
	{ CCI_REG16(0xe2b6), 0x05c1 },
	{ CCI_REG16(0xe2b8), 0x0000 },
	{ CCI_REG16(0xe2ba), 0x003f },
	{ CCI_REG16(0xe2bc), 0xeeff },
	{ CCI_REG16(0xe2be), 0xfbff },
	{ CCI_REG16(0x1218), 0x2001 },
	{ CCI_REG16(0x121a), 0xddcd },
	{ CCI_REG16(0x1270), 0x2001 },
	{ CCI_REG16(0x1272), 0xde47 },
	{ CCI_REG16(0x128c), 0x0002 },
	{ CCI_REG16(0x128e), 0xc003 },
	{ CCI_REG16(0x1290), 0x2001 },
	{ CCI_REG16(0x1292), 0xdca3 },
	{ CCI_REG16(0x1294), 0x0002 },
	{ CCI_REG16(0x1296), 0x9a23 },
	{ CCI_REG16(0x1298), 0x2001 },
	{ CCI_REG16(0x129a), 0xdcfb },
	{ CCI_REG16(0x129c), 0x0000 },
	{ CCI_REG16(0x129e), 0x9213 },
	{ CCI_REG16(0x12a0), 0x2001 },
	{ CCI_REG16(0x12a2), 0xdf71 },
	{ CCI_REG16(0x12a4), 0x0000 },
	{ CCI_REG16(0x12a6), 0xc251 },
	{ CCI_REG16(0x12a8), 0x2001 },
	{ CCI_REG16(0x12aa), 0xe0e9 },
	{ CCI_REG16(0x12ac), 0x0001 },
	{ CCI_REG16(0x12ae), 0x845b },
	{ CCI_REG16(0x12b0), 0x2001 },
	{ CCI_REG16(0x12b2), 0xe067 },
	{ CCI_REG16(0xfcfc), 0x2000 },
	{ CCI_REG16(0x0e3c), 0x7f06 },
	{ CCI_REG16(0x0e82), 0x0000 },
	{ CCI_REG16(0x0e84), 0x0101 },
	{ CCI_REG16(0x251c), 0x0000 },
	{ CCI_REG16(0x319c), 0x0100 },
	{ CCI_REG16(0x319e), 0x05dc },
	{ CCI_REG16(0x31a6), 0x0100 },
	{ CCI_REG16(0x3256), 0x0000 },
	{ CCI_REG16(0x3296), 0x0001 },
	{ CCI_REG16(0x339a), 0x0048 },
	{ CCI_REG16(0x339c), 0x0048 },
	{ CCI_REG16(0x33a0), 0x0048 },
	{ CCI_REG16(0x33a2), 0x0048 },
	{ CCI_REG16(0x33a4), 0x0048 },
	{ CCI_REG16(0x33ac), 0x0048 },
	{ CCI_REG16(0x33c2), 0x0048 },
	{ CCI_REG16(0x33c4), 0x0048 },
	{ CCI_REG16(0x33c8), 0x0048 },
	{ CCI_REG16(0x33ca), 0x0048 },
	{ CCI_REG16(0x33cc), 0x0048 },
	{ CCI_REG16(0x33d4), 0x0048 },
	{ CCI_REG16(0x3b9a), 0x0811 },
	{ CCI_REG16(0x3b9c), 0x0d6f },
	{ CCI_REG16(0x3bc0), 0x02b3 },
	{ CCI_REG16(0x3bc2), 0x0811 },
	{ CCI_REG16(0x3bc4), 0x0d6f },
	{ CCI_REG16(0xaa36), 0x0a0e },
	{ CCI_REG16(0xaa38), 0x0e0e },
	{ CCI_REG16(0xaa3c), 0x020e },
	{ CCI_REG16(0xaa4a), 0x0001 },
	{ CCI_REG16(0xaa4c), 0x0101 },
	{ CCI_REG16(0xaa50), 0x0001 },
	{ CCI_REG16(0xaa84), 0x1313 },
	{ CCI_REG16(0xaa86), 0x0b13 },
	{ CCI_REG16(0xaa88), 0x1313 },
	{ CCI_REG16(0xaa8c), 0x0313 },
	{ CCI_REG16(0xaaac), 0x2c30 },
	{ CCI_REG16(0xaad6), 0x0083 },
	{ CCI_REG16(0xaada), 0x34dd },
	{ CCI_REG16(0xaae2), 0x6281 },
	{ CCI_REG16(0xaaf0), 0x0100 },
	{ CCI_REG16(0xad70), 0x3ed4 },
	{ CCI_REG16(0xad72), 0x1fc0 },
	{ CCI_REG16(0xad76), 0xdf6a },
	{ CCI_REG16(0xadae), 0x0000 },
	{ CCI_REG16(0xadf2), 0x0233 },
	{ CCI_REG16(0xadf4), 0x0233 },
	{ CCI_REG16(0xadfe), 0x0020 },
	{ CCI_REG16(0xae00), 0x0026 },
	{ CCI_REG16(0xae0a), 0x0100 },
	{ CCI_REG16(0xae0c), 0x01ff },
	{ CCI_REG16(0xae0e), 0x03ff },
	{ CCI_REG16(0xae10), 0x07ff },
	{ CCI_REG16(0xae12), 0x0fff },
	{ CCI_REG16(0xae14), 0x4000 },
	{ CCI_REG16(0xbd00), 0x4e80 },
	{ CCI_REG16(0xbd30), 0xff40 },
	{ CCI_REG16(0xbd32), 0xff40 },
	{ CCI_REG16(0xc74c), 0x0100 },
	{ CCI_REG16(0xc74e), 0xff40 },
	{ CCI_REG16(0xc750), 0xff40 },
	{ CCI_REG16(0xc752), 0xff40 },
	{ CCI_REG16(0xc754), 0xff40 },
	{ CCI_REG16(0xc756), 0xff40 },
	{ CCI_REG16(0xc758), 0xff40 },
	{ CCI_REG16(0xc75a), 0xff40 },
	{ CCI_REG16(0xc75c), 0xff40 },
	{ CCI_REG16(0xc76e), 0x076c },
	{ CCI_REG16(0xc77e), 0x0100 },
	{ CCI_REG16(0xc780), 0xff40 },
	{ CCI_REG16(0xc782), 0xff40 },
	{ CCI_REG16(0xc784), 0xff40 },
	{ CCI_REG16(0xc786), 0xff40 },
	{ CCI_REG16(0xc788), 0xff40 },
	{ CCI_REG16(0xc78a), 0xff40 },
	{ CCI_REG16(0xc78c), 0xff40 },
	{ CCI_REG16(0xc78e), 0xff40 },
	{ CCI_REG16(0xcb00), 0x0001 },
	{ CCI_REG16(0xda64), 0xd51c },
	{ CCI_REG16(0xdade), 0xfffc },
	{ CCI_REG16(0xdae0), 0x00ff },
	{ CCI_REG16(0xdae2), 0xc000 },
	{ CCI_REG16(0xdae4), 0xffff },
	{ CCI_REG16(0xdae6), 0x000f },
	{ CCI_REG16(0xdae8), 0x0200 },
	{ CCI_REG16(0xdaea), 0x8000 },
	{ CCI_REG16(0xdaec), 0xffff },
	{ CCI_REG16(0xdaee), 0x001f },
	{ CCI_REG16(0xdaf0), 0xf400 },
	{ CCI_REG16(0xdaf2), 0xffff },
	{ CCI_REG16(0xdaf4), 0xffff },
	{ CCI_REG16(0xdaf6), 0x003f },
	{ CCI_REG16(0xdaf8), 0xe000 },
	{ CCI_REG16(0xdafa), 0xffff },
	{ CCI_REG16(0xdafc), 0x0003 },
	{ CCI_REG16(0xdafe), 0xff00 },
	{ CCI_REG16(0xdb00), 0x3fff },
	{ CCI_REG16(0xdb02), 0x0000 },
	{ CCI_REG16(0xdb04), 0xfff0 },
	{ CCI_REG16(0xdb06), 0x03ff },
	{ CCI_REG16(0xdb08), 0x0000 },
	{ CCI_REG16(0xdb0a), 0xffff },
	{ CCI_REG16(0xdb0c), 0xffff },
	{ CCI_REG16(0xdb0e), 0x0fff },
	{ CCI_REG16(0xdb10), 0x0000 },
	{ CCI_REG16(0xdb12), 0x0000 },
	{ CCI_REG16(0xdb14), 0x0000 },
	{ CCI_REG16(0xdb16), 0xc000 },
	{ CCI_REG16(0xdb18), 0xffff },
	{ CCI_REG16(0xdb1a), 0xffff },
	{ CCI_REG16(0xdb1c), 0x03ff },
	{ CCI_REG16(0xdb1e), 0x0000 },
	{ CCI_REG16(0xdb20), 0x0000 },
	{ CCI_REG16(0xdb22), 0x0000 },
	{ CCI_REG16(0xdb24), 0xf000 },
	{ CCI_REG16(0xdb26), 0xffff },
	{ CCI_REG16(0xdb28), 0xffff },
	{ CCI_REG16(0xdb2a), 0xfcff },
	{ CCI_REG16(0xdb2c), 0xffff },
	{ CCI_REG16(0xdb2e), 0xfffe },
	{ CCI_REG16(0xdb30), 0x003f },
	{ CCI_REG16(0xdb32), 0x0c00 },
	{ CCI_REG16(0xdb34), 0x0000 },
	{ CCI_REG16(0xdb36), 0x114a },
	{ CCI_REG16(0xdb38), 0x90d0 },
	{ CCI_REG16(0xdb3a), 0x33ff },
	{ CCI_REG16(0xdb3c), 0x00ca },
	{ CCI_REG16(0xdb3e), 0xd4d5 },
	{ CCI_REG16(0xdb40), 0xa03f },
	{ CCI_REG16(0xdb42), 0x0ffb },
	{ CCI_REG16(0xdb44), 0x0271 },
	{ CCI_REG16(0xdb46), 0x4584 },
	{ CCI_REG16(0xdb48), 0x9900 },
	{ CCI_REG16(0xdb4a), 0x402b },
	{ CCI_REG16(0xdb4c), 0x0043 },
	{ CCI_REG16(0xdb4e), 0xb610 },
	{ CCI_REG16(0xdb50), 0x03fe },
	{ CCI_REG16(0xdb52), 0x0048 },
	{ CCI_REG16(0xdb54), 0xff39 },
	{ CCI_REG16(0xdb56), 0x4f3f },
	{ CCI_REG16(0xdb58), 0xd00d },
	{ CCI_REG16(0xdb5a), 0x0270 },
	{ CCI_REG16(0xdb5c), 0x37f4 },
	{ CCI_REG16(0xdb5e), 0xb8ff },
	{ CCI_REG16(0xdb60), 0x3fb7 },
	{ CCI_REG16(0xdb62), 0xfffd },
	{ CCI_REG16(0xdb64), 0x00bf },
	{ CCI_REG16(0xdb66), 0xd000 },
	{ CCI_REG16(0xdb68), 0xffff },
	{ CCI_REG16(0xdb6a), 0x000b },
	{ CCI_REG16(0xdb6c), 0x0200 },
	{ CCI_REG16(0xdb6e), 0x8000 },
	{ CCI_REG16(0xdb70), 0xffff },
	{ CCI_REG16(0xdb72), 0x001f },
	{ CCI_REG16(0xdb74), 0xf400 },
	{ CCI_REG16(0xdb76), 0xffff },
	{ CCI_REG16(0xdb78), 0xffff },
	{ CCI_REG16(0xdb7a), 0x003f },
	{ CCI_REG16(0xdb7c), 0xf000 },
	{ CCI_REG16(0xdb7e), 0xffff },
	{ CCI_REG16(0xdb80), 0x0003 },
	{ CCI_REG16(0xdb82), 0xff00 },
	{ CCI_REG16(0xdb84), 0x3fff },
	{ CCI_REG16(0xdb86), 0x0000 },
	{ CCI_REG16(0xdb88), 0xfff0 },
	{ CCI_REG16(0xdb8a), 0x03ff },
	{ CCI_REG16(0xdb8c), 0x0000 },
	{ CCI_REG16(0xdb8e), 0xffff },
	{ CCI_REG16(0xdb90), 0xffff },
	{ CCI_REG16(0xdb92), 0x0fff },
	{ CCI_REG16(0xdb94), 0x0000 },
	{ CCI_REG16(0xdb96), 0x0000 },
	{ CCI_REG16(0xdb98), 0x0000 },
	{ CCI_REG16(0xdb9a), 0xc000 },
	{ CCI_REG16(0xdb9c), 0xffff },
	{ CCI_REG16(0xdb9e), 0xffff },
	{ CCI_REG16(0xdba0), 0x03ff },
	{ CCI_REG16(0xdba2), 0x0000 },
	{ CCI_REG16(0xdba4), 0x0000 },
	{ CCI_REG16(0xdba6), 0x0000 },
	{ CCI_REG16(0xdba8), 0xf000 },
	{ CCI_REG16(0xdbaa), 0xffff },
	{ CCI_REG16(0xdbac), 0xffff },
	{ CCI_REG16(0xdbae), 0xfdff },
	{ CCI_REG16(0xdbb0), 0x3fff },
	{ CCI_REG16(0xdbb2), 0xffff },
	{ CCI_REG16(0xdbb4), 0x002f },
	{ CCI_REG16(0xdbb6), 0x0800 },
	{ CCI_REG16(0xdbb8), 0x0000 },
	{ CCI_REG16(0xdbba), 0x0b95 },
	{ CCI_REG16(0xdbbc), 0xe210 },
	{ CCI_REG16(0xdbbe), 0x03ff },
	{ CCI_REG16(0xdbc0), 0x0080 },
	{ CCI_REG16(0xdbc2), 0xe185 },
	{ CCI_REG16(0xdbc4), 0xa63f },
	{ CCI_REG16(0xdbc6), 0x8ffb },
	{ CCI_REG16(0xdbc8), 0x01ed },
	{ CCI_REG16(0xdbca), 0x4fd4 },
	{ CCI_REG16(0xdbcc), 0x8500 },
	{ CCI_REG16(0xdbce), 0x4022 },
	{ CCI_REG16(0xdbd0), 0x00c3 },
	{ CCI_REG16(0xdbd2), 0x0f50 },
	{ CCI_REG16(0xdbd4), 0x43ff },
	{ CCI_REG16(0xdbd6), 0x0029 },
	{ CCI_REG16(0xdbd8), 0x0275 },
	{ CCI_REG16(0xdbda), 0xee40 },
	{ CCI_REG16(0xdbdc), 0x1008 },
	{ CCI_REG16(0xdbde), 0x0192 },
	{ CCI_REG16(0xdbe0), 0x8454 },
	{ CCI_REG16(0xdbe2), 0xccff },
	{ CCI_REG16(0xdbe4), 0x3fcd },
	{ CCI_REG16(0xdbe6), 0xffff },
	{ CCI_REG16(0xdbe8), 0x003f },
	{ CCI_REG16(0xdbea), 0xf000 },
	{ CCI_REG16(0xdbec), 0xffff },
	{ CCI_REG16(0xdbee), 0x0003 },
	{ CCI_REG16(0xdbf0), 0xff00 },
	{ CCI_REG16(0xdbf2), 0x3fff },
	{ CCI_REG16(0xdbf4), 0x0000 },
	{ CCI_REG16(0xdbf6), 0x0000 },
	{ CCI_REG16(0xdbf8), 0xfc00 },
	{ CCI_REG16(0xdbfa), 0xffff },
	{ CCI_REG16(0xdbfc), 0x0002 },
	{ CCI_REG16(0xdbfe), 0xff40 },
	{ CCI_REG16(0xdc00), 0x2fff },
	{ CCI_REG16(0xdc02), 0x0000 },
	{ CCI_REG16(0xdc04), 0xfff4 },
	{ CCI_REG16(0xdc06), 0xffff },
	{ CCI_REG16(0xdc08), 0x3fff },
	{ CCI_REG16(0xdc0a), 0x0000 },
	{ CCI_REG16(0xdc0c), 0xfff0 },
	{ CCI_REG16(0xdc0e), 0x03ff },
	{ CCI_REG16(0xdc10), 0x0000 },
	{ CCI_REG16(0xdc12), 0xfffe },
	{ CCI_REG16(0xdc14), 0xffbf },
	{ CCI_REG16(0xdc16), 0x1fff },
	{ CCI_REG16(0xdc18), 0x0000 },
	{ CCI_REG16(0xdc1a), 0x0004 },
	{ CCI_REG16(0xdc1c), 0xff00 },
	{ CCI_REG16(0xdc1e), 0x3fff },
	{ CCI_REG16(0xdc20), 0x0000 },
	{ CCI_REG16(0xdc22), 0x0000 },
	{ CCI_REG16(0xdc24), 0xfc00 },
	{ CCI_REG16(0xdc26), 0xffff },
	{ CCI_REG16(0xdc28), 0x0002 },
	{ CCI_REG16(0xdc2a), 0x0080 },
	{ CCI_REG16(0xdc2c), 0xd000 },
	{ CCI_REG16(0xdc2e), 0xffff },
	{ CCI_REG16(0xdc30), 0xfff7 },
	{ CCI_REG16(0xdc32), 0xffff },
	{ CCI_REG16(0xdc34), 0xffff },
	{ CCI_REG16(0xdc36), 0xffff },
	{ CCI_REG16(0xdc38), 0x000f },
	{ CCI_REG16(0xdc3a), 0x0000 },
	{ CCI_REG16(0xdc3c), 0x0000 },
	{ CCI_REG16(0xdc3e), 0x07df },
	{ CCI_REG16(0xdc40), 0x5490 },
	{ CCI_REG16(0xdc42), 0xc401 },
	{ CCI_REG16(0xdc44), 0xffe2 },
	{ CCI_REG16(0xdc46), 0xeea8 },
	{ CCI_REG16(0xdc48), 0x163f },
	{ CCI_REG16(0xdc4a), 0xcfff },
	{ CCI_REG16(0xdc4c), 0x002e },
	{ CCI_REG16(0xdc4e), 0xeb14 },
	{ CCI_REG16(0xdc50), 0xf4ff },
	{ CCI_REG16(0xdc52), 0x4007 },
	{ CCI_REG16(0xdc54), 0xef2d },
	{ CCI_REG16(0xdc56), 0x2a0f },
	{ CCI_REG16(0xdc58), 0xe401 },
	{ CCI_REG16(0xdc5a), 0xfff5 },
	{ CCI_REG16(0xdc5c), 0x5a40 },
	{ CCI_REG16(0xdc5e), 0x8640 },
	{ CCI_REG16(0xdc60), 0xd002 },
	{ CCI_REG16(0xdc62), 0xff76 },
	{ CCI_REG16(0xdc64), 0x08d3 },
	{ CCI_REG16(0xdc66), 0x1500 },
	{ CCI_REG16(0xdc68), 0x3ff1 },
	{ CCI_REG16(0xea48), 0x0320 },
	{ CCI_REG16(0xea4a), 0x0800 },
	{ CCI_REG16(0xfcfc), 0x2001 },
	{ CCI_REG16(0x0890), 0x0000 },
	{ CCI_REG16(0x0892), 0x0100 },
	{ CCI_REG16(0x0896), 0x0080 },
	{ CCI_REG16(0x08aa), 0x0000 },
	{ CCI_REG16(0x08ac), 0xffff },
	{ CCI_REG16(0x08ae), 0x0206 },
	{ CCI_REG16(0x08b2), 0x0001 },
	{ CCI_REG16(0x08b8), 0x0080 },
	{ CCI_REG16(0x08c0), 0x0000 },
	{ CCI_REG16(0x08c2), 0x0020 },
	{ CCI_REG16(0x08c4), 0x0800 },
	{ CCI_REG16(0xfcfc), 0x2002 },
	{ CCI_REG16(0x81e6), 0x0000 },
	{ CCI_REG16(0x8220), 0x0001 },
	{ CCI_REG16(0x8222), 0x0100 },
	{ CCI_REG16(0x8224), 0x0100 },
	{ CCI_REG16(0x8226), 0x0100 },
	{ CCI_REG16(0x8228), 0x0100 },
	{ CCI_REG16(0x822a), 0x0100 },
	{ CCI_REG16(0x822c), 0x011c },
	{ CCI_REG16(0x822e), 0x0121 },
	{ CCI_REG16(0x8230), 0x0133 },
	{ CCI_REG16(0x8232), 0x0145 },
	{ CCI_REG16(0x8236), 0x000b },
	{ CCI_REG16(0x8238), 0x000b },
	{ CCI_REG16(0x823c), 0x000b },
	{ CCI_REG16(0x8240), 0x000b },
	{ CCI_REG16(0x8244), 0x000b },
	{ CCI_REG16(0x8248), 0x000b },
	{ CCI_REG16(0xfcfc), 0x4000 },
	{ CCI_REG16(0x031a), 0x0003 },
	{ CCI_REG16(0x031c), 0x002f },
	{ CCI_REG16(0x031e), 0x0001 },
	{ CCI_REG16(0xa102), 0x0b02 },
	{ CCI_REG16(0xfcfc), 0x2000 },
	{ CCI_REG16(0xe0fe), 0x0000 },
	{ CCI_REG16(0xe100), 0x0000 },
	{ CCI_REG16(0xe102), 0x0100 },
	{ CCI_REG16(0xe104), 0x0000 },
	{ CCI_REG16(0xe106), 0x0100 },
	{ CCI_REG16(0xe108), 0x0000 },
	{ CCI_REG16(0xe10a), 0x0000 },
	{ CCI_REG16(0xe10c), 0x0100 },
	{ CCI_REG16(0xe10e), 0x0000 },
	{ CCI_REG16(0xe110), 0x0000 },
	{ CCI_REG16(0xe112), 0x0000 },
	{ CCI_REG16(0xe114), 0x0000 },
	{ CCI_REG16(0xe116), 0x0000 },
	{ CCI_REG16(0xe118), 0x0000 },
	{ CCI_REG16(0xe11a), 0x0200 },
	{ CCI_REG16(0xe11c), 0x0000 },
	{ CCI_REG16(0xe11e), 0x0000 },
	{ CCI_REG16(0xe120), 0x0000 },
	{ CCI_REG16(0xe122), 0x0000 },
	{ CCI_REG16(0xe124), 0x0000 },
	{ CCI_REG16(0xe126), 0x0000 },
	{ CCI_REG16(0xe128), 0x0000 },
	{ CCI_REG16(0xe12a), 0x0200 },
	{ CCI_REG16(0xe12c), 0x0000 },
	{ CCI_REG16(0xfcfc), 0x2002 },
	{ CCI_REG16(0x8138), 0x0410 },
	{ CCI_REG16(0x813a), 0x0800 },
	{ CCI_REG16(0x813c), 0x0c00 },
	{ CCI_REG16(0x813e), 0x1000 },
	{ CCI_REG16(0x8140), 0x1480 },
	{ CCI_REG16(0x8142), 0x1920 },
	{ CCI_REG16(0x8144), 0x2000 },
	{ CCI_REG16(0x8146), 0x25e0 },
	{ CCI_REG16(0x8148), 0x2bc0 },
	{ CCI_REG16(0x818c), 0x2000 },
	{ CCI_REG16(0x818e), 0x1eaa },
	{ CCI_REG16(0x8190), 0x1db0 },
	{ CCI_REG16(0x8192), 0x1cb6 },
	{ CCI_REG16(0x8194), 0x1bbc },
	{ CCI_REG16(0x8196), 0x1ac2 },
	{ CCI_REG16(0x8198), 0x1964 },
	{ CCI_REG16(0x819a), 0x1838 },
	{ CCI_REG16(0x819c), 0x170c },
	{ CCI_REG16(0x819e), 0x2000 },
	{ CCI_REG16(0x81a0), 0x1eaa },
	{ CCI_REG16(0x81a2), 0x1db0 },
	{ CCI_REG16(0x81a4), 0x1cb6 },
	{ CCI_REG16(0x81a6), 0x1bbc },
	{ CCI_REG16(0x81a8), 0x1ac2 },
	{ CCI_REG16(0x81aa), 0x1964 },
	{ CCI_REG16(0x81ac), 0x1838 },
	{ CCI_REG16(0x81ae), 0x170c },
	{ CCI_REG16(0x81b0), 0x2000 },
	{ CCI_REG16(0x81b2), 0x1eaa },
	{ CCI_REG16(0x81b4), 0x1db0 },
	{ CCI_REG16(0x81b6), 0x1cb6 },
	{ CCI_REG16(0x81b8), 0x1bbc },
	{ CCI_REG16(0x81ba), 0x1ac2 },
	{ CCI_REG16(0x81bc), 0x1964 },
	{ CCI_REG16(0x81be), 0x1838 },
	{ CCI_REG16(0x81c0), 0x170c },
	{ CCI_REG16(0x81c2), 0x2000 },
	{ CCI_REG16(0x81c4), 0x1eaa },
	{ CCI_REG16(0x81c6), 0x1db0 },
	{ CCI_REG16(0x81c8), 0x1cb6 },
	{ CCI_REG16(0x81ca), 0x1bbc },
	{ CCI_REG16(0x81cc), 0x1ac2 },
	{ CCI_REG16(0x81ce), 0x1964 },
	{ CCI_REG16(0x81d0), 0x1838 },
	{ CCI_REG16(0x81d2), 0x170c },
	{ CCI_REG16(0xfcfc), 0x2000 },
	{ CCI_REG16(0x0f3a), 0x0001 },
	{ CCI_REG16(0x0f3c), 0x0000 },
	{ CCI_REG16(0x0f3e), 0xba00 },
	{ CCI_REG16(0xfcfc), 0x2001 },
	{ CCI_REG16(0x009c), 0x0002 },
};

/*
 * Vendor bin group 1 = stock res index 1 (CAM_START_DEV实录: Res index
 * switch index 1, 0xff0 x 0xbf4, NORMAL 2PD, 30fps): 4080x3060 RAW10, CPHY
 * 3-phase 3-lane (0x0114 = 0x0301). 775 records, pure direct writes - this
 * sensor family uses 0xfcfc page select and has NO FIFO-window patch data
 * (unlike GD2/3P9 0x6028/0x6f12). VTS 0x16d0 (5840), HTS 0x2ac0 (10944).
 */
static const struct cci_reg_sequence s5kgn5sp_4080x3060_mode[] = {
	{ CCI_REG16(0xfcfc), 0x4000 },
	{ CCI_REG16(0x0344), 0x0020 },
	{ CCI_REG16(0x0346), 0x0040 },
	{ CCI_REG16(0x0348), 0x3fdf },
	{ CCI_REG16(0x034a), 0x187f },
	{ CCI_REG16(0x034c), 0x0ff0 },
	{ CCI_REG16(0x034e), 0x0bf4 },
	{ CCI_REG16(0x0340), 0x16d0 },
	{ CCI_REG16(0x0342), 0x2ac0 },
	{ CCI_REG16(0x0900), 0x2222 },
	{ CCI_REG16(0x0380), 0x0002 },
	{ CCI_REG16(0x0382), 0x0002 },
	{ CCI_REG16(0x0384), 0x0002 },
	{ CCI_REG16(0x0386), 0x0002 },
	{ CCI_REG16(0x0350), 0x0000 },
	{ CCI_REG16(0x0352), 0x0016 },
	{ CCI_REG16(0x0136), 0x1300 },
	{ CCI_REG16(0x013e), 0x00c8 },
	{ CCI_REG16(0x0304), 0x0002 },
	{ CCI_REG16(0x0306), 0x00c8 },
	{ CCI_REG16(0x030e), 0x0003 },
	{ CCI_REG16(0x0310), 0x00c2 },
	{ CCI_REG16(0x0312), 0x0000 },
	{ CCI_REG16(0x0b02), 0x0103 },
	{ CCI_REG16(0x0b08), 0x0001 },
	{ CCI_REG16(0x0b30), 0x01ff },
	{ CCI_REG16(0xfcfc), 0x2000 },
	{ CCI_REG16(0xa9e2), 0x0a03 },
	{ CCI_REG16(0xa9e4), 0x0100 },
	{ CCI_REG16(0x3108), 0xffff },
	{ CCI_REG16(0x323e), 0x0101 },
	{ CCI_REG16(0xeb60), 0x0200 },
	{ CCI_REG16(0xbd10), 0x0100 },
	{ CCI_REG16(0xebd0), 0x0000 },
	{ CCI_REG16(0xebd6), 0x0000 },
	{ CCI_REG16(0xebd8), 0x0000 },
	{ CCI_REG16(0xaad4), 0x0096 },
	{ CCI_REG16(0x3238), 0x0800 },
	{ CCI_REG16(0xad7e), 0x0101 },
	{ CCI_REG16(0xfcfc), 0x4000 },
	{ CCI_REG16(0x0202), 0x0100 },
	{ CCI_REG16(0x022c), 0x0100 },
	{ CCI_REG16(0x0226), 0x0100 },
	{ CCI_REG16(0x021e), 0x0000 },
	{ CCI_REG16(0x0228), 0x0100 },
	{ CCI_REG16(0x0702), 0x0000 },
	{ CCI_REG16(0x0112), 0x0a0a },
	{ CCI_REG16(0x0114), 0x0301 },
	{ CCI_REG16(0x0118), 0x0004 },
	{ CCI_REG16(0x011a), 0x0001 },
	{ CCI_REG16(0x011c), 0x0101 },
	{ CCI_REG16(0x0260), 0x0001 },
	{ CCI_REG16(0x0262), 0x0200 },
	{ CCI_REG16(0x0264), 0x0102 },
	{ CCI_REG16(0x0266), 0x0000 },
	{ CCI_REG16(0x0274), 0x2b2b },
	{ CCI_REG16(0x0720), 0x0001 },
	{ CCI_REG16(0x0724), 0x0000 },
	{ CCI_REG16(0x0728), 0x07f8 },
	{ CCI_REG16(0x072a), 0x02fc },
	{ CCI_REG16(0x072c), 0x0000 },
	{ CCI_REG16(0x072e), 0x0000 },
	{ CCI_REG16(0xfcfc), 0x2002 },
	{ CCI_REG16(0x8234), 0x0000 },
	{ CCI_REG16(0xfcfc), 0x2000 },
	{ CCI_REG16(0xea60), 0x0018 },
	{ CCI_REG16(0xea62), 0x0018 },
	{ CCI_REG16(0xe0e0), 0x0101 },
	{ CCI_REG16(0xe0ea), 0x0ff0 },
	{ CCI_REG16(0xe0ec), 0x0c20 },
	{ CCI_REG16(0xe0ee), 0x0ff0 },
	{ CCI_REG16(0xe0f0), 0x0c20 },
	{ CCI_REG16(0xfcfc), 0x2002 },
	{ CCI_REG16(0x8134), 0x0101 },
	{ CCI_REG16(0x8136), 0x0000 },
	{ CCI_REG16(0x81d6), 0x0020 },
	{ CCI_REG16(0x81d8), 0x0010 },
	{ CCI_REG16(0x81da), 0x002f },
	{ CCI_REG16(0x81dc), 0x001f },
	{ CCI_REG16(0x81de), 0x1940 },
	{ CCI_REG16(0x81e0), 0x1930 },
	{ CCI_REG16(0x81e2), 0x194f },
	{ CCI_REG16(0x81e4), 0x193f },
	{ CCI_REG16(0xfcfc), 0x2000 },
	{ CCI_REG16(0x325c), 0x1930 },
	{ CCI_REG16(0x325e), 0x0010 },
	{ CCI_REG16(0xcb22), 0x0002 },
	{ CCI_REG16(0xb408), 0x0100 },
	{ CCI_REG16(0xb422), 0x0000 },
	{ CCI_REG16(0xb424), 0xab80 },
	{ CCI_REG16(0xb426), 0xab90 },
	{ CCI_REG16(0xda60), 0x0200 },
	{ CCI_REG16(0xdaaa), 0x0100 },
	{ CCI_REG16(0xfcfc), 0x2002 },
	{ CCI_REG16(0x81d4), 0x0000 },
	{ CCI_REG16(0xfcfc), 0x2001 },
	{ CCI_REG16(0x08b6), 0x07ff },
	{ CCI_REG16(0x08bc), 0x7fff },
	{ CCI_REG16(0xfcfc), 0x2000 },
	{ CCI_REG16(0xb928), 0x0022 },
	{ CCI_REG16(0xb976), 0x0000 },
	{ CCI_REG16(0xb98a), 0x0001 },
	{ CCI_REG16(0xb97c), 0x000c },
	{ CCI_REG16(0xb990), 0x002a },
	{ CCI_REG16(0xb99e), 0x0000 },
	{ CCI_REG16(0xb9a0), 0x0000 },
	{ CCI_REG16(0xb9a2), 0x0000 },
	{ CCI_REG16(0xb9a4), 0x0000 },
	{ CCI_REG16(0xb9a6), 0x0000 },
	{ CCI_REG16(0xb9a8), 0x0000 },
	{ CCI_REG16(0xb9aa), 0x0000 },
	{ CCI_REG16(0xcaa0), 0x0200 },
	{ CCI_REG16(0xcaaa), 0x0000 },
	{ CCI_REG16(0xcab4), 0x0100 },
	{ CCI_REG16(0xcab6), 0x0100 },
	{ CCI_REG16(0xcab8), 0x0100 },
	{ CCI_REG16(0xcaba), 0x00ff },
	{ CCI_REG16(0xbd24), 0x0700 },
	{ CCI_REG16(0xbd26), 0x06a4 },
	{ CCI_REG16(0xbd28), 0x06c0 },
	{ CCI_REG16(0xbd2a), 0x0064 },
	{ CCI_REG16(0xbd2c), 0x07ff },
	{ CCI_REG16(0xbd2e), 0x0700 },
	{ CCI_REG16(0x0e78), 0x0078 },
	{ CCI_REG16(0x0e9e), 0x4fc0 },
	{ CCI_REG16(0xc770), 0x049e },
	{ CCI_REG16(0xc7a0), 0x07f0 },
	{ CCI_REG16(0xc7a2), 0x0444 },
	{ CCI_REG16(0x2510), 0x0000 },
	{ CCI_REG16(0x2514), 0x0000 },
	{ CCI_REG16(0x2520), 0x0100 },
	{ CCI_REG16(0x252a), 0x00ff },
	{ CCI_REG16(0x252c), 0x0200 },
	{ CCI_REG16(0x253c), 0x0100 },
	{ CCI_REG16(0x253e), 0x0200 },
	{ CCI_REG16(0x2550), 0x0015 },
	{ CCI_REG16(0x2552), 0x0017 },
	{ CCI_REG16(0x2554), 0x4000 },
	{ CCI_REG16(0x2556), 0xfda0 },
	{ CCI_REG16(0x2558), 0x0015 },
	{ CCI_REG16(0x255a), 0x0017 },
	{ CCI_REG16(0x255c), 0x4000 },
	{ CCI_REG16(0x255e), 0xfda2 },
	{ CCI_REG16(0x2af0), 0x0101 },
	{ CCI_REG16(0x2af2), 0x0100 },
	{ CCI_REG16(0x2afa), 0x007f },
	{ CCI_REG16(0x2afc), 0x007f },
	{ CCI_REG16(0x2afe), 0x00ff },
	{ CCI_REG16(0x2b0c), 0x0080 },
	{ CCI_REG16(0x2b0e), 0x0080 },
	{ CCI_REG16(0x2b10), 0x0100 },
	{ CCI_REG16(0x2b20), 0x8001 },
	{ CCI_REG16(0x2b22), 0x03c1 },
	{ CCI_REG16(0x2b24), 0x4000 },
	{ CCI_REG16(0x2b26), 0xfdb6 },
	{ CCI_REG16(0x2b28), 0x00c2 },
	{ CCI_REG16(0x2b2a), 0x0000 },
	{ CCI_REG16(0x2b2c), 0x4000 },
	{ CCI_REG16(0x2b2e), 0xfdba },
	{ CCI_REG16(0x2bc0), 0x0007 },
	{ CCI_REG16(0x2bc2), 0x0007 },
	{ CCI_REG16(0x2bc4), 0x4000 },
	{ CCI_REG16(0x2bc6), 0xde10 },
	{ CCI_REG16(0x2bc8), 0x0007 },
	{ CCI_REG16(0x2bca), 0x0007 },
	{ CCI_REG16(0x2bcc), 0x4000 },
	{ CCI_REG16(0x2bce), 0xdf10 },
	{ CCI_REG16(0x2c60), 0x34dd },
	{ CCI_REG16(0x2c62), 0x345d },
	{ CCI_REG16(0x2c64), 0x4000 },
	{ CCI_REG16(0x2c66), 0xfd06 },
	{ CCI_REG16(0x3336), 0x000c },
	{ CCI_REG16(0x334a), 0x0013 },
	{ CCI_REG16(0x335e), 0x000c },
	{ CCI_REG16(0x344e), 0x017d },
	{ CCI_REG16(0x3454), 0x0020 },
	{ CCI_REG16(0x3462), 0x000a },
	{ CCI_REG16(0x3468), 0x0048 },
	{ CCI_REG16(0x3476), 0x001a },
	{ CCI_REG16(0x34ee), 0x0001 },
	{ CCI_REG16(0x34f4), 0x000c },
	{ CCI_REG16(0x3502), 0x0025 },
	{ CCI_REG16(0x3516), 0x0000 },
	{ CCI_REG16(0x352a), 0x0000 },
	{ CCI_REG16(0x353e), 0x0000 },
	{ CCI_REG16(0x3552), 0x0000 },
	{ CCI_REG16(0x3566), 0x0000 },
	{ CCI_REG16(0x357a), 0x0000 },
	{ CCI_REG16(0x35f2), 0x0000 },
	{ CCI_REG16(0x3606), 0x0000 },
	{ CCI_REG16(0x362e), 0x0010 },
	{ CCI_REG16(0x3656), 0x0020 },
	{ CCI_REG16(0x366a), 0x0020 },
	{ CCI_REG16(0x36a6), 0x000c },
	{ CCI_REG16(0x36ac), 0x002a },
	{ CCI_REG16(0x36ce), 0x0082 },
	{ CCI_REG16(0x3796), 0x0010 },
	{ CCI_REG16(0x379c), 0x000c },
	{ CCI_REG16(0x37aa), 0x0012 },
	{ CCI_REG16(0x3b7e), 0x0014 },
	{ CCI_REG16(0x3b92), 0x0004 },
	{ CCI_REG16(0x3b98), 0x02b3 },
	{ CCI_REG16(0x3d36), 0x0016 },
	{ CCI_REG16(0x3d3c), 0x0016 },
	{ CCI_REG16(0x3d4a), 0x0034 },
	{ CCI_REG16(0x3d50), 0x0034 },
	{ CCI_REG16(0x3d5e), 0x004c },
	{ CCI_REG16(0x3d64), 0x004c },
	{ CCI_REG16(0x4d9e), 0x0030 },
	{ CCI_REG16(0x4da4), 0x006c },
	{ CCI_REG16(0x4dda), 0x004a },
	{ CCI_REG16(0x4de0), 0x00c8 },
	{ CCI_REG16(0x4e02), 0x0057 },
	{ CCI_REG16(0x4e08), 0x00dc },
	{ CCI_REG16(0x4e2a), 0x00fa },
	{ CCI_REG16(0x4e30), 0x0118 },
	{ CCI_REG16(0x6202), 0x01ab },
	{ CCI_REG16(0x6208), 0x808c },
	{ CCI_REG16(0x62a2), 0x01ab },
	{ CCI_REG16(0x62a8), 0x808c },
	{ CCI_REG16(0xa9c0), 0x0ab0 },
	{ CCI_REG16(0xaa34), 0x0e0e },
	{ CCI_REG16(0xaa48), 0x0701 },
	{ CCI_REG16(0xaa52), 0x0200 },
	{ CCI_REG16(0xaa5c), 0x0204 },
	{ CCI_REG16(0xaa66), 0x0000 },
	{ CCI_REG16(0xaa70), 0x0404 },
	{ CCI_REG16(0xaaa8), 0x0080 },
	{ CCI_REG16(0xaaaa), 0x3bc0 },
	{ CCI_REG16(0xaad8), 0x445f },
	{ CCI_REG16(0xaadc), 0xfc3f },
	{ CCI_REG16(0xaade), 0x4dd9 },
	{ CCI_REG16(0xaae8), 0x0020 },
	{ CCI_REG16(0xaaea), 0x90c0 },
	{ CCI_REG16(0xab1a), 0x0000 },
	{ CCI_REG16(0xab46), 0x0003 },
	{ CCI_REG16(0xab6c), 0x0003 },
	{ CCI_REG16(0xab92), 0x0003 },
	{ CCI_REG16(0xabb8), 0x0003 },
	{ CCI_REG16(0xac4e), 0x0100 },
	{ CCI_REG16(0xac52), 0x0000 },
	{ CCI_REG16(0xac5c), 0x000f },
	{ CCI_REG16(0xad6e), 0x2672 },
	{ CCI_REG16(0xad74), 0x4800 },
	{ CCI_REG16(0xade0), 0x000b },
	{ CCI_REG16(0xade2), 0x000b },
	{ CCI_REG16(0xade4), 0x000b },
	{ CCI_REG16(0xade6), 0x000b },
	{ CCI_REG16(0xade8), 0x000b },
	{ CCI_REG16(0xadea), 0x000b },
	{ CCI_REG16(0xae18), 0x4000 },
	{ CCI_REG16(0xae1a), 0xee0c },
	{ CCI_REG16(0xae1c), 0x4000 },
	{ CCI_REG16(0xae1e), 0xfdd2 },
	{ CCI_REG16(0xae20), 0x0000 },
	{ CCI_REG16(0xae22), 0x0000 },
	{ CCI_REG16(0xae24), 0x0000 },
	{ CCI_REG16(0xae26), 0x0000 },
	{ CCI_REG16(0xae28), 0x0000 },
	{ CCI_REG16(0xae2a), 0x0000 },
	{ CCI_REG16(0xae68), 0x0005 },
	{ CCI_REG16(0xae6a), 0x0005 },
	{ CCI_REG16(0xae6c), 0x0005 },
	{ CCI_REG16(0xae6e), 0x0005 },
	{ CCI_REG16(0xae70), 0x0005 },
	{ CCI_REG16(0xae7c), 0x0005 },
	{ CCI_REG16(0xae7e), 0x0005 },
	{ CCI_REG16(0xae80), 0x0005 },
	{ CCI_REG16(0xae82), 0x0005 },
	{ CCI_REG16(0xae84), 0x0005 },
	{ CCI_REG16(0xae86), 0x0030 },
	{ CCI_REG16(0xae88), 0x0030 },
	{ CCI_REG16(0xae8a), 0x0030 },
	{ CCI_REG16(0xae8c), 0x0030 },
	{ CCI_REG16(0xae8e), 0x0030 },
	{ CCI_REG16(0xae9a), 0x0045 },
	{ CCI_REG16(0xae9c), 0x0050 },
	{ CCI_REG16(0xae9e), 0x0045 },
	{ CCI_REG16(0xaea0), 0x0050 },
	{ CCI_REG16(0xaea2), 0x0030 },
	{ CCI_REG16(0xaea4), 0x0000 },
	{ CCI_REG16(0xaea6), 0x0000 },
	{ CCI_REG16(0xaea8), 0x0000 },
	{ CCI_REG16(0xaeaa), 0x0000 },
	{ CCI_REG16(0xaeac), 0x0000 },
	{ CCI_REG16(0xaeb8), 0x0000 },
	{ CCI_REG16(0xaeba), 0x0000 },
	{ CCI_REG16(0xaebc), 0x0000 },
	{ CCI_REG16(0xaebe), 0x0000 },
	{ CCI_REG16(0xaec0), 0x0000 },
	{ CCI_REG16(0xaec2), 0x0000 },
	{ CCI_REG16(0xaec4), 0x0000 },
	{ CCI_REG16(0xaec6), 0x0000 },
	{ CCI_REG16(0xaec8), 0x0000 },
	{ CCI_REG16(0xaeca), 0x0000 },
	{ CCI_REG16(0xaed6), 0x0000 },
	{ CCI_REG16(0xaed8), 0x0000 },
	{ CCI_REG16(0xaeda), 0x0000 },
	{ CCI_REG16(0xaedc), 0x0000 },
	{ CCI_REG16(0xaede), 0x0000 },
	{ CCI_REG16(0xaee0), 0x0000 },
	{ CCI_REG16(0xaee2), 0x0000 },
	{ CCI_REG16(0xaee4), 0x0000 },
	{ CCI_REG16(0xaee6), 0x0000 },
	{ CCI_REG16(0xaee8), 0x0000 },
	{ CCI_REG16(0xaef4), 0x0000 },
	{ CCI_REG16(0xaef6), 0x0000 },
	{ CCI_REG16(0xaef8), 0x0000 },
	{ CCI_REG16(0xaefa), 0x0000 },
	{ CCI_REG16(0xaefc), 0x0000 },
	{ CCI_REG16(0xcf6e), 0x0002 },
	{ CCI_REG16(0xcf70), 0x0100 },
	{ CCI_REG16(0xcf72), 0x03ff },
	{ CCI_REG16(0xcf74), 0x1000 },
	{ CCI_REG16(0xcf76), 0x1000 },
	{ CCI_REG16(0xcf78), 0x0400 },
	{ CCI_REG16(0xcf7a), 0x4000 },
	{ CCI_REG16(0xcf7c), 0x1000 },
	{ CCI_REG16(0xcf7e), 0x0fe0 },
	{ CCI_REG16(0xcf80), 0x0000 },
	{ CCI_REG16(0xcf82), 0x0000 },
	{ CCI_REG16(0xcf84), 0x0000 },
	{ CCI_REG16(0xcf86), 0x0000 },
	{ CCI_REG16(0xcf98), 0x0100 },
	{ CCI_REG16(0xcf9a), 0x03ff },
	{ CCI_REG16(0xcf9c), 0x1000 },
	{ CCI_REG16(0xcf9e), 0x1000 },
	{ CCI_REG16(0xcfa0), 0x0400 },
	{ CCI_REG16(0xcfa2), 0x4000 },
	{ CCI_REG16(0xcfa4), 0x1000 },
	{ CCI_REG16(0xcfa6), 0x0ff0 },
	{ CCI_REG16(0xcfa8), 0x0000 },
	{ CCI_REG16(0xcfaa), 0x0000 },
	{ CCI_REG16(0xcfac), 0x0000 },
	{ CCI_REG16(0xcfae), 0x0000 },
	{ CCI_REG16(0xcfc0), 0x0100 },
	{ CCI_REG16(0xcfc2), 0x03ff },
	{ CCI_REG16(0xcfc4), 0x1000 },
	{ CCI_REG16(0xcfc6), 0x1000 },
	{ CCI_REG16(0xcfc8), 0x0400 },
	{ CCI_REG16(0xcfca), 0x4000 },
	{ CCI_REG16(0xcfcc), 0x1000 },
	{ CCI_REG16(0xcfce), 0x0ff0 },
	{ CCI_REG16(0xcfd0), 0x0000 },
	{ CCI_REG16(0xcfd2), 0x0000 },
	{ CCI_REG16(0xcfd4), 0x0000 },
	{ CCI_REG16(0xcfd6), 0x0000 },
	{ CCI_REG16(0xcfe8), 0x0100 },
	{ CCI_REG16(0xcfea), 0x03ff },
	{ CCI_REG16(0xcfec), 0x1000 },
	{ CCI_REG16(0xcfee), 0x1000 },
	{ CCI_REG16(0xcff0), 0x0400 },
	{ CCI_REG16(0xcff2), 0x4000 },
	{ CCI_REG16(0xcff4), 0x1000 },
	{ CCI_REG16(0xcff6), 0x0fe0 },
	{ CCI_REG16(0xcff8), 0x0000 },
	{ CCI_REG16(0xcffa), 0x0000 },
	{ CCI_REG16(0xcffc), 0x0000 },
	{ CCI_REG16(0xcffe), 0x0000 },
	{ CCI_REG16(0xd15a), 0x0000 },
	{ CCI_REG16(0xd15e), 0x0000 },
	{ CCI_REG16(0xd16a), 0x0000 },
	{ CCI_REG16(0xd16e), 0x0000 },
	{ CCI_REG16(0xd1da), 0x0005 },
	{ CCI_REG16(0xd1dc), 0x0000 },
	{ CCI_REG16(0xd1de), 0x0002 },
	{ CCI_REG16(0xd1e0), 0x0000 },
	{ CCI_REG16(0xd1e2), 0x0000 },
	{ CCI_REG16(0xd1e4), 0x0000 },
	{ CCI_REG16(0xd1e6), 0x0000 },
	{ CCI_REG16(0xd1e8), 0x0000 },
	{ CCI_REG16(0xd1ea), 0x0000 },
	{ CCI_REG16(0xd1ec), 0x0000 },
	{ CCI_REG16(0xd1ee), 0x0007 },
	{ CCI_REG16(0xd1f0), 0x0000 },
	{ CCI_REG16(0xd1f2), 0x0000 },
	{ CCI_REG16(0xd1f4), 0x0000 },
	{ CCI_REG16(0xd1f6), 0x0000 },
	{ CCI_REG16(0xd1f8), 0x0000 },
	{ CCI_REG16(0xd25a), 0x0100 },
	{ CCI_REG16(0xd2dc), 0x0025 },
	{ CCI_REG16(0xd2e0), 0x0025 },
	{ CCI_REG16(0xd2ec), 0x0025 },
	{ CCI_REG16(0xd2f0), 0x0025 },
	{ CCI_REG16(0x311a), 0x0001 },
	{ CCI_REG16(0x311c), 0x0005 },
	{ CCI_REG16(0xfcfc), 0x4000 },
	{ CCI_REG16(0xee26), 0x05dc },
	{ CCI_REG16(0xfdb6), 0x8001 },
	{ CCI_REG16(0xfdba), 0x00c2 },
	{ CCI_REG16(0xfdd2), 0x0020 },
	{ CCI_REG16(0xfcfc), 0x2002 },
	{ CCI_REG16(0x823a), 0x0008 },
	{ CCI_REG16(0x823e), 0x0008 },
	{ CCI_REG16(0x8242), 0x0008 },
	{ CCI_REG16(0x8246), 0x0008 },
	{ CCI_REG16(0x8000), 0x0100 },
	{ CCI_REG16(0x8002), 0x0000 },
	{ CCI_REG16(0x8004), 0x0000 },
	{ CCI_REG16(0x8006), 0x0000 },
	{ CCI_REG16(0x8008), 0x0000 },
	{ CCI_REG16(0x800a), 0x0000 },
	{ CCI_REG16(0x800c), 0x0000 },
	{ CCI_REG16(0x800e), 0x0000 },
	{ CCI_REG16(0x8010), 0x0000 },
	{ CCI_REG16(0x8012), 0x0000 },
	{ CCI_REG16(0x8014), 0x0000 },
	{ CCI_REG16(0x8016), 0x0000 },
	{ CCI_REG16(0x8018), 0x0000 },
	{ CCI_REG16(0x801a), 0x0000 },
	{ CCI_REG16(0x801c), 0x0000 },
	{ CCI_REG16(0x801e), 0x0000 },
	{ CCI_REG16(0x8020), 0x0000 },
	{ CCI_REG16(0x8022), 0xffed },
	{ CCI_REG16(0x8024), 0x0000 },
	{ CCI_REG16(0x8026), 0xffea },
	{ CCI_REG16(0x8028), 0x0000 },
	{ CCI_REG16(0x802a), 0x0000 },
	{ CCI_REG16(0x802c), 0x0000 },
	{ CCI_REG16(0x802e), 0x0000 },
	{ CCI_REG16(0x8030), 0x0000 },
	{ CCI_REG16(0x8032), 0xffe8 },
	{ CCI_REG16(0x8034), 0x0000 },
	{ CCI_REG16(0x8036), 0xffef },
	{ CCI_REG16(0x8038), 0x0000 },
	{ CCI_REG16(0x803a), 0x0000 },
	{ CCI_REG16(0x803c), 0x0000 },
	{ CCI_REG16(0x803e), 0x0000 },
	{ CCI_REG16(0x8040), 0x0000 },
	{ CCI_REG16(0x8042), 0x0000 },
	{ CCI_REG16(0x8044), 0x0000 },
	{ CCI_REG16(0x8046), 0x0000 },
	{ CCI_REG16(0x8048), 0x0000 },
	{ CCI_REG16(0x804a), 0x0100 },
	{ CCI_REG16(0x804c), 0x03ff },
	{ CCI_REG16(0x804e), 0x1000 },
	{ CCI_REG16(0x8050), 0x1000 },
	{ CCI_REG16(0x8052), 0x0400 },
	{ CCI_REG16(0x8054), 0x4000 },
	{ CCI_REG16(0x8056), 0x1010 },
	{ CCI_REG16(0x8058), 0x0ff5 },
	{ CCI_REG16(0x805a), 0x0000 },
	{ CCI_REG16(0x805c), 0x0000 },
	{ CCI_REG16(0x805e), 0x0000 },
	{ CCI_REG16(0x8060), 0x0000 },
	{ CCI_REG16(0x8062), 0x0100 },
	{ CCI_REG16(0x8064), 0x03ff },
	{ CCI_REG16(0x8066), 0x1000 },
	{ CCI_REG16(0x8068), 0x0ff0 },
	{ CCI_REG16(0x806a), 0x0400 },
	{ CCI_REG16(0x806c), 0x4000 },
	{ CCI_REG16(0x806e), 0x1010 },
	{ CCI_REG16(0x8070), 0x0ff5 },
	{ CCI_REG16(0x8072), 0x0000 },
	{ CCI_REG16(0x8074), 0x0000 },
	{ CCI_REG16(0x8076), 0x0000 },
	{ CCI_REG16(0x8078), 0x0000 },
	{ CCI_REG16(0x807a), 0x0100 },
	{ CCI_REG16(0x807c), 0x03ff },
	{ CCI_REG16(0x807e), 0x1000 },
	{ CCI_REG16(0x8080), 0x0ff0 },
	{ CCI_REG16(0x8082), 0x0400 },
	{ CCI_REG16(0x8084), 0x4000 },
	{ CCI_REG16(0x8086), 0x1010 },
	{ CCI_REG16(0x8088), 0x0ff5 },
	{ CCI_REG16(0x808a), 0x0000 },
	{ CCI_REG16(0x808c), 0x0000 },
	{ CCI_REG16(0x808e), 0x0000 },
	{ CCI_REG16(0x8090), 0x0000 },
	{ CCI_REG16(0x8092), 0x0100 },
	{ CCI_REG16(0x8094), 0x03ff },
	{ CCI_REG16(0x8096), 0x1000 },
	{ CCI_REG16(0x8098), 0x1000 },
	{ CCI_REG16(0x809a), 0x0400 },
	{ CCI_REG16(0x809c), 0x4000 },
	{ CCI_REG16(0x809e), 0x1010 },
	{ CCI_REG16(0x80a0), 0x0ff5 },
	{ CCI_REG16(0x80a2), 0x0000 },
	{ CCI_REG16(0x80a4), 0x0000 },
	{ CCI_REG16(0x80a6), 0x0000 },
	{ CCI_REG16(0x80a8), 0x0000 },
	{ CCI_REG16(0x80aa), 0x0000 },
	{ CCI_REG16(0x80ac), 0x0000 },
	{ CCI_REG16(0x80ae), 0x0000 },
	{ CCI_REG16(0x80b0), 0x0000 },
	{ CCI_REG16(0x80b2), 0x0000 },
	{ CCI_REG16(0x80b4), 0x0000 },
	{ CCI_REG16(0x80b6), 0x0000 },
	{ CCI_REG16(0x80b8), 0x0000 },
	{ CCI_REG16(0x80ba), 0x0000 },
	{ CCI_REG16(0x80bc), 0x0000 },
	{ CCI_REG16(0x80be), 0x0000 },
	{ CCI_REG16(0x80c0), 0x0000 },
	{ CCI_REG16(0x80c2), 0x0000 },
	{ CCI_REG16(0x80c4), 0x0000 },
	{ CCI_REG16(0x80c6), 0x0000 },
	{ CCI_REG16(0x80c8), 0x0000 },
	{ CCI_REG16(0x80ca), 0x0000 },
	{ CCI_REG16(0x80cc), 0x0000 },
	{ CCI_REG16(0x80ce), 0x0000 },
	{ CCI_REG16(0x80d0), 0x0000 },
	{ CCI_REG16(0x80d2), 0x0000 },
	{ CCI_REG16(0x80d4), 0x0000 },
	{ CCI_REG16(0x80d6), 0x0000 },
	{ CCI_REG16(0x80d8), 0x0000 },
	{ CCI_REG16(0x80da), 0x0000 },
	{ CCI_REG16(0x80dc), 0x0000 },
	{ CCI_REG16(0x80de), 0x0000 },
	{ CCI_REG16(0x80e0), 0x0000 },
	{ CCI_REG16(0x80e2), 0x0000 },
	{ CCI_REG16(0x80e4), 0x0000 },
	{ CCI_REG16(0x80e6), 0x0000 },
	{ CCI_REG16(0x80e8), 0x0000 },
	{ CCI_REG16(0x80ea), 0x0000 },
	{ CCI_REG16(0x80ec), 0x0000 },
	{ CCI_REG16(0x80ee), 0x0000 },
	{ CCI_REG16(0x80f0), 0x0000 },
	{ CCI_REG16(0x80f2), 0x0000 },
	{ CCI_REG16(0x80f4), 0x0000 },
	{ CCI_REG16(0x80f6), 0x0000 },
	{ CCI_REG16(0x80f8), 0x0000 },
	{ CCI_REG16(0x80fa), 0x0000 },
	{ CCI_REG16(0x80fc), 0x0000 },
	{ CCI_REG16(0x80fe), 0x0000 },
	{ CCI_REG16(0x8100), 0x0000 },
	{ CCI_REG16(0x8102), 0x0000 },
	{ CCI_REG16(0x8104), 0x0000 },
	{ CCI_REG16(0x8106), 0x0000 },
	{ CCI_REG16(0x8108), 0x0000 },
	{ CCI_REG16(0xfcfc), 0x2000 },
	{ CCI_REG16(0xae08), 0x0100 },
	{ CCI_REG16(0x25f0), 0x0000 },
	{ CCI_REG16(0x25f2), 0x0000 },
	{ CCI_REG16(0x25f4), 0x0000 },
	{ CCI_REG16(0x25f6), 0x0000 },
	{ CCI_REG16(0x25f8), 0x0000 },
	{ CCI_REG16(0x25fa), 0x0000 },
	{ CCI_REG16(0x25fc), 0x0000 },
	{ CCI_REG16(0x25fe), 0x0000 },
	{ CCI_REG16(0x2600), 0x0000 },
	{ CCI_REG16(0x2602), 0x0000 },
	{ CCI_REG16(0x2604), 0x0000 },
	{ CCI_REG16(0x2606), 0x0000 },
	{ CCI_REG16(0x2608), 0x0000 },
	{ CCI_REG16(0x260a), 0x0000 },
	{ CCI_REG16(0x260c), 0x0000 },
	{ CCI_REG16(0x260e), 0x0000 },
	{ CCI_REG16(0x2610), 0x0000 },
	{ CCI_REG16(0x2612), 0x0000 },
	{ CCI_REG16(0x2614), 0x0000 },
	{ CCI_REG16(0x2616), 0x0000 },
	{ CCI_REG16(0x2618), 0x0000 },
	{ CCI_REG16(0x261a), 0x0000 },
	{ CCI_REG16(0x261c), 0x0000 },
	{ CCI_REG16(0x261e), 0x0000 },
	{ CCI_REG16(0x2620), 0x0000 },
	{ CCI_REG16(0x2622), 0x0000 },
	{ CCI_REG16(0x2624), 0x0000 },
	{ CCI_REG16(0x2626), 0x0000 },
	{ CCI_REG16(0x2628), 0x0000 },
	{ CCI_REG16(0x262a), 0x0000 },
	{ CCI_REG16(0x262c), 0x0000 },
	{ CCI_REG16(0x262e), 0x0000 },
	{ CCI_REG16(0x2630), 0x0000 },
	{ CCI_REG16(0x2632), 0x0000 },
	{ CCI_REG16(0x2634), 0x0000 },
	{ CCI_REG16(0x2636), 0x0000 },
	{ CCI_REG16(0x30c8), 0x0001 },
	{ CCI_REG16(0x30ca), 0x0000 },
	{ CCI_REG16(0x30cc), 0x0001 },
	{ CCI_REG16(0x30ce), 0x0000 },
	{ CCI_REG16(0x323c), 0x2020 },
	{ CCI_REG16(0xae40), 0x0000 },
	{ CCI_REG16(0xae42), 0x0000 },
	{ CCI_REG16(0xae44), 0x0000 },
	{ CCI_REG16(0xae46), 0x0000 },
	{ CCI_REG16(0xae48), 0x0000 },
	{ CCI_REG16(0xae4a), 0x0000 },
	{ CCI_REG16(0xae4c), 0x0000 },
	{ CCI_REG16(0xae4e), 0x0000 },
	{ CCI_REG16(0xaf94), 0x0000 },
	{ CCI_REG16(0xaf96), 0x0000 },
	{ CCI_REG16(0xafa8), 0x0000 },
	{ CCI_REG16(0xafaa), 0x0000 },
	{ CCI_REG16(0xafac), 0x0000 },
	{ CCI_REG16(0xafae), 0x0000 },
	{ CCI_REG16(0xafb0), 0x0000 },
	{ CCI_REG16(0xafb2), 0x0000 },
	{ CCI_REG16(0xafb4), 0x0000 },
	{ CCI_REG16(0xafc6), 0x0000 },
	{ CCI_REG16(0xafc8), 0x0000 },
	{ CCI_REG16(0xafca), 0x0000 },
	{ CCI_REG16(0xafcc), 0x0000 },
	{ CCI_REG16(0xafce), 0x0000 },
	{ CCI_REG16(0xafd0), 0x0000 },
	{ CCI_REG16(0xafd2), 0x0000 },
	{ CCI_REG16(0xafe4), 0x0000 },
	{ CCI_REG16(0xafe6), 0x0000 },
	{ CCI_REG16(0xafe8), 0x0000 },
	{ CCI_REG16(0xafea), 0x0000 },
	{ CCI_REG16(0xafec), 0x0000 },
	{ CCI_REG16(0xafee), 0x0000 },
	{ CCI_REG16(0xaff0), 0x0000 },
	{ CCI_REG16(0xb002), 0x0000 },
	{ CCI_REG16(0xb004), 0x0000 },
	{ CCI_REG16(0xb006), 0x0000 },
	{ CCI_REG16(0xb008), 0x0000 },
	{ CCI_REG16(0xb00a), 0x0000 },
	{ CCI_REG16(0xbbe8), 0x0003 },
	{ CCI_REG16(0xbc04), 0x0002 },
	{ CCI_REG16(0xbc06), 0x0102 },
	{ CCI_REG16(0xd010), 0x0000 },
	{ CCI_REG16(0xd012), 0x0000 },
	{ CCI_REG16(0xd014), 0x0000 },
	{ CCI_REG16(0xd016), 0x0000 },
	{ CCI_REG16(0xd018), 0x0000 },
	{ CCI_REG16(0xd01a), 0x0000 },
	{ CCI_REG16(0xd01c), 0x0000 },
	{ CCI_REG16(0xd01e), 0x0000 },
	{ CCI_REG16(0xd020), 0x0000 },
	{ CCI_REG16(0xd022), 0x0000 },
	{ CCI_REG16(0xd024), 0x0000 },
	{ CCI_REG16(0xd026), 0x0000 },
	{ CCI_REG16(0xd038), 0x0000 },
	{ CCI_REG16(0xd03a), 0x0000 },
	{ CCI_REG16(0xd03c), 0x0000 },
	{ CCI_REG16(0xd03e), 0x0000 },
	{ CCI_REG16(0xd040), 0x0000 },
	{ CCI_REG16(0xd042), 0x0000 },
	{ CCI_REG16(0xd044), 0x0000 },
	{ CCI_REG16(0xd046), 0x0000 },
	{ CCI_REG16(0xd048), 0x0000 },
	{ CCI_REG16(0xd04a), 0x0000 },
	{ CCI_REG16(0xd04c), 0x0000 },
	{ CCI_REG16(0xd04e), 0x0000 },
	{ CCI_REG16(0xd060), 0x0000 },
	{ CCI_REG16(0xd062), 0x0000 },
	{ CCI_REG16(0xd064), 0x0000 },
	{ CCI_REG16(0xd066), 0x0000 },
	{ CCI_REG16(0xd068), 0x0000 },
	{ CCI_REG16(0xd06a), 0x0000 },
	{ CCI_REG16(0xd06c), 0x0000 },
	{ CCI_REG16(0xd06e), 0x0000 },
	{ CCI_REG16(0xd070), 0x0000 },
	{ CCI_REG16(0xd072), 0x0000 },
	{ CCI_REG16(0xd074), 0x0000 },
	{ CCI_REG16(0xd076), 0x0000 },
	{ CCI_REG16(0xd088), 0x0000 },
	{ CCI_REG16(0xd08a), 0x0000 },
	{ CCI_REG16(0xd08c), 0x0000 },
	{ CCI_REG16(0xd08e), 0x0000 },
	{ CCI_REG16(0xd090), 0x0000 },
	{ CCI_REG16(0xd092), 0x0000 },
	{ CCI_REG16(0xd094), 0x0000 },
	{ CCI_REG16(0xd096), 0x0000 },
	{ CCI_REG16(0xd098), 0x0000 },
	{ CCI_REG16(0xd09a), 0x0000 },
	{ CCI_REG16(0xd09c), 0x0000 },
	{ CCI_REG16(0xd09e), 0x0000 },
	{ CCI_REG16(0xd164), 0x0000 },
	{ CCI_REG16(0xd168), 0x0000 },
	{ CCI_REG16(0xd174), 0x0000 },
	{ CCI_REG16(0xd178), 0x0000 },
	{ CCI_REG16(0xd2e6), 0x0000 },
	{ CCI_REG16(0xd2ea), 0x0000 },
	{ CCI_REG16(0xd2f6), 0x0000 },
	{ CCI_REG16(0xd2fa), 0x0000 },
	{ CCI_REG16(0xfcfc), 0x2001 },
	{ CCI_REG16(0x08f2), 0x1f98 },
	{ CCI_REG16(0x08f4), 0x1fa0 },
	{ CCI_REG16(0x08f6), 0x0006 },
	{ CCI_REG16(0x08fa), 0x0008 },
	{ CCI_REG16(0x08fe), 0x07ff },
	{ CCI_REG16(0x0900), 0x0000 },
	{ CCI_REG16(0x0904), 0x7fff },
	{ CCI_REG16(0x0908), 0x0080 },
	{ CCI_REG16(0x090a), 0x0000 },
	{ CCI_REG16(0x090c), 0xffff },
	{ CCI_REG16(0x09e4), 0x0000 },
	{ CCI_REG16(0x09e6), 0x0000 },
	{ CCI_REG16(0x09e8), 0x0000 },
	{ CCI_REG16(0x0a8a), 0x0000 },
	{ CCI_REG16(0x0a8e), 0x0000 },
	{ CCI_REG16(0x0a92), 0x0000 },
	{ CCI_REG16(0x0a96), 0x0000 },
	{ CCI_REG16(0x0b6a), 0x0101 },
	{ CCI_REG16(0x0b6c), 0x4000 },
	{ CCI_REG16(0x0b6e), 0x0100 },
	{ CCI_REG16(0x0b70), 0x4000 },
	{ CCI_REG16(0x0b74), 0x0100 },
	{ CCI_REG16(0x0b78), 0x0100 },
	{ CCI_REG16(0x0b7c), 0x0000 },
	{ CCI_REG16(0x0b80), 0x0000 },
	{ CCI_REG16(0x0b84), 0x0000 },
	{ CCI_REG16(0x1eb0), 0x0000 },
	{ CCI_REG16(0x1f38), 0xffff },
	{ CCI_REG16(0x1f3a), 0xffff },
	{ CCI_REG16(0x1f44), 0x0000 },
	{ CCI_REG16(0xfcfc), 0x2002 },
	{ CCI_REG16(0x824a), 0x0000 },
	{ CCI_REG16(0x824c), 0x0000 },
	{ CCI_REG16(0x8250), 0x0000 },
	{ CCI_REG16(0x8254), 0x0000 },
	{ CCI_REG16(0x8258), 0x0000 },
	{ CCI_REG16(0x824e), 0x0000 },
	{ CCI_REG16(0x8252), 0x0000 },
	{ CCI_REG16(0x8256), 0x0000 },
	{ CCI_REG16(0x825a), 0x0000 },
	{ CCI_REG16(0x825c), 0x0000 },
	{ CCI_REG16(0x8260), 0x0000 },
	{ CCI_REG16(0x8264), 0x0000 },
	{ CCI_REG16(0x8268), 0x0000 },
	{ CCI_REG16(0x825e), 0x0000 },
	{ CCI_REG16(0x8262), 0x0000 },
	{ CCI_REG16(0x8266), 0x0000 },
	{ CCI_REG16(0x826a), 0x0000 },
	{ CCI_REG16(0x828c), 0x0000 },
	{ CCI_REG16(0x8290), 0x0000 },
	{ CCI_REG16(0x8294), 0x0000 },
	{ CCI_REG16(0x8298), 0x0000 },
	{ CCI_REG16(0x828e), 0x0000 },
	{ CCI_REG16(0x8292), 0x0000 },
	{ CCI_REG16(0x8296), 0x0000 },
	{ CCI_REG16(0x829a), 0x0000 },
	{ CCI_REG16(0x829c), 0x0000 },
	{ CCI_REG16(0x82a0), 0x0000 },
	{ CCI_REG16(0x82a4), 0x0000 },
	{ CCI_REG16(0x82a8), 0x0000 },
	{ CCI_REG16(0x829e), 0x0000 },
	{ CCI_REG16(0x82a2), 0x0000 },
	{ CCI_REG16(0x82a6), 0x0000 },
	{ CCI_REG16(0x82aa), 0x0000 },
	{ CCI_REG16(0x826c), 0x0000 },
	{ CCI_REG16(0x8270), 0x0000 },
	{ CCI_REG16(0x8274), 0x0000 },
	{ CCI_REG16(0x8278), 0x0000 },
	{ CCI_REG16(0x826e), 0x0000 },
	{ CCI_REG16(0x8272), 0x0000 },
	{ CCI_REG16(0x8276), 0x0000 },
	{ CCI_REG16(0x827a), 0x0000 },
	{ CCI_REG16(0x827c), 0x0000 },
	{ CCI_REG16(0x8280), 0x0000 },
	{ CCI_REG16(0x8284), 0x0000 },
	{ CCI_REG16(0x8288), 0x0000 },
	{ CCI_REG16(0x827e), 0x0000 },
	{ CCI_REG16(0x8282), 0x0000 },
	{ CCI_REG16(0x8286), 0x0000 },
	{ CCI_REG16(0x828a), 0x0000 },
	{ CCI_REG16(0x82ac), 0x0000 },
	{ CCI_REG16(0x82b0), 0x0000 },
	{ CCI_REG16(0x82b4), 0x0000 },
	{ CCI_REG16(0x82b8), 0x0000 },
	{ CCI_REG16(0x82ae), 0x0000 },
	{ CCI_REG16(0x82b2), 0x0000 },
	{ CCI_REG16(0x82b6), 0x0000 },
	{ CCI_REG16(0x82ba), 0x0000 },
	{ CCI_REG16(0x82bc), 0x0000 },
	{ CCI_REG16(0x82c0), 0x0000 },
	{ CCI_REG16(0x82c4), 0x0000 },
	{ CCI_REG16(0x82c8), 0x0000 },
	{ CCI_REG16(0x82be), 0x0000 },
	{ CCI_REG16(0x82c2), 0x0000 },
	{ CCI_REG16(0x82c6), 0x0000 },
	{ CCI_REG16(0x82ca), 0x0000 },
	{ CCI_REG16(0x82cc), 0x0000 },
	{ CCI_REG16(0xfcfc), 0x4000 },
	{ CCI_REG16(0xfe8a), 0x0002 },
	{ CCI_REG16(0xfe8c), 0x0924 },
	{ CCI_REG16(0xfd5a), 0xa48b },
	{ CCI_REG16(0xfd5c), 0xa4ff },
	{ CCI_REG16(0xfd72), 0x0007 },
	{ CCI_REG16(0xfd74), 0x0007 },
	{ CCI_REG16(0xfd9e), 0x3f3f },
	{ CCI_REG16(0xfdac), 0x0240 },
	{ CCI_REG16(0xfdae), 0x0240 },
};

static const struct s5kgn5sp_mode s5kgn5sp_supported_modes[] = {
	{
		.width = 4080,
		.height = 3060,
		.hts = 10944,
		.vts = 5840,
		.link_freq_idx = S5KGN5SP_LF_IDX_FULL,
		.reg_list = {
			.regs = s5kgn5sp_4080x3060_mode,
			.num_regs = ARRAY_SIZE(s5kgn5sp_4080x3060_mode),
		},
	},
};

struct s5kgn5sp {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator *vdda;		/* analog 2.8V  = pm8008i L7 */
	struct regulator *vddd;		/* digital 1.056V = pm8008i L2 */
	struct regulator *vddio;	/* I/O 1.8V  = pm8008i L5 */

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *analog_gain;

	const struct s5kgn5sp_mode *mode;
};

static inline u64 s5kgn5sp_freq_to_pixel_rate(const u64 freq);

static int s5kgn5sp_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5kgn5sp *s5kgn5sp = container_of(ctrl->handler, struct s5kgn5sp,
						 ctrl_handler);
	const struct s5kgn5sp_mode *mode = s5kgn5sp->mode;
	s64 exposure_max;
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		exposure_max = mode->height + ctrl->val -
			       S5KGN5SP_EXPOSURE_MARGIN;
		__v4l2_ctrl_modify_range(s5kgn5sp->exposure,
					 s5kgn5sp->exposure->minimum,
					 exposure_max,
					 s5kgn5sp->exposure->step,
					 s5kgn5sp->exposure->default_value);
		break;
	case V4L2_CID_LINK_FREQ: {
		u64 pr = s5kgn5sp_freq_to_pixel_rate(
				s5kgn5sp_link_freq_menu[ctrl->val]);
		__v4l2_ctrl_modify_range(s5kgn5sp->pixel_rate, 0, pr, 1, pr);
		__v4l2_ctrl_s_ctrl_int64(s5kgn5sp->pixel_rate, pr);
		break;
	}
	}

	if (!pm_runtime_get_if_active(s5kgn5sp->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = cci_write(s5kgn5sp->regmap, S5KGN5SP_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(s5kgn5sp->regmap, S5KGN5SP_REG_ANALOG_GAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(s5kgn5sp->regmap, S5KGN5SP_REG_VTS,
				ctrl->val + mode->height, NULL);
		break;
	case V4L2_CID_HBLANK:
		ret = cci_write(s5kgn5sp->regmap, S5KGN5SP_REG_HTS,
				ctrl->val + mode->width, NULL);
		break;
	default:
		/* Not a hardware register ctrl (ranges already updated in the
		 * first switch): __v4l2_ctrl_handler_setup() walks ALL ctrls
		 * during streamon — returning -EINVAL here poisons the whole
		 * setup, trips the enable_streams error path (put_autosuspend)
		 * and autosuspend powers the sensor off ~1s into the stream.
		 * Found 2026-11-20: "failed to start streaming: -22" on every
		 * attempt, power_off 1.29s after power_on mid-stream. */
		ret = 0;
		break;
	}

	pm_runtime_put(s5kgn5sp->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5kgn5sp_ctrl_ops = {
	.s_ctrl = s5kgn5sp_set_ctrl,
};

static inline u64 s5kgn5sp_freq_to_pixel_rate(const u64 freq)
{
	return div_u64(freq * 2 * S5KGN5SP_DATA_LANES, 10);
}

static int s5kgn5sp_init_controls(struct s5kgn5sp *s5kgn5sp)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &s5kgn5sp->ctrl_handler;
	const struct s5kgn5sp_mode *mode = s5kgn5sp->mode;
	s64 pixel_rate, hblank, vblank, exposure_max;
	struct v4l2_ctrl *ctrl;
	u32 orientation = V4L2_CAMERA_ORIENTATION_BACK, rotation = 0;

	v4l2_ctrl_handler_init(ctrl_hdlr, 9);

	s5kgn5sp->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr,
					&s5kgn5sp_ctrl_ops, V4L2_CID_LINK_FREQ,
					ARRAY_SIZE(s5kgn5sp_link_freq_menu) - 1,
					mode->link_freq_idx,
					s5kgn5sp_link_freq_menu);

	pixel_rate = s5kgn5sp_freq_to_pixel_rate(
			s5kgn5sp_link_freq_menu[mode->link_freq_idx]);
	s5kgn5sp->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &s5kgn5sp_ctrl_ops,
						 V4L2_CID_PIXEL_RATE,
						 0, pixel_rate, 1, pixel_rate);
	if (s5kgn5sp->pixel_rate)
		s5kgn5sp->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* HBLANK writable (libcamera simple-pipeline hard requirement, found
	 * via the 2026-11-21 plasma-camera probe). */
	hblank = mode->hts - mode->width;
	s5kgn5sp->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5kgn5sp_ctrl_ops,
					     V4L2_CID_HBLANK, hblank,
					     S5KGN5SP_VTS_MAX - mode->width,
					     1, hblank);

	vblank = mode->vts - mode->height;
	s5kgn5sp->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5kgn5sp_ctrl_ops,
					     V4L2_CID_VBLANK, vblank,
					     S5KGN5SP_VTS_MAX - mode->height,
					     1, vblank);

	exposure_max = mode->vts - S5KGN5SP_EXPOSURE_MARGIN;
	s5kgn5sp->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &s5kgn5sp_ctrl_ops,
					       V4L2_CID_EXPOSURE,
					       S5KGN5SP_EXPOSURE_MIN,
					       exposure_max,
					       S5KGN5SP_EXPOSURE_STEP,
					       mode->vts / 2);

	s5kgn5sp->analog_gain = v4l2_ctrl_new_std(ctrl_hdlr, &s5kgn5sp_ctrl_ops,
						  V4L2_CID_ANALOGUE_GAIN,
						  S5KGN5SP_ANALOG_GAIN_MIN,
						  S5KGN5SP_ANALOG_GAIN_MAX,
						  S5KGN5SP_ANALOG_GAIN_STEP,
						  S5KGN5SP_ANALOG_GAIN_MIN);

	/*
	 * Mounting-info ctrls: libcamera's CameraSensor reads only their
	 * defaults for the Location/Rotation properties (0.8.0 has no DT
	 * fallback), so expose them read-only from the firmware values.
	 * READ_ONLY => __v4l2_ctrl_handler_setup() skips them and userspace
	 * writes get EACCES; NULL ops by design.
	 */
	device_property_read_u32(s5kgn5sp->dev, "orientation", &orientation);
	device_property_read_u32(s5kgn5sp->dev, "rotation", &rotation);

	ctrl = v4l2_ctrl_new_std_menu(ctrl_hdlr, NULL,
				      V4L2_CID_CAMERA_ORIENTATION,
				      V4L2_CAMERA_ORIENTATION_EXTERNAL, 0,
				      orientation);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrl = v4l2_ctrl_new_std(ctrl_hdlr, NULL,
				 V4L2_CID_CAMERA_SENSOR_ROTATION,
				 0, 359, 1, rotation);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	s5kgn5sp->sd.ctrl_handler = ctrl_hdlr;

	return 0;
}

static int s5kgn5sp_enable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state, u32 pad,
				   u64 streams_mask)
{
	struct s5kgn5sp *s5kgn5sp = to_s5kgn5sp(sd);
	const struct s5kgn5sp_reg_list *reg_list = &s5kgn5sp->mode->reg_list;
	int ret;

	ret = pm_runtime_resume_and_get(s5kgn5sp->dev);
	if (ret) {
		dev_err(s5kgn5sp->dev, "pm resume failed in enable: %d\n", ret);
		return ret;
	}

	/* Vendor initSettings (group 15): id-lock, FW patch upload, setup.
	 * The two vendor delays are the segment boundaries. */
	cci_multi_reg_write(s5kgn5sp->regmap, s5kgn5sp_init_a,
			    ARRAY_SIZE(s5kgn5sp_init_a), &ret);
	if (ret)
		goto error;
	usleep_range(1000, 1500);

	cci_multi_reg_write(s5kgn5sp->regmap, s5kgn5sp_init_b,
			    ARRAY_SIZE(s5kgn5sp_init_b), &ret);
	if (ret)
		goto error;
	usleep_range(20000, 21000);

	cci_multi_reg_write(s5kgn5sp->regmap, s5kgn5sp_init_c,
			    ARRAY_SIZE(s5kgn5sp_init_c), &ret);
	if (ret)
		goto error;

	cci_multi_reg_write(s5kgn5sp->regmap, reg_list->regs,
			    reg_list->num_regs, &ret);
	if (ret)
		goto error;

	ret = __v4l2_ctrl_handler_setup(s5kgn5sp->sd.ctrl_handler);

	/* Stream on, page-select first (vendor startStreaming sequence) */
	cci_write(s5kgn5sp->regmap, S5KGN5SP_REG_PAGE_SELECT, 0x4000, &ret);
	cci_write(s5kgn5sp->regmap, S5KGN5SP_REG_MODE_SELECT,
		  S5KGN5SP_MODE_STREAMING, &ret);
	if (ret)
		goto error;

	usleep_range(5 * USEC_PER_MSEC, 6 * USEC_PER_MSEC);

	return 0;

error:
	dev_err(s5kgn5sp->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(s5kgn5sp->dev);

	return ret;
}

static int s5kgn5sp_disable_streams(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state, u32 pad,
				    u64 streams_mask)
{
	struct s5kgn5sp *s5kgn5sp = to_s5kgn5sp(sd);
	int ret;

	cci_write(s5kgn5sp->regmap, S5KGN5SP_REG_PAGE_SELECT, 0x4000, NULL);
	ret = cci_write(s5kgn5sp->regmap, S5KGN5SP_REG_MODE_SELECT,
			S5KGN5SP_MODE_STANDBY, NULL);
	if (ret)
		dev_err(s5kgn5sp->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(s5kgn5sp->dev);

	return ret;
}

static void s5kgn5sp_update_pad_format(struct s5kgn5sp *s5kgn5sp,
				       const struct s5kgn5sp_mode *mode,
				       struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = s5kgn5sp_mbus_format;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5kgn5sp_set_pad_format(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_format *fmt)
{
	struct s5kgn5sp *s5kgn5sp = to_s5kgn5sp(sd);
	s64 hblank, vblank, exposure_max;
	const struct s5kgn5sp_mode *mode;
	u64 new_pixel_rate;

	mode = v4l2_find_nearest_size(s5kgn5sp_supported_modes,
				      ARRAY_SIZE(s5kgn5sp_supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	s5kgn5sp_update_pad_format(s5kgn5sp, mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY || s5kgn5sp->mode == mode)
		goto set_format;

	hblank = mode->hts - mode->width;
	__v4l2_ctrl_modify_range(s5kgn5sp->hblank, hblank,
				 S5KGN5SP_VTS_MAX - mode->width, 1, hblank);
	__v4l2_ctrl_s_ctrl(s5kgn5sp->hblank, hblank);

	vblank = mode->vts - mode->height;
	__v4l2_ctrl_modify_range(s5kgn5sp->vblank, vblank,
				 S5KGN5SP_VTS_MAX - mode->height, 1, vblank);
	__v4l2_ctrl_s_ctrl(s5kgn5sp->vblank, vblank);

	exposure_max = mode->vts - S5KGN5SP_EXPOSURE_MARGIN;
	__v4l2_ctrl_modify_range(s5kgn5sp->exposure, S5KGN5SP_EXPOSURE_MIN,
				 exposure_max, S5KGN5SP_EXPOSURE_STEP,
				 mode->vts / 2);
	__v4l2_ctrl_s_ctrl(s5kgn5sp->exposure, mode->vts / 2);

	new_pixel_rate = s5kgn5sp_freq_to_pixel_rate(
				s5kgn5sp_link_freq_menu[mode->link_freq_idx]);
	__v4l2_ctrl_modify_range(s5kgn5sp->pixel_rate, 0, new_pixel_rate,
				 1, new_pixel_rate);
	__v4l2_ctrl_s_ctrl_int64(s5kgn5sp->pixel_rate, new_pixel_rate);
	__v4l2_ctrl_s_ctrl(s5kgn5sp->link_freq, mode->link_freq_idx);

	if (s5kgn5sp->sd.ctrl_handler->error)
		return s5kgn5sp->sd.ctrl_handler->error;

	s5kgn5sp->mode = mode;

set_format:
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int s5kgn5sp_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = s5kgn5sp_mbus_format;

	return 0;
}

static int s5kgn5sp_enum_frame_size(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(s5kgn5sp_supported_modes))
		return -EINVAL;

	if (fse->code != s5kgn5sp_mbus_format)
		return -EINVAL;

	fse->min_width = s5kgn5sp_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5kgn5sp_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int s5kgn5sp_get_selection(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_selection *sel)
{
	struct s5kgn5sp *s5kgn5sp = to_s5kgn5sp(sd);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = s5kgn5sp->mode->width;
		sel->r.height = s5kgn5sp->mode->height;
		return 0;
	default:
		return -EINVAL;
	}
}

static int s5kgn5sp_init_state(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state)
{
	struct s5kgn5sp *s5kgn5sp = to_s5kgn5sp(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = s5kgn5sp->mode->width,
			.height = s5kgn5sp->mode->height,
		},
	};

	s5kgn5sp_set_pad_format(sd, state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops s5kgn5sp_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5kgn5sp_pad_ops = {
	.set_fmt = s5kgn5sp_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = s5kgn5sp_get_selection,
	.enum_mbus_code = s5kgn5sp_enum_mbus_code,
	.enum_frame_size = s5kgn5sp_enum_frame_size,
	.enable_streams = s5kgn5sp_enable_streams,
	.disable_streams = s5kgn5sp_disable_streams,
};

static const struct v4l2_subdev_ops s5kgn5sp_subdev_ops = {
	.video = &s5kgn5sp_video_ops,
	.pad = &s5kgn5sp_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5kgn5sp_internal_ops = {
	.init_state = s5kgn5sp_init_state,
};

static const struct media_entity_operations s5kgn5sp_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/*
 * Bring-up instruments (GD2-driver pattern; remove when capture works):
 * keep_power = hold rails after a FAILED probe (scan the bus with power
 * applied, client unbound); no_probe = skip chip-id so probe SUCCEEDS and
 * autosuspend stays disabled (fully-powered bound scan).
 */
static bool keep_power;
module_param(keep_power, bool, 0444);
MODULE_PARM_DESC(keep_power, "keep rails on after failed probe (bus scan)");

static bool no_probe;
module_param(no_probe, bool, 0444);
MODULE_PARM_DESC(no_probe, "skip chip-id identify, hold power state");

static int s5kgn5sp_identify_sensor(struct s5kgn5sp *s5kgn5sp)
{
	u64 val;
	int ret, i;

	for (i = 0; i < 3; i++) {
		ret = cci_read(s5kgn5sp->regmap, S5KGN5SP_REG_CHIP_ID, &val,
			       NULL);
		dev_info(s5kgn5sp->dev,
			 "identify try %d: read 0x0000 -> ret=%d val=%#llx\n",
			 i, ret, ret ? 0 : val);
		if (!ret)
			break;
		usleep_range(50000, 60000);
	}
	if (ret) {
		dev_err(s5kgn5sp->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (val != S5KGN5SP_CHIP_ID) {
		dev_err(s5kgn5sp->dev, "chip id mismatch: %llx!=%x\n", val,
			S5KGN5SP_CHIP_ID);
		return -ENODEV;
	}

	return 0;
}

static int s5kgn5sp_check_hwcfg(struct s5kgn5sp *s5kgn5sp)
{
	struct fwnode_handle *fwnode = dev_fwnode(s5kgn5sp->dev), *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus = {
			.mipi_csi2 = {
				.num_data_lanes = S5KGN5SP_DATA_LANES,
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

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != S5KGN5SP_DATA_LANES) {
		dev_err(s5kgn5sp->dev, "Invalid number of data lanes: %u\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto endpoint_free;
	}

	ret = v4l2_link_freq_to_bitmap(s5kgn5sp->dev,
				       bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       s5kgn5sp_link_freq_menu,
				       ARRAY_SIZE(s5kgn5sp_link_freq_menu),
				       &freq_bitmap);

endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int s5kgn5sp_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5kgn5sp *s5kgn5sp = to_s5kgn5sp(sd);
	int ret;

	/*
	 * Bin powerSetting (vana+1ms -> vdig -> vio -> gpio -> mclk -> gpio).
	 * Runtime truth (r109 GD2 precedent, same -110 signature): the
	 * vendor delays understate the post-release settle — 12ms gave
	 * identify -110 there, stock bin said 70ms; mirror the proven GD2
	 * sequence (rails -> 5ms -> mclk -> reset release -> 75ms -> i2c).
	 */
	ret = regulator_enable(s5kgn5sp->vddio);
	if (ret) {
		dev_err(s5kgn5sp->dev, "vddio enable failed: %d\n", ret);
		return ret;
	}

	ret = regulator_enable(s5kgn5sp->vddd);
	if (ret) {
		dev_err(s5kgn5sp->dev, "vddd enable failed: %d\n", ret);
		goto disable_vddio;
	}

	ret = regulator_enable(s5kgn5sp->vdda);
	if (ret) {
		dev_err(s5kgn5sp->dev, "vdda enable failed: %d\n", ret);
		goto disable_vddd;
	}

	usleep_range(5000, 6000);

	ret = clk_prepare_enable(s5kgn5sp->mclk);
	if (ret) {
		dev_err(s5kgn5sp->dev, "mclk enable failed: %d\n", ret);
		goto disable_vdda;
	}

	gpiod_set_value_cansleep(s5kgn5sp->reset_gpio, 0);
	usleep_range(75000, 80000);

	return 0;

disable_vddio:
	regulator_disable(s5kgn5sp->vddio);
disable_vddd:
	regulator_disable(s5kgn5sp->vddd);
disable_vdda:
	regulator_disable(s5kgn5sp->vdda);

	return ret;
}

static int s5kgn5sp_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5kgn5sp *s5kgn5sp = to_s5kgn5sp(sd);

	gpiod_set_value_cansleep(s5kgn5sp->reset_gpio, 1);

	clk_disable_unprepare(s5kgn5sp->mclk);

	regulator_disable(s5kgn5sp->vddio);
	regulator_disable(s5kgn5sp->vddd);
	regulator_disable(s5kgn5sp->vdda);

	return 0;
}

static int s5kgn5sp_probe(struct i2c_client *client)
{
	struct s5kgn5sp *s5kgn5sp;
	unsigned long freq;
	int ret;

	s5kgn5sp = devm_kzalloc(&client->dev, sizeof(*s5kgn5sp), GFP_KERNEL);
	if (!s5kgn5sp)
		return -ENOMEM;

	s5kgn5sp->dev = &client->dev;
	v4l2_i2c_subdev_init(&s5kgn5sp->sd, client, &s5kgn5sp_subdev_ops);

	s5kgn5sp->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5kgn5sp->regmap))
		return dev_err_probe(s5kgn5sp->dev, PTR_ERR(s5kgn5sp->regmap),
				     "failed to init CCI\n");

	s5kgn5sp->mclk = devm_v4l2_sensor_clk_get(s5kgn5sp->dev, NULL);
	if (IS_ERR(s5kgn5sp->mclk))
		return dev_err_probe(s5kgn5sp->dev, PTR_ERR(s5kgn5sp->mclk),
				     "failed to get MCLK clock\n");

	freq = clk_get_rate(s5kgn5sp->mclk);
	if (freq != S5KGN5SP_MCLK_FREQ_19MHZ)
		return dev_err_probe(s5kgn5sp->dev, -EINVAL,
				     "MCLK clock frequency %lu is not supported\n",
				     freq);

	ret = s5kgn5sp_check_hwcfg(s5kgn5sp);
	if (ret)
		return dev_err_probe(s5kgn5sp->dev, ret,
				     "failed to check HW configuration\n");

	s5kgn5sp->reset_gpio = devm_gpiod_get_optional(s5kgn5sp->dev, "reset",
						       GPIOD_OUT_HIGH);
	if (IS_ERR(s5kgn5sp->reset_gpio))
		return dev_err_probe(s5kgn5sp->dev,
				     PTR_ERR(s5kgn5sp->reset_gpio),
				     "cannot get reset GPIO\n");

	s5kgn5sp->vdda = devm_regulator_get_optional(s5kgn5sp->dev, "vdda");
	if (IS_ERR(s5kgn5sp->vdda))
		return dev_err_probe(s5kgn5sp->dev, PTR_ERR(s5kgn5sp->vdda),
				     "cannot get vdda regulator\n");

	s5kgn5sp->vddd = devm_regulator_get_optional(s5kgn5sp->dev, "vddd");
	if (IS_ERR(s5kgn5sp->vddd))
		return dev_err_probe(s5kgn5sp->dev, PTR_ERR(s5kgn5sp->vddd),
				     "cannot get vddd regulator\n");

	s5kgn5sp->vddio = devm_regulator_get_optional(s5kgn5sp->dev, "vddio");
	if (IS_ERR(s5kgn5sp->vddio))
		return dev_err_probe(s5kgn5sp->dev, PTR_ERR(s5kgn5sp->vddio),
				     "cannot get vddio regulator\n");

	/* The sensor must be powered on to read the CHIP_ID register */
	ret = s5kgn5sp_power_on(s5kgn5sp->dev);
	if (ret)
		return ret;

	if (!no_probe) {
		ret = s5kgn5sp_identify_sensor(s5kgn5sp);
		if (ret) {
			dev_err_probe(s5kgn5sp->dev, ret,
				      "failed to find sensor\n");
			if (keep_power) {
				dev_err(s5kgn5sp->dev,
					"keep_power=1: rails held for bus scan\n");
				return ret;
			}
			goto power_off;
		}
	} else {
		dev_err(s5kgn5sp->dev,
			"no_probe=1: skipping chip-id, holding full power state\n");
	}

	s5kgn5sp->mode = &s5kgn5sp_supported_modes[0];
	ret = s5kgn5sp_init_controls(s5kgn5sp);
	if (ret) {
		dev_err_probe(s5kgn5sp->dev, ret, "failed to init controls\n");
		goto power_off;
	}

	s5kgn5sp->sd.state_lock = s5kgn5sp->ctrl_handler.lock;
	s5kgn5sp->sd.internal_ops = &s5kgn5sp_internal_ops;
	s5kgn5sp->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5kgn5sp->sd.entity.ops = &s5kgn5sp_subdev_entity_ops;
	s5kgn5sp->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5kgn5sp->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&s5kgn5sp->sd.entity, 1, &s5kgn5sp->pad);
	if (ret) {
		dev_err_probe(s5kgn5sp->dev, ret,
			      "failed to init media entity pads\n");
		goto v4l2_ctrl_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&s5kgn5sp->sd);
	if (ret < 0) {
		dev_err_probe(s5kgn5sp->dev, ret,
			      "failed to init v4l2 subdev\n");
		goto media_entity_cleanup;
	}

	pm_runtime_set_active(s5kgn5sp->dev);
	pm_runtime_enable(s5kgn5sp->dev);

	ret = v4l2_async_register_subdev_sensor(&s5kgn5sp->sd);
	if (ret < 0) {
		dev_err_probe(s5kgn5sp->dev, ret,
			      "failed to register V4L2 subdev\n");
		goto subdev_cleanup;
	}

	pm_runtime_set_autosuspend_delay(s5kgn5sp->dev,
					 no_probe ? -1 : 1000);
	pm_runtime_use_autosuspend(s5kgn5sp->dev);
	pm_runtime_idle(s5kgn5sp->dev);

	return 0;

subdev_cleanup:
	v4l2_subdev_cleanup(&s5kgn5sp->sd);
	pm_runtime_disable(s5kgn5sp->dev);
	pm_runtime_set_suspended(s5kgn5sp->dev);

media_entity_cleanup:
	media_entity_cleanup(&s5kgn5sp->sd.entity);

v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(s5kgn5sp->sd.ctrl_handler);

power_off:
	s5kgn5sp_power_off(s5kgn5sp->dev);

	return ret;
}

static void s5kgn5sp_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5kgn5sp *s5kgn5sp = to_s5kgn5sp(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(s5kgn5sp->dev);

	if (!pm_runtime_status_suspended(s5kgn5sp->dev)) {
		s5kgn5sp_power_off(s5kgn5sp->dev);
		pm_runtime_set_suspended(s5kgn5sp->dev);
	}
}

static const struct dev_pm_ops s5kgn5sp_pm_ops = {
	SET_RUNTIME_PM_OPS(s5kgn5sp_power_off, s5kgn5sp_power_on, NULL)
};

static const struct of_device_id s5kgn5sp_of_match[] = {
	{ .compatible = "samsung,s5kgn5sp" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5kgn5sp_of_match);

static struct i2c_driver s5kgn5sp_i2c_driver = {
	.driver = {
		.name = "s5kgn5sp",
		.pm = &s5kgn5sp_pm_ops,
		.of_match_table = s5kgn5sp_of_match,
	},
	.probe = s5kgn5sp_probe,
	.remove = s5kgn5sp_remove,
};

module_i2c_driver(s5kgn5sp_i2c_driver);

MODULE_AUTHOR("Nura Meizu 20 porting crew");
MODULE_DESCRIPTION("Samsung S5KGN5SP image sensor driver");
MODULE_LICENSE("GPL");
