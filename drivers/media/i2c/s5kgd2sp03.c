// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S5KGD2SP03 32MP raw image sensor driver (Meizu 20 front camera).
 *
 * All parameters reverse-engineered from device-specific truth, no guessing
 * (2026-11-21 GD2 round):
 *  - I2C addr 0x10 / chip id 0x0842 @0x0000: Meizu sensormodule .bin
 *    differential field fit vs the known-good 3P9 bin (slave 0x5a/8bit =
 *    0x2d/7bit, id 0x3109), cross-confirmed by the bin's own boot stream
 *    (0x0000 <- 0x0842) and the public Volla mt6878 s5kgd2sp driver
 *    (i2c_addr_table {0x20, 0x5a} = 8-bit forms of 0x10/0x2d).
 *  - 4-lane MIPI D-PHY: stock module_gd2.dtsi (Samsung A54, CSI_DATA_LANES_4)
 *    + MTK imgsensor SENSOR_MIPI_4_LANE.
 *  - Supplies: stock FDT qcom,cam-sensor1 (C1_M1_S5KGD2SP03) phandles:
 *    vio=1.8V (pm8008j l5), vdig=1.056V (pm8008i l1), vana=2.8V (pm8008i l3);
 *    voltage set cross-confirmed by Samsung A54 power seq (VDDA_2.8/VDDD_1.05/
 *    VDDIO_1.8). Reset active-low (Samsung "sen_rst low" = asserted).
 *  - MCLK4 = tlmm gpio104 / RESET4 = gpio120 / CCI2 master1 / csiphy4:
 *    stock FDT (gpios <tlmm 104>/<tlmm 120>, cci-master 1, csiphy-sd-index 4).
 *  - Modes (sensormodule bin resolutionData <-> regSetting groups, one PLL
 *    block shared by all three):
 *      g0 = 6528x4896  FLL 5128 / LLP 20592  (15 fps)
 *      g1 = 3264x2448  FLL 2932 / LLP 18000  (30 fps)
 *      g2 = 3264x2448  FLL 2932 / LLP 9040   (60 fps)
 *    fps truth = the bin's own per-mode rate doubles (15.05/30.05/60.03),
 *    which back-solve to pix = 19.2MHz/7*145*2 = 1590.9MHz at <=0.3% spread.
 *    MIPI lane rate = pix/2 DDR = 1590.9 Mbps/lane -> link-freq 795.43MHz
 *    (bandwidth check: worst mode needs 1.44Gbps/lane < 1.59G ✓; Samsung
 *    A54 runs the same die at 2054 Mbps/lane, Meizu trims slower).
 *  - Stream on/off = 0x0100 <- 0x0001/0x0000 (bin groups g3/g4; Volla
 *    reg_addr_stream = 0x0100 write 0x01).
 *  - Exposure = 0x0202 (Volla reg_addr_exposure), min 12 lines (bin
 *    minLineCount). Analog gain = 0x0204, code/1024 = multiplier, 1x = 1024,
 *    max 16x = 16384 (Volla s5kgd2sp ana_gain table).
 *  - Bayer: quad (4-cell) die, hardware-binned output is Gr-first standard
 *    Bayer (MTK RAW_4CELL_HW_BAYER_Gr) -> SGRGB10 labels; single-shot
 *    validation against a known-colour target still pending (3P9 lesson:
 *    verify empirically, the grey-path decode cannot distinguish).
 *  - Full-res mode is quad-bayer; the stock remosaic patch (g7 tail) is
 *    applied at every stream start like the vendor FW patch. If binned
 *    modes misbehave on hardware, try dropping the patch per-mode.
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

#define S5KGD2_MCLK_FREQ_19MHZ		(19200000)
#define S5KGD2_DATA_LANES		4

#define S5KGD2_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5KGD2_CHIP_ID			0x0842

/* Samsung page select (word port: 0x602a = addr, 0x6f12 = data) */
#define S5KGD2_REG_PAGE_SELECT		CCI_REG16(0x6028)

#define S5KGD2_REG_MODE_SELECT		CCI_REG16(0x0100)
#define S5KGD2_MODE_STREAMING		0x0001	/* bin g3: 0x0100 <- 0x0001 */
#define S5KGD2_MODE_STANDBY		0x0000	/* bin g4 */

#define S5KGD2_REG_GROUP_HOLD		CCI_REG16(0x0104)	/* bin g5/g6 */
#define S5KGD2_GROUP_HOLD_ON		0x0001
#define S5KGD2_GROUP_HOLD_OFF		0x0000

#define S5KGD2_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5KGD2_EXPOSURE_MIN		12	/* bin minLineCount = 0xc */
#define S5KGD2_EXPOSURE_STEP		1
#define S5KGD2_EXPOSURE_MARGIN		12

#define S5KGD2_REG_ANALOG_GAIN		CCI_REG16(0x0204)
#define S5KGD2_ANALOG_GAIN_MIN		1024	/* code/1024 = x, 1x */
#define S5KGD2_ANALOG_GAIN_MAX		16384	/* 16x (Volla gain table) */
#define S5KGD2_ANALOG_GAIN_STEP		32

#define S5KGD2_REG_VTS			CCI_REG16(0x0340)
#define S5KGD2_VTS_MAX			0xffff
#define S5KGD2_REG_HTS			CCI_REG16(0x0342)

#define to_s5kgd2(_sd)			container_of(_sd, struct s5kgd2, sd)

/*
 * Single PLL block shared by all three bin modes:
 * pix = 19.2MHz / pre(7) * mult(145) * x2 = 1590.86 MHz
 * MIPI lane clock = pix/2 (DDR) -> link-freq 795.43 MHz per lane.
 */
static const s64 s5kgd2_link_freq_menu[] = {
	795430000,
};
#define S5KGD2_LF_IDX_ALL		0

/* Quad-bayer die; HW-binned output is Gr-first standard Bayer. */
static const u32 s5kgd2_mbus_format = MEDIA_BUS_FMT_SGRBG10_1X10;

/* g7 head: page 0x4000, ID verify (0x0842), patch trigger; +5ms after last write */
static const struct cci_reg_sequence s5kgd2_boot_seq[] = {
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x0000), 0x0007 },
	{ CCI_REG16(0x0000), 0x0842 },
	{ CCI_REG16(0x001e), 0x0300 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x6010), 0x0001 },
};

/* PLL kick 0x6214/0x6218; +11ms after */
static const struct cci_reg_sequence s5kgd2_pll_kick_seq[] = {
	{ CCI_REG16(0x6214), 0xff7d },
	{ CCI_REG16(0x6218), 0x0000 },
};

/* g7 tail: sensor FW patch table (1256 recs, page 0x2001 @ 0xd604+) */
static const struct cci_reg_sequence s5kgd2_patch_seq[] = {
	{ CCI_REG16(0x6028), 0x2001 },
	{ CCI_REG16(0x602a), 0xd604 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0348 },
	{ CCI_REG16(0x6f12), 0x90f8 },
	{ CCI_REG16(0x6f12), 0xda0c },
	{ CCI_REG16(0x6f12), 0x0128 },
	{ CCI_REG16(0x6f12), 0x01d0 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x09b8 },
	{ CCI_REG16(0x6f12), 0x7047 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x12e0 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0548 },
	{ CCI_REG16(0x6f12), 0x0449 },
	{ CCI_REG16(0x6f12), 0x054a },
	{ CCI_REG16(0x6f12), 0xc0f8 },
	{ CCI_REG16(0x6f12), 0x8c17 },
	{ CCI_REG16(0x6f12), 0x511a },
	{ CCI_REG16(0x6f12), 0xc0f8 },
	{ CCI_REG16(0x6f12), 0x9017 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x50ba },
	{ CCI_REG16(0x6f12), 0x2001 },
	{ CCI_REG16(0x6f12), 0xdcb8 },
	{ CCI_REG16(0x6f12), 0x2001 },
	{ CCI_REG16(0x6f12), 0x8400 },
	{ CCI_REG16(0x6f12), 0x2002 },
	{ CCI_REG16(0x6f12), 0x3600 },
	{ CCI_REG16(0x6f12), 0x2de9 },
	{ CCI_REG16(0x6f12), 0xf041 },
	{ CCI_REG16(0x6f12), 0xfe4d },
	{ CCI_REG16(0x6f12), 0x0646 },
	{ CCI_REG16(0x6f12), 0xfe4f },
	{ CCI_REG16(0x6f12), 0xac89 },
	{ CCI_REG16(0x6f12), 0x2007 },
	{ CCI_REG16(0x6f12), 0x08d5 },
	{ CCI_REG16(0x6f12), 0xb7f8 },
	{ CCI_REG16(0x6f12), 0xfe08 },
	{ CCI_REG16(0x6f12), 0x28b1 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x0821 },
	{ CCI_REG16(0x6f12), 0x46f2 },
	{ CCI_REG16(0x6f12), 0x1420 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x89fa },
	{ CCI_REG16(0x6f12), 0x24f0 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0xf84c },
	{ CCI_REG16(0x6f12), 0xa4f8 },
	{ CCI_REG16(0x6f12), 0x1802 },
	{ CCI_REG16(0x6f12), 0x4ff4 },
	{ CCI_REG16(0x6f12), 0x8068 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x4146 },
	{ CCI_REG16(0x6f12), 0x46f2 },
	{ CCI_REG16(0x6f12), 0x3020 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x7cfa },
	{ CCI_REG16(0x6f12), 0xf148 },
	{ CCI_REG16(0x6f12), 0x7830 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x7dfa },
	{ CCI_REG16(0x6f12), 0x00b1 },
	{ CCI_REG16(0x6f12), 0x3eb1 },
	{ CCI_REG16(0x6f12), 0xe888 },
	{ CCI_REG16(0x6f12), 0xa4f8 },
	{ CCI_REG16(0x6f12), 0x0e02 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x7bfa },
	{ CCI_REG16(0x6f12), 0x3864 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0xf081 },
	{ CCI_REG16(0x6f12), 0x46f2 },
	{ CCI_REG16(0x6f12), 0x0e24 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x4146 },
	{ CCI_REG16(0x6f12), 0x2046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x67fa },
	{ CCI_REG16(0x6f12), 0x0120 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x73fa },
	{ CCI_REG16(0x6f12), 0x0122 },
	{ CCI_REG16(0x6f12), 0x8021 },
	{ CCI_REG16(0x6f12), 0x2046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x5ffa },
	{ CCI_REG16(0x6f12), 0xea88 },
	{ CCI_REG16(0x6f12), 0x4ff6 },
	{ CCI_REG16(0x6f12), 0xff51 },
	{ CCI_REG16(0x6f12), 0x2046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x6dfa },
	{ CCI_REG16(0x6f12), 0xe4e7 },
	{ CCI_REG16(0x6f12), 0x2de9 },
	{ CCI_REG16(0x6f12), 0xf04f },
	{ CCI_REG16(0x6f12), 0xe048 },
	{ CCI_REG16(0x6f12), 0x91b0 },
	{ CCI_REG16(0x6f12), 0x0568 },
	{ CCI_REG16(0x6f12), 0x95f8 },
	{ CCI_REG16(0x6f12), 0x3c73 },
	{ CCI_REG16(0x6f12), 0x05f5 },
	{ CCI_REG16(0x6f12), 0x4f75 },
	{ CCI_REG16(0x6f12), 0xf807 },
	{ CCI_REG16(0x6f12), 0x78d0 },
	{ CCI_REG16(0x6f12), 0x0220 },
	{ CCI_REG16(0x6f12), 0x00ea },
	{ CCI_REG16(0x6f12), 0x5704 },
	{ CCI_REG16(0x6f12), 0x6888 },
	{ CCI_REG16(0x6f12), 0x1090 },
	{ CCI_REG16(0x6f12), 0xb0f5 },
	{ CCI_REG16(0x6f12), 0x806f },
	{ CCI_REG16(0x6f12), 0x01d3 },
	{ CCI_REG16(0x6f12), 0x44f0 },
	{ CCI_REG16(0x6f12), 0x1004 },
	{ CCI_REG16(0x6f12), 0x002c },
	{ CCI_REG16(0x6f12), 0x60d1 },
	{ CCI_REG16(0x6f12), 0xb807 },
	{ CCI_REG16(0x6f12), 0x52d5 },
	{ CCI_REG16(0x6f12), 0xd649 },
	{ CCI_REG16(0x6f12), 0xb1f8 },
	{ CCI_REG16(0x6f12), 0x0a01 },
	{ CCI_REG16(0x6f12), 0x38b1 },
	{ CCI_REG16(0x6f12), 0x0128 },
	{ CCI_REG16(0x6f12), 0x0ad0 },
	{ CCI_REG16(0x6f12), 0x0228 },
	{ CCI_REG16(0x6f12), 0x16d0 },
	{ CCI_REG16(0x6f12), 0x54f0 },
	{ CCI_REG16(0x6f12), 0x8004 },
	{ CCI_REG16(0x6f12), 0x0ad0 },
	{ CCI_REG16(0x6f12), 0x52e0 },
	{ CCI_REG16(0x6f12), 0x4ff0 },
	{ CCI_REG16(0x6f12), 0x000b },
	{ CCI_REG16(0x6f12), 0x4ff4 },
	{ CCI_REG16(0x6f12), 0x8030 },
	{ CCI_REG16(0x6f12), 0x03e0 },
	{ CCI_REG16(0x6f12), 0xb1f8 },
	{ CCI_REG16(0x6f12), 0x0cb1 },
	{ CCI_REG16(0x6f12), 0xb1f8 },
	{ CCI_REG16(0x6f12), 0x0e01 },
	{ CCI_REG16(0x6f12), 0x8246 },
	{ CCI_REG16(0x6f12), 0x6946 },
	{ CCI_REG16(0x6f12), 0x1098 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x3efa },
	{ CCI_REG16(0x6f12), 0x4028 },
	{ CCI_REG16(0x6f12), 0x07d0 },
	{ CCI_REG16(0x6f12), 0x44f0 },
	{ CCI_REG16(0x6f12), 0x0104 },
	{ CCI_REG16(0x6f12), 0x3fe0 },
	{ CCI_REG16(0x6f12), 0xb1f8 },
	{ CCI_REG16(0x6f12), 0x10b1 },
	{ CCI_REG16(0x6f12), 0xb1f8 },
	{ CCI_REG16(0x6f12), 0x1201 },
	{ CCI_REG16(0x6f12), 0xf0e7 },
	{ CCI_REG16(0x6f12), 0xe946 },
	{ CCI_REG16(0x6f12), 0x05f1 },
	{ CCI_REG16(0x6f12), 0x0408 },
	{ CCI_REG16(0x6f12), 0x0026 },
	{ CCI_REG16(0x6f12), 0xd8f8 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0xc1b1 },
	{ CCI_REG16(0x6f12), 0xd9f8 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x20b1 },
	{ CCI_REG16(0x6f12), 0x8842 },
	{ CCI_REG16(0x6f12), 0x13d0 },
	{ CCI_REG16(0x6f12), 0x44f0 },
	{ CCI_REG16(0x6f12), 0x0404 },
	{ CCI_REG16(0x6f12), 0x10e0 },
	{ CCI_REG16(0x6f12), 0x1098 },
	{ CCI_REG16(0x6f12), 0x06eb },
	{ CCI_REG16(0x6f12), 0x8010 },
	{ CCI_REG16(0x6f12), 0x8345 },
	{ CCI_REG16(0x6f12), 0x09d8 },
	{ CCI_REG16(0x6f12), 0x5045 },
	{ CCI_REG16(0x6f12), 0x07d2 },
	{ CCI_REG16(0x6f12), 0x4246 },
	{ CCI_REG16(0x6f12), 0x0421 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x1ffa },
	{ CCI_REG16(0x6f12), 0x20b9 },
	{ CCI_REG16(0x6f12), 0x44f0 },
	{ CCI_REG16(0x6f12), 0x4004 },
	{ CCI_REG16(0x6f12), 0x01e0 },
	{ CCI_REG16(0x6f12), 0x44f0 },
	{ CCI_REG16(0x6f12), 0x2004 },
	{ CCI_REG16(0x6f12), 0x361d },
	{ CCI_REG16(0x6f12), 0x09f1 },
	{ CCI_REG16(0x6f12), 0x0409 },
	{ CCI_REG16(0x6f12), 0x08f1 },
	{ CCI_REG16(0x6f12), 0x0408 },
	{ CCI_REG16(0x6f12), 0x402e },
	{ CCI_REG16(0x6f12), 0xdcd3 },
	{ CCI_REG16(0x6f12), 0x3806 },
	{ CCI_REG16(0x6f12), 0x11d5 },
	{ CCI_REG16(0x6f12), 0x4022 },
	{ CCI_REG16(0x6f12), 0x0021 },
	{ CCI_REG16(0x6f12), 0x281d },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x10fa },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x6f12), 0x291d },
	{ CCI_REG16(0x6f12), 0x1098 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x01fa },
	{ CCI_REG16(0x6f12), 0x4028 },
	{ CCI_REG16(0x6f12), 0x05d0 },
	{ CCI_REG16(0x6f12), 0x4022 },
	{ CCI_REG16(0x6f12), 0x0021 },
	{ CCI_REG16(0x6f12), 0x281d },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x04fa },
	{ CCI_REG16(0x6f12), 0xbce7 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x06fa },
	{ CCI_REG16(0x6f12), 0x08b1 },
	{ CCI_REG16(0x6f12), 0x44f0 },
	{ CCI_REG16(0x6f12), 0x0804 },
	{ CCI_REG16(0x6f12), 0x0cb1 },
	{ CCI_REG16(0x6f12), 0x47f0 },
	{ CCI_REG16(0x6f12), 0x0407 },
	{ CCI_REG16(0x6f12), 0x27f0 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x6c70 },
	{ CCI_REG16(0x6f12), 0x2870 },
	{ CCI_REG16(0x6f12), 0x11b0 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0xf08f },
	{ CCI_REG16(0x6f12), 0x70b5 },
	{ CCI_REG16(0x6f12), 0x0446 },
	{ CCI_REG16(0x6f12), 0x9f48 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x8068 },
	{ CCI_REG16(0x6f12), 0x86b2 },
	{ CCI_REG16(0x6f12), 0x050c },
	{ CCI_REG16(0x6f12), 0x3146 },
	{ CCI_REG16(0x6f12), 0x2846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xc6f9 },
	{ CCI_REG16(0x6f12), 0x2046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xf0f9 },
	{ CCI_REG16(0x6f12), 0x0122 },
	{ CCI_REG16(0x6f12), 0x3146 },
	{ CCI_REG16(0x6f12), 0x2846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xbef9 },
	{ CCI_REG16(0x6f12), 0x2188 },
	{ CCI_REG16(0x6f12), 0x4ff0 },
	{ CCI_REG16(0x6f12), 0x8040 },
	{ CCI_REG16(0x6f12), 0xb0fb },
	{ CCI_REG16(0x6f12), 0xf1f1 },
	{ CCI_REG16(0x6f12), 0x01f5 },
	{ CCI_REG16(0x6f12), 0x0071 },
	{ CCI_REG16(0x6f12), 0x890a },
	{ CCI_REG16(0x6f12), 0xe180 },
	{ CCI_REG16(0x6f12), 0x6188 },
	{ CCI_REG16(0x6f12), 0xb0fb },
	{ CCI_REG16(0x6f12), 0xf1f1 },
	{ CCI_REG16(0x6f12), 0x01f5 },
	{ CCI_REG16(0x6f12), 0x0071 },
	{ CCI_REG16(0x6f12), 0x890a },
	{ CCI_REG16(0x6f12), 0x2181 },
	{ CCI_REG16(0x6f12), 0xa188 },
	{ CCI_REG16(0x6f12), 0xb0fb },
	{ CCI_REG16(0x6f12), 0xf1f0 },
	{ CCI_REG16(0x6f12), 0x00f5 },
	{ CCI_REG16(0x6f12), 0x0070 },
	{ CCI_REG16(0x6f12), 0x800a },
	{ CCI_REG16(0x6f12), 0x6081 },
	{ CCI_REG16(0x6f12), 0x70bd },
	{ CCI_REG16(0x6f12), 0x2de9 },
	{ CCI_REG16(0x6f12), 0xf84f },
	{ CCI_REG16(0x6f12), 0x8246 },
	{ CCI_REG16(0x6f12), 0x8948 },
	{ CCI_REG16(0x6f12), 0x8946 },
	{ CCI_REG16(0x6f12), 0x9046 },
	{ CCI_REG16(0x6f12), 0xc168 },
	{ CCI_REG16(0x6f12), 0x1e46 },
	{ CCI_REG16(0x6f12), 0x0d0c },
	{ CCI_REG16(0x6f12), 0x8fb2 },
	{ CCI_REG16(0x6f12), 0x0a9c },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x3946 },
	{ CCI_REG16(0x6f12), 0x2846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x96f9 },
	{ CCI_REG16(0x6f12), 0x3346 },
	{ CCI_REG16(0x6f12), 0x4246 },
	{ CCI_REG16(0x6f12), 0x4946 },
	{ CCI_REG16(0x6f12), 0x5046 },
	{ CCI_REG16(0x6f12), 0x0094 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xc1f9 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xc4f9 },
	{ CCI_REG16(0x6f12), 0x10b1 },
	{ CCI_REG16(0x6f12), 0xa088 },
	{ CCI_REG16(0x6f12), 0x401e },
	{ CCI_REG16(0x6f12), 0xa080 },
	{ CCI_REG16(0x6f12), 0x3946 },
	{ CCI_REG16(0x6f12), 0x2846 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0xf84f },
	{ CCI_REG16(0x6f12), 0x0122 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x82b9 },
	{ CCI_REG16(0x6f12), 0x10b5 },
	{ CCI_REG16(0x6f12), 0x0721 },
	{ CCI_REG16(0x6f12), 0x0220 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xb9f9 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0x1040 },
	{ CCI_REG16(0x6f12), 0x4ff6 },
	{ CCI_REG16(0x6f12), 0xff71 },
	{ CCI_REG16(0x6f12), 0x0a20 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xb2b9 },
	{ CCI_REG16(0x6f12), 0x2de9 },
	{ CCI_REG16(0x6f12), 0xf05f },
	{ CCI_REG16(0x6f12), 0x6d48 },
	{ CCI_REG16(0x6f12), 0x724c },
	{ CCI_REG16(0x6f12), 0x4ff4 },
	{ CCI_REG16(0x6f12), 0xd44b },
	{ CCI_REG16(0x6f12), 0x90f8 },
	{ CCI_REG16(0x6f12), 0x9a07 },
	{ CCI_REG16(0x6f12), 0x4ff0 },
	{ CCI_REG16(0x6f12), 0x0009 },
	{ CCI_REG16(0x6f12), 0xc4f8 },
	{ CCI_REG16(0x6f12), 0x3c90 },
	{ CCI_REG16(0x6f12), 0x2034 },
	{ CCI_REG16(0x6f12), 0x48b1 },
	{ CCI_REG16(0x6f12), 0xdff8 },
	{ CCI_REG16(0x6f12), 0xb481 },
	{ CCI_REG16(0x6f12), 0xb8f8 },
	{ CCI_REG16(0x6f12), 0x7800 },
	{ CCI_REG16(0x6f12), 0xb8f8 },
	{ CCI_REG16(0x6f12), 0x7a10 },
	{ CCI_REG16(0x6f12), 0x8842 },
	{ CCI_REG16(0x6f12), 0x05d2 },
	{ CCI_REG16(0x6f12), 0x6180 },
	{ CCI_REG16(0x6f12), 0x04e0 },
	{ CCI_REG16(0x6f12), 0x0520 },
	{ CCI_REG16(0x6f12), 0x2070 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0xf09f },
	{ CCI_REG16(0x6f12), 0x6080 },
	{ CCI_REG16(0x6f12), 0x604d },
	{ CCI_REG16(0x6f12), 0x6088 },
	{ CCI_REG16(0x6f12), 0x4ff4 },
	{ CCI_REG16(0x6f12), 0x7a7a },
	{ CCI_REG16(0x6f12), 0xef6b },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xa7fb },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0x5346 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x8ff9 },
	{ CCI_REG16(0x6f12), 0xb8f8 },
	{ CCI_REG16(0x6f12), 0x7c00 },
	{ CCI_REG16(0x6f12), 0x0e46 },
	{ CCI_REG16(0x6f12), 0xa7fb },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x5346 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x86f9 },
	{ CCI_REG16(0x6f12), 0xb5f8 },
	{ CCI_REG16(0x6f12), 0x7603 },
	{ CCI_REG16(0x6f12), 0x761e },
	{ CCI_REG16(0x6f12), 0x401c },
	{ CCI_REG16(0x6f12), 0xb6fb },
	{ CCI_REG16(0x6f12), 0xf0f0 },
	{ CCI_REG16(0x6f12), 0xa081 },
	{ CCI_REG16(0x6f12), 0xb5f8 },
	{ CCI_REG16(0x6f12), 0x7603 },
	{ CCI_REG16(0x6f12), 0x491e },
	{ CCI_REG16(0x6f12), 0x401c },
	{ CCI_REG16(0x6f12), 0xb1fb },
	{ CCI_REG16(0x6f12), 0xf0f0 },
	{ CCI_REG16(0x6f12), 0x6081 },
	{ CCI_REG16(0x6f12), 0xb5f8 },
	{ CCI_REG16(0x6f12), 0x4003 },
	{ CCI_REG16(0x6f12), 0xe081 },
	{ CCI_REG16(0x6f12), 0xa189 },
	{ CCI_REG16(0x6f12), 0x6289 },
	{ CCI_REG16(0x6f12), 0x1144 },
	{ CCI_REG16(0x6f12), 0x0844 },
	{ CCI_REG16(0x6f12), 0x2082 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x73f9 },
	{ CCI_REG16(0x6f12), 0xe080 },
	{ CCI_REG16(0x6f12), 0xb5f8 },
	{ CCI_REG16(0x6f12), 0x3616 },
	{ CCI_REG16(0x6f12), 0x4ff2 },
	{ CCI_REG16(0x6f12), 0x3c05 },
	{ CCI_REG16(0x6f12), 0x081a },
	{ CCI_REG16(0x6f12), 0x80b2 },
	{ CCI_REG16(0x6f12), 0x2081 },
	{ CCI_REG16(0x6f12), 0x218a },
	{ CCI_REG16(0x6f12), 0x411a },
	{ CCI_REG16(0x6f12), 0x0029 },
	{ CCI_REG16(0x6f12), 0x1edd },
	{ CCI_REG16(0x6f12), 0x88b2 },
	{ CCI_REG16(0x6f12), 0x6082 },
	{ CCI_REG16(0x6f12), 0xe188 },
	{ CCI_REG16(0x6f12), 0x464e },
	{ CCI_REG16(0x6f12), 0x0844 },
	{ CCI_REG16(0x6f12), 0x7061 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x64f9 },
	{ CCI_REG16(0x6f12), 0x7169 },
	{ CCI_REG16(0x6f12), 0xc140 },
	{ CCI_REG16(0x6f12), 0x2846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x64f9 },
	{ CCI_REG16(0x6f12), 0xa089 },
	{ CCI_REG16(0x6f12), 0xf18a },
	{ CCI_REG16(0x6f12), 0x0844 },
	{ CCI_REG16(0x6f12), 0xe082 },
	{ CCI_REG16(0x6f12), 0x98f8 },
	{ CCI_REG16(0x6f12), 0x8100 },
	{ CCI_REG16(0x6f12), 0x98f8 },
	{ CCI_REG16(0x6f12), 0x8210 },
	{ CCI_REG16(0x6f12), 0x40ea },
	{ CCI_REG16(0x6f12), 0x4100 },
	{ CCI_REG16(0x6f12), 0x98f8 },
	{ CCI_REG16(0x6f12), 0x8310 },
	{ CCI_REG16(0x6f12), 0x40ea },
	{ CCI_REG16(0x6f12), 0x8100 },
	{ CCI_REG16(0x6f12), 0x3d49 },
	{ CCI_REG16(0x6f12), 0x0883 },
	{ CCI_REG16(0x6f12), 0x0120 },
	{ CCI_REG16(0x6f12), 0x05e0 },
	{ CCI_REG16(0x6f12), 0x4ff0 },
	{ CCI_REG16(0x6f12), 0xff31 },
	{ CCI_REG16(0x6f12), 0x2846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x4df9 },
	{ CCI_REG16(0x6f12), 0x4846 },
	{ CCI_REG16(0x6f12), 0x2070 },
	{ CCI_REG16(0x6f12), 0x2078 },
	{ CCI_REG16(0x6f12), 0x5946 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0xf05f },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x4ab9 },
	{ CCI_REG16(0x6f12), 0x70b5 },
	{ CCI_REG16(0x6f12), 0x2d48 },
	{ CCI_REG16(0x6f12), 0x324c },
	{ CCI_REG16(0x6f12), 0x4ff4 },
	{ CCI_REG16(0x6f12), 0xd446 },
	{ CCI_REG16(0x6f12), 0x90f8 },
	{ CCI_REG16(0x6f12), 0x9a07 },
	{ CCI_REG16(0x6f12), 0x2034 },
	{ CCI_REG16(0x6f12), 0x4ff2 },
	{ CCI_REG16(0x6f12), 0x3c05 },
	{ CCI_REG16(0x6f12), 0x80b1 },
	{ CCI_REG16(0x6f12), 0x0220 },
	{ CCI_REG16(0x6f12), 0x2070 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x31f9 },
	{ CCI_REG16(0x6f12), 0x2a4a },
	{ CCI_REG16(0x6f12), 0xa189 },
	{ CCI_REG16(0x6f12), 0x5269 },
	{ CCI_REG16(0x6f12), 0x1144 },
	{ CCI_REG16(0x6f12), 0xc140 },
	{ CCI_REG16(0x6f12), 0x2846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x2ef9 },
	{ CCI_REG16(0x6f12), 0x3046 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0x7040 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x33b9 },
	{ CCI_REG16(0x6f12), 0x4ff0 },
	{ CCI_REG16(0x6f12), 0xff31 },
	{ CCI_REG16(0x6f12), 0x2846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x24f9 },
	{ CCI_REG16(0x6f12), 0x2249 },
	{ CCI_REG16(0x6f12), 0x0320 },
	{ CCI_REG16(0x6f12), 0x81f8 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0xf0e7 },
	{ CCI_REG16(0x6f12), 0x2de9 },
	{ CCI_REG16(0x6f12), 0xf047 },
	{ CCI_REG16(0x6f12), 0x1f4c },
	{ CCI_REG16(0x6f12), 0x0027 },
	{ CCI_REG16(0x6f12), 0x2034 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x0ef9 },
	{ CCI_REG16(0x6f12), 0xdff8 },
	{ CCI_REG16(0x6f12), 0x7090 },
	{ CCI_REG16(0x6f12), 0x2085 },
	{ CCI_REG16(0x6f12), 0x4846 },
	{ CCI_REG16(0x6f12), 0x10f8 },
	{ CCI_REG16(0x6f12), 0x201f },
	{ CCI_REG16(0x6f12), 0x84f8 },
	{ CCI_REG16(0x6f12), 0x2a10 },
	{ CCI_REG16(0x6f12), 0x0078 },
	{ CCI_REG16(0x6f12), 0x134e },
	{ CCI_REG16(0x6f12), 0x0128 },
	{ CCI_REG16(0x6f12), 0x02d0 },
	{ CCI_REG16(0x6f12), 0x0228 },
	{ CCI_REG16(0x6f12), 0x61d1 },
	{ CCI_REG16(0x6f12), 0x0ae0 },
	{ CCI_REG16(0x6f12), 0xe188 },
	{ CCI_REG16(0x6f12), 0xd6f8 },
	{ CCI_REG16(0x6f12), 0x3406 },
	{ CCI_REG16(0x6f12), 0x401a },
	{ CCI_REG16(0x6f12), 0x218a },
	{ CCI_REG16(0x6f12), 0x471a },
	{ CCI_REG16(0x6f12), 0xb6f8 },
	{ CCI_REG16(0x6f12), 0x7603 },
	{ CCI_REG16(0x6f12), 0x7f1c },
	{ CCI_REG16(0x6f12), 0x401e },
	{ CCI_REG16(0x6f12), 0x4743 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x0bf9 },
	{ CCI_REG16(0x6f12), 0xdff8 },
	{ CCI_REG16(0x6f12), 0x3c80 },
	{ CCI_REG16(0x6f12), 0x8246 },
	{ CCI_REG16(0x6f12), 0x316c },
	{ CCI_REG16(0x6f12), 0xb8f8 },
	{ CCI_REG16(0x6f12), 0x7e00 },
	{ CCI_REG16(0x6f12), 0x0025 },
	{ CCI_REG16(0x6f12), 0xa1fb },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0x2a46 },
	{ CCI_REG16(0x6f12), 0x4ff4 },
	{ CCI_REG16(0x6f12), 0x7a73 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xdff8 },
	{ CCI_REG16(0x6f12), 0xc819 },
	{ CCI_REG16(0x6f12), 0xa061 },
	{ CCI_REG16(0x6f12), 0x12e0 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x12b0 },
	{ CCI_REG16(0x6f12), 0x2001 },
	{ CCI_REG16(0x6f12), 0x8400 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x6f12), 0x6000 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x0c10 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x12e0 },
	{ CCI_REG16(0x6f12), 0x2001 },
	{ CCI_REG16(0x6f12), 0xdc70 },
	{ CCI_REG16(0x6f12), 0x2001 },
	{ CCI_REG16(0x6f12), 0x9040 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x2110 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x6f12), 0x6a00 },
	{ CCI_REG16(0x6f12), 0xb6f8 },
	{ CCI_REG16(0x6f12), 0x7613 },
	{ CCI_REG16(0x6f12), 0x401e },
	{ CCI_REG16(0x6f12), 0x491c },
	{ CCI_REG16(0x6f12), 0xb0fb },
	{ CCI_REG16(0x6f12), 0xf1f0 },
	{ CCI_REG16(0x6f12), 0x2084 },
	{ CCI_REG16(0x6f12), 0x98f8 },
	{ CCI_REG16(0x6f12), 0x8000 },
	{ CCI_REG16(0x6f12), 0xa169 },
	{ CCI_REG16(0x6f12), 0x0144 },
	{ CCI_REG16(0x6f12), 0xb1fb },
	{ CCI_REG16(0x6f12), 0xf0f0 },
	{ CCI_REG16(0x6f12), 0xa061 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xd7f8 },
	{ CCI_REG16(0x6f12), 0x4946 },
	{ CCI_REG16(0x6f12), 0x03e0 },
	{ CCI_REG16(0x6f12), 0xa269 },
	{ CCI_REG16(0x6f12), 0x6d1c },
	{ CCI_REG16(0x6f12), 0xaa42 },
	{ CCI_REG16(0x6f12), 0x03d9 },
	{ CCI_REG16(0x6f12), 0x91f8 },
	{ CCI_REG16(0x6f12), 0x2020 },
	{ CCI_REG16(0x6f12), 0x032a },
	{ CCI_REG16(0x6f12), 0xf7d1 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xd0f8 },
	{ CCI_REG16(0x6f12), 0xb6f8 },
	{ CCI_REG16(0x6f12), 0x7613 },
	{ CCI_REG16(0x6f12), 0x401e },
	{ CCI_REG16(0x6f12), 0x491c },
	{ CCI_REG16(0x6f12), 0xb0fb },
	{ CCI_REG16(0x6f12), 0xf1f0 },
	{ CCI_REG16(0x6f12), 0x6084 },
	{ CCI_REG16(0x6f12), 0xa069 },
	{ CCI_REG16(0x6f12), 0xa842 },
	{ CCI_REG16(0x6f12), 0x04d8 },
	{ CCI_REG16(0x6f12), 0x0021 },
	{ CCI_REG16(0x6f12), 0x40f6 },
	{ CCI_REG16(0x6f12), 0x1460 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xc6f8 },
	{ CCI_REG16(0x6f12), 0x5046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xbef8 },
	{ CCI_REG16(0x6f12), 0x8008 },
	{ CCI_REG16(0x6f12), 0xe061 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0xf047 },
	{ CCI_REG16(0x6f12), 0x4ff0 },
	{ CCI_REG16(0x6f12), 0xff31 },
	{ CCI_REG16(0x6f12), 0x4ff2 },
	{ CCI_REG16(0x6f12), 0x3c00 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xa0b8 },
	{ CCI_REG16(0x6f12), 0x70b5 },
	{ CCI_REG16(0x6f12), 0x1d4c },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0x9b41 },
	{ CCI_REG16(0x6f12), 0x236a },
	{ CCI_REG16(0x6f12), 0x1b48 },
	{ CCI_REG16(0x6f12), 0x9847 },
	{ CCI_REG16(0x6f12), 0x1b4d },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x2860 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0x2741 },
	{ CCI_REG16(0x6f12), 0x236a },
	{ CCI_REG16(0x6f12), 0x1948 },
	{ CCI_REG16(0x6f12), 0x9847 },
	{ CCI_REG16(0x6f12), 0x6860 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0x2931 },
	{ CCI_REG16(0x6f12), 0x236a },
	{ CCI_REG16(0x6f12), 0x1748 },
	{ CCI_REG16(0x6f12), 0x9847 },
	{ CCI_REG16(0x6f12), 0xa860 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0xdf21 },
	{ CCI_REG16(0x6f12), 0x236a },
	{ CCI_REG16(0x6f12), 0x1448 },
	{ CCI_REG16(0x6f12), 0x9847 },
	{ CCI_REG16(0x6f12), 0xe860 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0xa721 },
	{ CCI_REG16(0x6f12), 0x236a },
	{ CCI_REG16(0x6f12), 0x1248 },
	{ CCI_REG16(0x6f12), 0x9847 },
	{ CCI_REG16(0x6f12), 0x2861 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0x9b21 },
	{ CCI_REG16(0x6f12), 0x236a },
	{ CCI_REG16(0x6f12), 0x0f48 },
	{ CCI_REG16(0x6f12), 0x9847 },
	{ CCI_REG16(0x6f12), 0xa861 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0xa911 },
	{ CCI_REG16(0x6f12), 0x236a },
	{ CCI_REG16(0x6f12), 0x0d48 },
	{ CCI_REG16(0x6f12), 0x9847 },
	{ CCI_REG16(0x6f12), 0xe861 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0x6911 },
	{ CCI_REG16(0x6f12), 0x236a },
	{ CCI_REG16(0x6f12), 0x0a48 },
	{ CCI_REG16(0x6f12), 0x9847 },
	{ CCI_REG16(0x6f12), 0x2862 },
	{ CCI_REG16(0x6f12), 0x70bd },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x8a80 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x9c21 },
	{ CCI_REG16(0x6f12), 0x2001 },
	{ CCI_REG16(0x6f12), 0xdc70 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x8907 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x4305 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x11c9 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x7d6f },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x98cb },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x99cb },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x9b1d },
	{ CCI_REG16(0x6f12), 0x4bf2 },
	{ CCI_REG16(0x6f12), 0x390c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x48f6 },
	{ CCI_REG16(0x6f12), 0x8b5c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x44f6 },
	{ CCI_REG16(0x6f12), 0x9b6c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x4af6 },
	{ CCI_REG16(0x6f12), 0xcf4c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x4bf2 },
	{ CCI_REG16(0x6f12), 0x550c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x48f6 },
	{ CCI_REG16(0x6f12), 0xff0c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x46f6 },
	{ CCI_REG16(0x6f12), 0x2b2c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x020c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x4bf2 },
	{ CCI_REG16(0x6f12), 0x633c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x47f2 },
	{ CCI_REG16(0x6f12), 0x2d4c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x44f2 },
	{ CCI_REG16(0x6f12), 0x053c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x41f2 },
	{ CCI_REG16(0x6f12), 0xc91c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x49f2 },
	{ CCI_REG16(0x6f12), 0x374c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x020c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x47f2 },
	{ CCI_REG16(0x6f12), 0x032c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x4cf2 },
	{ CCI_REG16(0x6f12), 0x713c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x020c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x47f2 },
	{ CCI_REG16(0x6f12), 0xa52c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x020c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x49f6 },
	{ CCI_REG16(0x6f12), 0x252c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x020c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x4bf2 },
	{ CCI_REG16(0x6f12), 0x990c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x49f6 },
	{ CCI_REG16(0x6f12), 0x2f0c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x49f6 },
	{ CCI_REG16(0x6f12), 0x7b0c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x4af6 },
	{ CCI_REG16(0x6f12), 0x6f4c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x4af6 },
	{ CCI_REG16(0x6f12), 0xa34c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x45f6 },
	{ CCI_REG16(0x6f12), 0xed6c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x0842 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x03b1 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x13be },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x2a04 },
	{ CCI_REG16(0x6f12), 0x5608 },
	{ CCI_REG16(0x602a), 0x31ea },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x422a },
	{ CCI_REG16(0x6f12), 0x0008 },
	{ CCI_REG16(0x6f12), 0x0808 },
	{ CCI_REG16(0x6f12), 0x0808 },
	{ CCI_REG16(0x6f12), 0x0808 },
	{ CCI_REG16(0x6f12), 0x0801 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x4208 },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x6f12), 0x0c8d },
	{ CCI_REG16(0x602a), 0x219a },
	{ CCI_REG16(0x6f12), 0x6401 },
	{ CCI_REG16(0x602a), 0x2b96 },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x602a), 0x2bfc },
	{ CCI_REG16(0x6f12), 0x033f },
	{ CCI_REG16(0x6f12), 0x033f },
	{ CCI_REG16(0x6f12), 0x033f },
	{ CCI_REG16(0x6f12), 0x033f },
	{ CCI_REG16(0x6f12), 0x033f },
	{ CCI_REG16(0x6f12), 0x033f },
	{ CCI_REG16(0x6f12), 0x033f },
	{ CCI_REG16(0x6f12), 0x033f },
	{ CCI_REG16(0x6f12), 0x033f },
	{ CCI_REG16(0x6f12), 0x033f },
	{ CCI_REG16(0x6f12), 0x033f },
	{ CCI_REG16(0x6f12), 0x033f },
	{ CCI_REG16(0x6f12), 0x033f },
	{ CCI_REG16(0x6f12), 0x033f },
	{ CCI_REG16(0x6f12), 0x033f },
	{ CCI_REG16(0x6f12), 0x033f },
	{ CCI_REG16(0x6f12), 0x033f },
	{ CCI_REG16(0x6f12), 0x033f },
	{ CCI_REG16(0x6f12), 0x033f },
	{ CCI_REG16(0x6f12), 0x033f },
	{ CCI_REG16(0x602a), 0x31c8 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x2a00 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x0480 },
	{ CCI_REG16(0x602a), 0x29f8 },
	{ CCI_REG16(0x6f12), 0x090f },
	{ CCI_REG16(0x602a), 0x29f2 },
	{ CCI_REG16(0x6f12), 0x0305 },
	{ CCI_REG16(0x602a), 0x29fa },
	{ CCI_REG16(0x6f12), 0x0203 },
	{ CCI_REG16(0x602a), 0x5ac4 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x5b30 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5b18 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x6ca6 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x6c96 },
	{ CCI_REG16(0x6f12), 0x04b1 },
	{ CCI_REG16(0x6f12), 0x04b1 },
	{ CCI_REG16(0x6f12), 0x04b1 },
	{ CCI_REG16(0x6f12), 0x04b1 },
	{ CCI_REG16(0x6f12), 0x04b1 },
	{ CCI_REG16(0x6f12), 0x04b1 },
	{ CCI_REG16(0x602a), 0x6b16 },
	{ CCI_REG16(0x6f12), 0x0021 },
	{ CCI_REG16(0x6f12), 0x0021 },
	{ CCI_REG16(0x6f12), 0x0021 },
	{ CCI_REG16(0x6f12), 0x0021 },
	{ CCI_REG16(0x6f12), 0x0021 },
	{ CCI_REG16(0x6f12), 0x0021 },
	{ CCI_REG16(0x602a), 0x6b06 },
	{ CCI_REG16(0x6f12), 0x0015 },
	{ CCI_REG16(0x6f12), 0x0015 },
	{ CCI_REG16(0x6f12), 0x0015 },
	{ CCI_REG16(0x6f12), 0x0015 },
	{ CCI_REG16(0x6f12), 0x0015 },
	{ CCI_REG16(0x6f12), 0x0015 },
	{ CCI_REG16(0x602a), 0x6b36 },
	{ CCI_REG16(0x6f12), 0x004c },
	{ CCI_REG16(0x6f12), 0x004c },
	{ CCI_REG16(0x6f12), 0x004c },
	{ CCI_REG16(0x6f12), 0x004c },
	{ CCI_REG16(0x6f12), 0x004c },
	{ CCI_REG16(0x6f12), 0x004c },
	{ CCI_REG16(0x602a), 0x6b26 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x6af6 },
	{ CCI_REG16(0x6f12), 0x06fa },
	{ CCI_REG16(0x6f12), 0x06fa },
	{ CCI_REG16(0x6f12), 0x06fa },
	{ CCI_REG16(0x6f12), 0x06fa },
	{ CCI_REG16(0x6f12), 0x06fa },
	{ CCI_REG16(0x6f12), 0x06fa },
	{ CCI_REG16(0x602a), 0x6ae6 },
	{ CCI_REG16(0x6f12), 0x1753 },
	{ CCI_REG16(0x6f12), 0x1753 },
	{ CCI_REG16(0x6f12), 0x1753 },
	{ CCI_REG16(0x6f12), 0x1753 },
	{ CCI_REG16(0x6f12), 0x1753 },
	{ CCI_REG16(0x6f12), 0x1753 },
	{ CCI_REG16(0x602a), 0x6bd6 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x6bc6 },
	{ CCI_REG16(0x6f12), 0x001b },
	{ CCI_REG16(0x6f12), 0x001b },
	{ CCI_REG16(0x6f12), 0x001b },
	{ CCI_REG16(0x6f12), 0x001b },
	{ CCI_REG16(0x6f12), 0x001b },
	{ CCI_REG16(0x6f12), 0x001b },
	{ CCI_REG16(0x602a), 0x6bf6 },
	{ CCI_REG16(0x6f12), 0x0027 },
	{ CCI_REG16(0x6f12), 0x0027 },
	{ CCI_REG16(0x6f12), 0x0027 },
	{ CCI_REG16(0x6f12), 0x0027 },
	{ CCI_REG16(0x6f12), 0x0027 },
	{ CCI_REG16(0x6f12), 0x0027 },
	{ CCI_REG16(0x602a), 0x6be6 },
	{ CCI_REG16(0x6f12), 0x0075 },
	{ CCI_REG16(0x6f12), 0x0075 },
	{ CCI_REG16(0x6f12), 0x0075 },
	{ CCI_REG16(0x6f12), 0x0075 },
	{ CCI_REG16(0x6f12), 0x0075 },
	{ CCI_REG16(0x6f12), 0x0075 },
	{ CCI_REG16(0x602a), 0x6bb6 },
	{ CCI_REG16(0x6f12), 0x0054 },
	{ CCI_REG16(0x6f12), 0x0054 },
	{ CCI_REG16(0x6f12), 0x0054 },
	{ CCI_REG16(0x6f12), 0x0054 },
	{ CCI_REG16(0x6f12), 0x0054 },
	{ CCI_REG16(0x6f12), 0x0054 },
	{ CCI_REG16(0x602a), 0x6ba6 },
	{ CCI_REG16(0x6f12), 0x0579 },
	{ CCI_REG16(0x6f12), 0x0579 },
	{ CCI_REG16(0x6f12), 0x0579 },
	{ CCI_REG16(0x6f12), 0x0579 },
	{ CCI_REG16(0x6f12), 0x0579 },
	{ CCI_REG16(0x6f12), 0x0579 },
	{ CCI_REG16(0x602a), 0x6b76 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x602a), 0x6b66 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x6b96 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x6b86 },
	{ CCI_REG16(0x6f12), 0x007d },
	{ CCI_REG16(0x6f12), 0x007d },
	{ CCI_REG16(0x6f12), 0x007d },
	{ CCI_REG16(0x6f12), 0x007d },
	{ CCI_REG16(0x6f12), 0x007d },
	{ CCI_REG16(0x6f12), 0x007d },
	{ CCI_REG16(0x602a), 0x6b56 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x6b46 },
	{ CCI_REG16(0x6f12), 0x0082 },
	{ CCI_REG16(0x6f12), 0x0082 },
	{ CCI_REG16(0x6f12), 0x0082 },
	{ CCI_REG16(0x6f12), 0x0082 },
	{ CCI_REG16(0x6f12), 0x0082 },
	{ CCI_REG16(0x6f12), 0x0082 },
	{ CCI_REG16(0x602a), 0x6c36 },
	{ CCI_REG16(0x6f12), 0x00b0 },
	{ CCI_REG16(0x6f12), 0x00b0 },
	{ CCI_REG16(0x6f12), 0x00b0 },
	{ CCI_REG16(0x6f12), 0x00b0 },
	{ CCI_REG16(0x6f12), 0x00b0 },
	{ CCI_REG16(0x6f12), 0x00b0 },
	{ CCI_REG16(0x602a), 0x6c46 },
	{ CCI_REG16(0x6f12), 0x0025 },
	{ CCI_REG16(0x6f12), 0x0025 },
	{ CCI_REG16(0x6f12), 0x0025 },
	{ CCI_REG16(0x6f12), 0x0025 },
	{ CCI_REG16(0x6f12), 0x0025 },
	{ CCI_REG16(0x6f12), 0x0025 },
	{ CCI_REG16(0x602a), 0x5b3c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5b48 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5b42 },
	{ CCI_REG16(0x6f12), 0x012c },
	{ CCI_REG16(0x602a), 0x5b4e },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5b5a },
	{ CCI_REG16(0x6f12), 0x0003 },
	{ CCI_REG16(0x602a), 0x5b54 },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x602a), 0x5b66 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x5b60 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x6c76 },
	{ CCI_REG16(0x6f12), 0x0fff },
	{ CCI_REG16(0x6f12), 0x0fff },
	{ CCI_REG16(0x6f12), 0x0fff },
	{ CCI_REG16(0x6f12), 0x0fff },
	{ CCI_REG16(0x6f12), 0x0fff },
	{ CCI_REG16(0x6f12), 0x0fff },
	{ CCI_REG16(0x602a), 0x6c86 },
	{ CCI_REG16(0x6f12), 0x0fff },
	{ CCI_REG16(0x6f12), 0x0fff },
	{ CCI_REG16(0x6f12), 0x0fff },
	{ CCI_REG16(0x6f12), 0x0fff },
	{ CCI_REG16(0x6f12), 0x0fff },
	{ CCI_REG16(0x6f12), 0x0fff },
	{ CCI_REG16(0x602a), 0x6c56 },
	{ CCI_REG16(0x6f12), 0x0176 },
	{ CCI_REG16(0x6f12), 0x0176 },
	{ CCI_REG16(0x6f12), 0x0176 },
	{ CCI_REG16(0x6f12), 0x0176 },
	{ CCI_REG16(0x6f12), 0x0176 },
	{ CCI_REG16(0x6f12), 0x0176 },
	{ CCI_REG16(0x602a), 0x6c66 },
	{ CCI_REG16(0x6f12), 0x0025 },
	{ CCI_REG16(0x6f12), 0x0025 },
	{ CCI_REG16(0x6f12), 0x0025 },
	{ CCI_REG16(0x6f12), 0x0025 },
	{ CCI_REG16(0x6f12), 0x0025 },
	{ CCI_REG16(0x6f12), 0x0025 },
	{ CCI_REG16(0x602a), 0x6ad6 },
	{ CCI_REG16(0x6f12), 0x0190 },
	{ CCI_REG16(0x6f12), 0x0190 },
	{ CCI_REG16(0x6f12), 0x0190 },
	{ CCI_REG16(0x6f12), 0x0190 },
	{ CCI_REG16(0x6f12), 0x0190 },
	{ CCI_REG16(0x6f12), 0x0190 },
	{ CCI_REG16(0x602a), 0x5a40 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x5b36 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5b78 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5b7e },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x5b1e },
	{ CCI_REG16(0x6f12), 0x01b3 },
	{ CCI_REG16(0x602a), 0x5b24 },
	{ CCI_REG16(0x6f12), 0x0363 },
	{ CCI_REG16(0x602a), 0x5b2a },
	{ CCI_REG16(0x6f12), 0x0007 },
	{ CCI_REG16(0x602a), 0x5b0c },
	{ CCI_REG16(0x6f12), 0x0333 },
	{ CCI_REG16(0x602a), 0x5b12 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5b06 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5afa },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5b00 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5af4 },
	{ CCI_REG16(0x6f12), 0x00fc },
	{ CCI_REG16(0x602a), 0x6c16 },
	{ CCI_REG16(0x6f12), 0x03ec },
	{ CCI_REG16(0x6f12), 0x03ec },
	{ CCI_REG16(0x6f12), 0x03ec },
	{ CCI_REG16(0x6f12), 0x03ec },
	{ CCI_REG16(0x6f12), 0x03ec },
	{ CCI_REG16(0x6f12), 0x03ec },
	{ CCI_REG16(0x602a), 0x5b72 },
	{ CCI_REG16(0x6f12), 0x0007 },
	{ CCI_REG16(0x602a), 0x5b8a },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x6c26 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x6c06 },
	{ CCI_REG16(0x6f12), 0x035d },
	{ CCI_REG16(0x6f12), 0x035d },
	{ CCI_REG16(0x6f12), 0x035d },
	{ CCI_REG16(0x6f12), 0x035d },
	{ CCI_REG16(0x6f12), 0x035d },
	{ CCI_REG16(0x6f12), 0x035d },
	{ CCI_REG16(0x602a), 0x5ae8 },
	{ CCI_REG16(0x6f12), 0x03fc },
	{ CCI_REG16(0x602a), 0x5aee },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5ae2 },
	{ CCI_REG16(0x6f12), 0x012c },
	{ CCI_REG16(0x602a), 0x5ad6 },
	{ CCI_REG16(0x6f12), 0x1984 },
	{ CCI_REG16(0x602a), 0x5adc },
	{ CCI_REG16(0x6f12), 0x3ffd },
	{ CCI_REG16(0x602a), 0x5ad0 },
	{ CCI_REG16(0x6f12), 0x0035 },
	{ CCI_REG16(0x602a), 0x5ab8 },
	{ CCI_REG16(0x6f12), 0x014c },
	{ CCI_REG16(0x602a), 0x5abe },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x5ab2 },
	{ CCI_REG16(0x6f12), 0x0009 },
	{ CCI_REG16(0x602a), 0x5aa6 },
	{ CCI_REG16(0x6f12), 0x3ffe },
	{ CCI_REG16(0x602a), 0x5aac },
	{ CCI_REG16(0x6f12), 0x00a2 },
	{ CCI_REG16(0x602a), 0x5aa0 },
	{ CCI_REG16(0x6f12), 0x184f },
	{ CCI_REG16(0x602a), 0x5a8e },
	{ CCI_REG16(0x6f12), 0x01d7 },
	{ CCI_REG16(0x602a), 0x5a94 },
	{ CCI_REG16(0x6f12), 0x03ff },
	{ CCI_REG16(0x602a), 0x5a88 },
	{ CCI_REG16(0x6f12), 0x03ff },
	{ CCI_REG16(0x602a), 0x5a7c },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x602a), 0x5a82 },
	{ CCI_REG16(0x6f12), 0x3ffe },
	{ CCI_REG16(0x602a), 0x5a76 },
	{ CCI_REG16(0x6f12), 0x1ed7 },
	{ CCI_REG16(0x602a), 0x5a64 },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x602a), 0x5a6a },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5a5e },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5a52 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5a58 },
	{ CCI_REG16(0x6f12), 0x3e68 },
	{ CCI_REG16(0x602a), 0x5a4c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5b84 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5b72 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x2186 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x1310 },
	{ CCI_REG16(0x6f12), 0x0f00 },
	{ CCI_REG16(0x602a), 0x2188 },
	{ CCI_REG16(0x6f12), 0x0650 },
	{ CCI_REG16(0x602a), 0x218e },
	{ CCI_REG16(0x6f12), 0x0650 },
	{ CCI_REG16(0x602a), 0x2194 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x3450 },
	{ CCI_REG16(0x6f12), 0x193c },
	{ CCI_REG16(0x602a), 0x43c8 },
	{ CCI_REG16(0x6f12), 0x0014 },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x6f12), 0x003a },
	{ CCI_REG16(0x6f12), 0x0033 },
	{ CCI_REG16(0x6f12), 0x0003 },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x602a), 0x43da },
	{ CCI_REG16(0x6f12), 0x19a0 },
	{ CCI_REG16(0x6f12), 0x1348 },
	{ CCI_REG16(0x6f12), 0x0138 },
	{ CCI_REG16(0x6f12), 0x0140 },
	{ CCI_REG16(0x6f12), 0x0110 },
	{ CCI_REG16(0x6f12), 0x011c },
	{ CCI_REG16(0x6f12), 0x012c },
	{ CCI_REG16(0x6f12), 0x013c },
	{ CCI_REG16(0x6f12), 0x0150 },
	{ CCI_REG16(0x6f12), 0x0168 },
	{ CCI_REG16(0x6f12), 0x0180 },
	{ CCI_REG16(0x6f12), 0x019c },
	{ CCI_REG16(0x6f12), 0x01bc },
	{ CCI_REG16(0x6f12), 0x01e4 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x012c },
	{ CCI_REG16(0x6f12), 0x0144 },
	{ CCI_REG16(0x6f12), 0x0160 },
	{ CCI_REG16(0x6f12), 0x0180 },
	{ CCI_REG16(0x6f12), 0x01a8 },
	{ CCI_REG16(0x6f12), 0x0214 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x0fe8), 0x4180 },
};

/* g0: full-res 6528x4896 (FLL 5128 / LLP 20592, 15fps) */
static const struct cci_reg_sequence s5kgd2_mode_6528x4896_regs[] = {
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x2a08 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0xa5e0 },
	{ CCI_REG16(0x602a), 0x21f8 },
	{ CCI_REG16(0x6f12), 0x1358 },
	{ CCI_REG16(0x602a), 0x30ce },
	{ CCI_REG16(0x6f12), 0x01e4 },
	{ CCI_REG16(0x602a), 0x8a00 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x3770 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x220e },
	{ CCI_REG16(0x6f12), 0x0009 },
	{ CCI_REG16(0x6f12), 0x0009 },
	{ CCI_REG16(0x602a), 0x31cc },
	{ CCI_REG16(0x6f12), 0x0019 },
	{ CCI_REG16(0x602a), 0x31d2 },
	{ CCI_REG16(0x6f12), 0x0012 },
	{ CCI_REG16(0x602a), 0x31ca },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x602a), 0x221c },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x2a18 },
	{ CCI_REG16(0x6f12), 0x0801 },
	{ CCI_REG16(0x602a), 0x5090 },
	{ CCI_REG16(0x6f12), 0x0600 },
	{ CCI_REG16(0x602a), 0x84f8 },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x602a), 0x8508 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5530 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x43b0 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x43d4 },
	{ CCI_REG16(0x6f12), 0x1300 },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x6f12), 0x13e8 },
	{ CCI_REG16(0x602a), 0x2a0e },
	{ CCI_REG16(0x6f12), 0x1fcd },
	{ CCI_REG16(0x602a), 0x1346 },
	{ CCI_REG16(0x6f12), 0x0018 },
	{ CCI_REG16(0x602a), 0x2308 },
	{ CCI_REG16(0x6f12), 0x005b },
	{ CCI_REG16(0x602a), 0x381a },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x3fff },
	{ CCI_REG16(0x602a), 0x3812 },
	{ CCI_REG16(0x6f12), 0x0003 },
	{ CCI_REG16(0x602a), 0x345e },
	{ CCI_REG16(0x6f12), 0x1f38 },
	{ CCI_REG16(0x6f12), 0x1f3f },
	{ CCI_REG16(0x602a), 0x3b1a },
	{ CCI_REG16(0x6f12), 0x0009 },
	{ CCI_REG16(0x6f12), 0x0009 },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x0009 },
	{ CCI_REG16(0x6f12), 0x0009 },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x0009 },
	{ CCI_REG16(0x6f12), 0x0009 },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x0009 },
	{ CCI_REG16(0x6f12), 0x0009 },
	{ CCI_REG16(0x602a), 0x3a9a },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0xffe0 },
	{ CCI_REG16(0x6f12), 0xffe0 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0xffe0 },
	{ CCI_REG16(0x6f12), 0xffe0 },
	{ CCI_REG16(0x6f12), 0xffe0 },
	{ CCI_REG16(0x6f12), 0xffe0 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0xffe0 },
	{ CCI_REG16(0x6f12), 0xffe0 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x602a), 0x39ee },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x6f12), 0x1020 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x2a1e },
	{ CCI_REG16(0x6f12), 0x0108 },
	{ CCI_REG16(0x602a), 0x134c },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x602a), 0x30b8 },
	{ CCI_REG16(0x6f12), 0xd020 },
	{ CCI_REG16(0x602a), 0x2580 },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x602a), 0x2584 },
	{ CCI_REG16(0x6f12), 0x0226 },
	{ CCI_REG16(0x602a), 0x8a10 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x8a6e },
	{ CCI_REG16(0x6f12), 0x1f38 },
	{ CCI_REG16(0x6f12), 0x1f3f },
	{ CCI_REG16(0x602a), 0x8a20 },
	{ CCI_REG16(0x6f12), 0x0008 },
	{ CCI_REG16(0x602a), 0x8a26 },
	{ CCI_REG16(0x6f12), 0x0008 },
	{ CCI_REG16(0x602a), 0x8a1e },
	{ CCI_REG16(0x6f12), 0x0080 },
	{ CCI_REG16(0x602a), 0x8a68 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x8a3e },
	{ CCI_REG16(0x6f12), 0x03ff },
	{ CCI_REG16(0x602a), 0x8a44 },
	{ CCI_REG16(0x6f12), 0x0080 },
	{ CCI_REG16(0x602a), 0x1370 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x602a), 0x1378 },
	{ CCI_REG16(0x6f12), 0x0084 },
	{ CCI_REG16(0x602a), 0x2134 },
	{ CCI_REG16(0x6f12), 0x00c6 },
	{ CCI_REG16(0x602a), 0x1fc0 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x005f },
	{ CCI_REG16(0x602a), 0x1fc6 },
	{ CCI_REG16(0x6f12), 0x0060 },
	{ CCI_REG16(0x602a), 0x1fcc },
	{ CCI_REG16(0x6f12), 0x1c80 },
	{ CCI_REG16(0x6f12), 0x1c82 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x6f12), 0xf4b2 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0xf484), 0x0080 },
	{ CCI_REG16(0xf498), 0x06a3 },
	{ CCI_REG16(0xf496), 0x0200 },
	{ CCI_REG16(0xf4b2), 0x1c82 },
	{ CCI_REG16(0xf41c), 0x4916 },
	{ CCI_REG16(0xf418), 0x001f },
	{ CCI_REG16(0xb606), 0x0300 },
	{ CCI_REG16(0xf482), 0x800d },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x623c), 0x8080 },
	{ CCI_REG16(0x623e), 0x830d },
	{ CCI_REG16(0x6240), 0x0000 },
	{ CCI_REG16(0xf4a6), 0x001d },
	{ CCI_REG16(0x0344), 0x0018 },
	{ CCI_REG16(0x0346), 0x0018 },
	{ CCI_REG16(0x0348), 0x1997 },
	{ CCI_REG16(0x034a), 0x133f },
	{ CCI_REG16(0x034c), 0x1980 },
	{ CCI_REG16(0x034e), 0x1320 },
	{ CCI_REG16(0x0350), 0x0000 },
	{ CCI_REG16(0x0352), 0x0004 },
	{ CCI_REG16(0x0900), 0x0011 },
	{ CCI_REG16(0x0380), 0x0001 },
	{ CCI_REG16(0x0382), 0x0001 },
	{ CCI_REG16(0x0384), 0x0001 },
	{ CCI_REG16(0x0386), 0x0001 },
	{ CCI_REG16(0x0400), 0x1010 },
	{ CCI_REG16(0x0408), 0x0100 },
	{ CCI_REG16(0x040a), 0x0100 },
	{ CCI_REG16(0x040c), 0x0000 },
	{ CCI_REG16(0x0110), 0x1002 },
	{ CCI_REG16(0x0114), 0x0300 },
	{ CCI_REG16(0x0116), 0x3000 },
	{ CCI_REG16(0x0118), 0x0001 },
	{ CCI_REG16(0x011a), 0x0000 },
	{ CCI_REG16(0x011c), 0x0000 },
	{ CCI_REG16(0x0136), 0x1300 },
	{ CCI_REG16(0x013e), 0x00c8 },
	{ CCI_REG16(0x0300), 0x0007 },
	{ CCI_REG16(0x0302), 0x0001 },
	{ CCI_REG16(0x0304), 0x0002 },
	{ CCI_REG16(0x0306), 0x0091 },
	{ CCI_REG16(0x0308), 0x000b },
	{ CCI_REG16(0x030a), 0x0001 },
	{ CCI_REG16(0x030c), 0x0000 },
	{ CCI_REG16(0x030e), 0x0002 },
	{ CCI_REG16(0x0310), 0x00bc },
	{ CCI_REG16(0x0312), 0x0001 },
	{ CCI_REG16(0x0340), 0x1408 },
	{ CCI_REG16(0x0342), 0x5070 },
	{ CCI_REG16(0x0702), 0x0000 },
	{ CCI_REG16(0x0202), 0x0100 },
	{ CCI_REG16(0x0200), 0x0100 },
	{ CCI_REG16(0x022c), 0x0100 },
	{ CCI_REG16(0x0226), 0x0100 },
	{ CCI_REG16(0x021e), 0x0000 },
	{ CCI_REG16(0x080e), 0x0000 },
	{ CCI_REG16(0x0b00), 0x0080 },
	{ CCI_REG16(0x0b08), 0x0101 },
	{ CCI_REG16(0x0d00), 0x0100 },
};

/* g1: binned 3264x2448 30fps (FLL 2932 / LLP 18000) */
static const struct cci_reg_sequence s5kgd2_mode_3264x2448_30fps_regs[] = {
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x2a08 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0xa5e0 },
	{ CCI_REG16(0x602a), 0x21f8 },
	{ CCI_REG16(0x6f12), 0x1358 },
	{ CCI_REG16(0x602a), 0x30ce },
	{ CCI_REG16(0x6f12), 0x01e4 },
	{ CCI_REG16(0x602a), 0x8a00 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x3770 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x220e },
	{ CCI_REG16(0x6f12), 0x0005 },
	{ CCI_REG16(0x6f12), 0x0005 },
	{ CCI_REG16(0x602a), 0x31cc },
	{ CCI_REG16(0x6f12), 0x0019 },
	{ CCI_REG16(0x602a), 0x31d2 },
	{ CCI_REG16(0x6f12), 0x0012 },
	{ CCI_REG16(0x602a), 0x31ca },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x602a), 0x221c },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x2a18 },
	{ CCI_REG16(0x6f12), 0x0801 },
	{ CCI_REG16(0x602a), 0x5090 },
	{ CCI_REG16(0x6f12), 0x0e00 },
	{ CCI_REG16(0x602a), 0x84f8 },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x602a), 0x8508 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5530 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x43b0 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x43d4 },
	{ CCI_REG16(0x6f12), 0x1300 },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x6f12), 0x13e8 },
	{ CCI_REG16(0x602a), 0x2a0e },
	{ CCI_REG16(0x6f12), 0x1fcd },
	{ CCI_REG16(0x602a), 0x1346 },
	{ CCI_REG16(0x6f12), 0x0018 },
	{ CCI_REG16(0x602a), 0x2308 },
	{ CCI_REG16(0x6f12), 0x005b },
	{ CCI_REG16(0x602a), 0x381a },
	{ CCI_REG16(0x6f12), 0x0008 },
	{ CCI_REG16(0x6f12), 0x0897 },
	{ CCI_REG16(0x602a), 0x3812 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x345e },
	{ CCI_REG16(0x6f12), 0x1f40 },
	{ CCI_REG16(0x6f12), 0x1f47 },
	{ CCI_REG16(0x602a), 0x3b1a },
	{ CCI_REG16(0x6f12), 0x000f },
	{ CCI_REG16(0x6f12), 0x000f },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x000f },
	{ CCI_REG16(0x6f12), 0x000f },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x3a9a },
	{ CCI_REG16(0x6f12), 0x0050 },
	{ CCI_REG16(0x6f12), 0x0050 },
	{ CCI_REG16(0x6f12), 0xffc0 },
	{ CCI_REG16(0x6f12), 0xffc0 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0xffc0 },
	{ CCI_REG16(0x6f12), 0xffc0 },
	{ CCI_REG16(0x6f12), 0x0050 },
	{ CCI_REG16(0x6f12), 0x0050 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x39ee },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x6f12), 0x1020 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x2a1e },
	{ CCI_REG16(0x6f12), 0x0108 },
	{ CCI_REG16(0x602a), 0x134c },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x30b8 },
	{ CCI_REG16(0x6f12), 0xd020 },
	{ CCI_REG16(0x602a), 0x2580 },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x602a), 0x2584 },
	{ CCI_REG16(0x6f12), 0x0226 },
	{ CCI_REG16(0x602a), 0x8a10 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x8a6e },
	{ CCI_REG16(0x6f12), 0x0fa0 },
	{ CCI_REG16(0x6f12), 0x0fa7 },
	{ CCI_REG16(0x602a), 0x8a20 },
	{ CCI_REG16(0x6f12), 0x0008 },
	{ CCI_REG16(0x602a), 0x8a26 },
	{ CCI_REG16(0x6f12), 0x0008 },
	{ CCI_REG16(0x602a), 0x8a1e },
	{ CCI_REG16(0x6f12), 0x0080 },
	{ CCI_REG16(0x602a), 0x8a68 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x8a3e },
	{ CCI_REG16(0x6f12), 0x03ff },
	{ CCI_REG16(0x602a), 0x8a44 },
	{ CCI_REG16(0x6f12), 0x0080 },
	{ CCI_REG16(0x602a), 0x1370 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1378 },
	{ CCI_REG16(0x6f12), 0x0003 },
	{ CCI_REG16(0x602a), 0x2134 },
	{ CCI_REG16(0x6f12), 0xffff },
	{ CCI_REG16(0x602a), 0x1fc0 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x005f },
	{ CCI_REG16(0x602a), 0x1fc6 },
	{ CCI_REG16(0x6f12), 0x0060 },
	{ CCI_REG16(0x602a), 0x1fcc },
	{ CCI_REG16(0x6f12), 0x1c80 },
	{ CCI_REG16(0x6f12), 0x1c82 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x6f12), 0xf4b2 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0xf484), 0x0080 },
	{ CCI_REG16(0xf498), 0x0343 },
	{ CCI_REG16(0xf496), 0x0200 },
	{ CCI_REG16(0xf4b2), 0x1c82 },
	{ CCI_REG16(0xf41c), 0x4116 },
	{ CCI_REG16(0xf418), 0x001f },
	{ CCI_REG16(0xb606), 0x0400 },
	{ CCI_REG16(0xf482), 0x800d },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x623c), 0x0080 },
	{ CCI_REG16(0x623e), 0x0000 },
	{ CCI_REG16(0x6240), 0x0000 },
	{ CCI_REG16(0xf4a6), 0x0016 },
	{ CCI_REG16(0x0344), 0x0018 },
	{ CCI_REG16(0x0346), 0x0018 },
	{ CCI_REG16(0x0348), 0x1997 },
	{ CCI_REG16(0x034a), 0x133f },
	{ CCI_REG16(0x034c), 0x0cc0 },
	{ CCI_REG16(0x034e), 0x0990 },
	{ CCI_REG16(0x0350), 0x0000 },
	{ CCI_REG16(0x0352), 0x0002 },
	{ CCI_REG16(0x0900), 0x2222 },
	{ CCI_REG16(0x0380), 0x0002 },
	{ CCI_REG16(0x0382), 0x0002 },
	{ CCI_REG16(0x0384), 0x0002 },
	{ CCI_REG16(0x0386), 0x0002 },
	{ CCI_REG16(0x0400), 0x1010 },
	{ CCI_REG16(0x0408), 0x0100 },
	{ CCI_REG16(0x040a), 0x0100 },
	{ CCI_REG16(0x040c), 0x0000 },
	{ CCI_REG16(0x0110), 0x1002 },
	{ CCI_REG16(0x0114), 0x0300 },
	{ CCI_REG16(0x0116), 0x3000 },
	{ CCI_REG16(0x0118), 0x0001 },
	{ CCI_REG16(0x011a), 0x0000 },
	{ CCI_REG16(0x011c), 0x0000 },
	{ CCI_REG16(0x0136), 0x1300 },
	{ CCI_REG16(0x013e), 0x00c8 },
	{ CCI_REG16(0x0300), 0x0007 },
	{ CCI_REG16(0x0302), 0x0001 },
	{ CCI_REG16(0x0304), 0x0002 },
	{ CCI_REG16(0x0306), 0x0091 },
	{ CCI_REG16(0x0308), 0x000b },
	{ CCI_REG16(0x030a), 0x0001 },
	{ CCI_REG16(0x030c), 0x0000 },
	{ CCI_REG16(0x030e), 0x0002 },
	{ CCI_REG16(0x0310), 0x00bc },
	{ CCI_REG16(0x0312), 0x0001 },
	{ CCI_REG16(0x0340), 0x0b74 },
	{ CCI_REG16(0x0342), 0x4650 },
	{ CCI_REG16(0x0702), 0x0000 },
	{ CCI_REG16(0x0202), 0x0100 },
	{ CCI_REG16(0x0200), 0x0100 },
	{ CCI_REG16(0x022c), 0x0100 },
	{ CCI_REG16(0x0226), 0x0100 },
	{ CCI_REG16(0x021e), 0x0000 },
	{ CCI_REG16(0x080e), 0x0000 },
	{ CCI_REG16(0x0b00), 0x0080 },
	{ CCI_REG16(0x0b08), 0x0100 },
	{ CCI_REG16(0x0d00), 0x0000 },
};

/* g2: binned 3264x2448 60fps (FLL 2932 / LLP 9040) */
static const struct cci_reg_sequence s5kgd2_mode_3264x2448_60fps_regs[] = {
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x2a08 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0xa5e0 },
	{ CCI_REG16(0x602a), 0x21f8 },
	{ CCI_REG16(0x6f12), 0x1358 },
	{ CCI_REG16(0x602a), 0x30ce },
	{ CCI_REG16(0x6f12), 0x01e4 },
	{ CCI_REG16(0x602a), 0x8a00 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x3770 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x220e },
	{ CCI_REG16(0x6f12), 0x0005 },
	{ CCI_REG16(0x6f12), 0x0005 },
	{ CCI_REG16(0x602a), 0x31cc },
	{ CCI_REG16(0x6f12), 0x0019 },
	{ CCI_REG16(0x602a), 0x31d2 },
	{ CCI_REG16(0x6f12), 0x0012 },
	{ CCI_REG16(0x602a), 0x31ca },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x602a), 0x221c },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x2a18 },
	{ CCI_REG16(0x6f12), 0x0801 },
	{ CCI_REG16(0x602a), 0x5090 },
	{ CCI_REG16(0x6f12), 0x0e00 },
	{ CCI_REG16(0x602a), 0x84f8 },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x602a), 0x8508 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x5530 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x43b0 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x43d4 },
	{ CCI_REG16(0x6f12), 0x1300 },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x6f12), 0x13e8 },
	{ CCI_REG16(0x602a), 0x2a0e },
	{ CCI_REG16(0x6f12), 0x1fcd },
	{ CCI_REG16(0x602a), 0x1346 },
	{ CCI_REG16(0x6f12), 0x0018 },
	{ CCI_REG16(0x602a), 0x2308 },
	{ CCI_REG16(0x6f12), 0x005b },
	{ CCI_REG16(0x602a), 0x381a },
	{ CCI_REG16(0x6f12), 0x0008 },
	{ CCI_REG16(0x6f12), 0x0897 },
	{ CCI_REG16(0x602a), 0x3812 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x345e },
	{ CCI_REG16(0x6f12), 0x1f40 },
	{ CCI_REG16(0x6f12), 0x1f47 },
	{ CCI_REG16(0x602a), 0x3b1a },
	{ CCI_REG16(0x6f12), 0x000f },
	{ CCI_REG16(0x6f12), 0x000f },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x000f },
	{ CCI_REG16(0x6f12), 0x000f },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x3a9a },
	{ CCI_REG16(0x6f12), 0x0050 },
	{ CCI_REG16(0x6f12), 0x0050 },
	{ CCI_REG16(0x6f12), 0xffc0 },
	{ CCI_REG16(0x6f12), 0xffc0 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0xffc0 },
	{ CCI_REG16(0x6f12), 0xffc0 },
	{ CCI_REG16(0x6f12), 0x0050 },
	{ CCI_REG16(0x6f12), 0x0050 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x39ee },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x6f12), 0x1020 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x2a1e },
	{ CCI_REG16(0x6f12), 0x0108 },
	{ CCI_REG16(0x602a), 0x134c },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x30b8 },
	{ CCI_REG16(0x6f12), 0xd020 },
	{ CCI_REG16(0x602a), 0x2580 },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x602a), 0x2584 },
	{ CCI_REG16(0x6f12), 0x0226 },
	{ CCI_REG16(0x602a), 0x8a10 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x8a6e },
	{ CCI_REG16(0x6f12), 0x0fa0 },
	{ CCI_REG16(0x6f12), 0x0fa7 },
	{ CCI_REG16(0x602a), 0x8a20 },
	{ CCI_REG16(0x6f12), 0x0008 },
	{ CCI_REG16(0x602a), 0x8a26 },
	{ CCI_REG16(0x6f12), 0x0008 },
	{ CCI_REG16(0x602a), 0x8a1e },
	{ CCI_REG16(0x6f12), 0x0080 },
	{ CCI_REG16(0x602a), 0x8a68 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x8a3e },
	{ CCI_REG16(0x6f12), 0x03ff },
	{ CCI_REG16(0x602a), 0x8a44 },
	{ CCI_REG16(0x6f12), 0x0080 },
	{ CCI_REG16(0x602a), 0x1370 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1378 },
	{ CCI_REG16(0x6f12), 0x0003 },
	{ CCI_REG16(0x602a), 0x2134 },
	{ CCI_REG16(0x6f12), 0xffff },
	{ CCI_REG16(0x602a), 0x1fc0 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x005f },
	{ CCI_REG16(0x602a), 0x1fc6 },
	{ CCI_REG16(0x6f12), 0x0060 },
	{ CCI_REG16(0x602a), 0x1fcc },
	{ CCI_REG16(0x6f12), 0x1c80 },
	{ CCI_REG16(0x6f12), 0x1c82 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x6f12), 0xf4b2 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0xf484), 0x0080 },
	{ CCI_REG16(0xf498), 0x0343 },
	{ CCI_REG16(0xf496), 0x0200 },
	{ CCI_REG16(0xf4b2), 0x1c82 },
	{ CCI_REG16(0xf41c), 0x4116 },
	{ CCI_REG16(0xf418), 0x001f },
	{ CCI_REG16(0xb606), 0x0400 },
	{ CCI_REG16(0xf482), 0x800d },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x623c), 0x0080 },
	{ CCI_REG16(0x623e), 0x0000 },
	{ CCI_REG16(0x6240), 0x0000 },
	{ CCI_REG16(0xf4a6), 0x0016 },
	{ CCI_REG16(0x0344), 0x0018 },
	{ CCI_REG16(0x0346), 0x0018 },
	{ CCI_REG16(0x0348), 0x1997 },
	{ CCI_REG16(0x034a), 0x133f },
	{ CCI_REG16(0x034c), 0x0cc0 },
	{ CCI_REG16(0x034e), 0x0990 },
	{ CCI_REG16(0x0350), 0x0000 },
	{ CCI_REG16(0x0352), 0x0002 },
	{ CCI_REG16(0x0900), 0x2222 },
	{ CCI_REG16(0x0380), 0x0002 },
	{ CCI_REG16(0x0382), 0x0002 },
	{ CCI_REG16(0x0384), 0x0002 },
	{ CCI_REG16(0x0386), 0x0002 },
	{ CCI_REG16(0x0400), 0x1010 },
	{ CCI_REG16(0x0408), 0x0100 },
	{ CCI_REG16(0x040a), 0x0100 },
	{ CCI_REG16(0x040c), 0x0000 },
	{ CCI_REG16(0x0110), 0x1002 },
	{ CCI_REG16(0x0114), 0x0300 },
	{ CCI_REG16(0x0116), 0x3000 },
	{ CCI_REG16(0x0118), 0x0001 },
	{ CCI_REG16(0x011a), 0x0000 },
	{ CCI_REG16(0x011c), 0x0000 },
	{ CCI_REG16(0x0136), 0x1300 },
	{ CCI_REG16(0x013e), 0x00c8 },
	{ CCI_REG16(0x0300), 0x0007 },
	{ CCI_REG16(0x0302), 0x0001 },
	{ CCI_REG16(0x0304), 0x0002 },
	{ CCI_REG16(0x0306), 0x0091 },
	{ CCI_REG16(0x0308), 0x000b },
	{ CCI_REG16(0x030a), 0x0001 },
	{ CCI_REG16(0x030c), 0x0000 },
	{ CCI_REG16(0x030e), 0x0002 },
	{ CCI_REG16(0x0310), 0x00bc },
	{ CCI_REG16(0x0312), 0x0001 },
	{ CCI_REG16(0x0340), 0x0b74 },
	{ CCI_REG16(0x0342), 0x2350 },
	{ CCI_REG16(0x0702), 0x0000 },
	{ CCI_REG16(0x0202), 0x0100 },
	{ CCI_REG16(0x0200), 0x0100 },
	{ CCI_REG16(0x022c), 0x0100 },
	{ CCI_REG16(0x0226), 0x0100 },
	{ CCI_REG16(0x021e), 0x0000 },
	{ CCI_REG16(0x080e), 0x0000 },
	{ CCI_REG16(0x0b00), 0x0080 },
	{ CCI_REG16(0x0b08), 0x0100 },
	{ CCI_REG16(0x0d00), 0x0000 },
};

struct s5kgd2_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct s5kgd2_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts;
	u32 link_freq_idx;
	const struct s5kgd2_reg_list reg_list;
};

static const struct s5kgd2_mode s5kgd2_supported_modes[] = {
	{
		.width = 6528,
		.height = 4896,
		.hts = 20592,
		.vts = 5128,
		.link_freq_idx = S5KGD2_LF_IDX_ALL,
		.reg_list = {
			.regs = s5kgd2_mode_6528x4896_regs,
			.num_regs = ARRAY_SIZE(s5kgd2_mode_6528x4896_regs),
		},
	},
	{
		.width = 3264,
		.height = 2448,
		.hts = 18000,
		.vts = 2932,
		.link_freq_idx = S5KGD2_LF_IDX_ALL,
		.reg_list = {
			.regs = s5kgd2_mode_3264x2448_30fps_regs,
			.num_regs = ARRAY_SIZE(s5kgd2_mode_3264x2448_30fps_regs),
		},
	},
	{
		.width = 3264,
		.height = 2448,
		.hts = 9040,
		.vts = 2932,
		.link_freq_idx = S5KGD2_LF_IDX_ALL,
		.reg_list = {
			.regs = s5kgd2_mode_3264x2448_60fps_regs,
			.num_regs = ARRAY_SIZE(s5kgd2_mode_3264x2448_60fps_regs),
		},
	},
};

struct s5kgd2 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator *vdda;		/* analog 2.8V   = pm8008i L3 */
	struct regulator *vddd;		/* digital 1.056V = pm8008i L1 */
	struct regulator *vddio;	/* I/O 1.8V     = pm8008j L5 */

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *analog_gain;

	const struct s5kgd2_mode *mode;
};

static inline u64 s5kgd2_freq_to_pixel_rate(const u64 freq);

static int s5kgd2_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5kgd2 *s5kgd2 = container_of(ctrl->handler, struct s5kgd2,
					     ctrl_handler);
	const struct s5kgd2_mode *mode = s5kgd2->mode;
	s64 exposure_max;
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		exposure_max = mode->height + ctrl->val -
			       S5KGD2_EXPOSURE_MARGIN;
		__v4l2_ctrl_modify_range(s5kgd2->exposure,
					 s5kgd2->exposure->minimum,
					 exposure_max,
					 s5kgd2->exposure->step,
					 s5kgd2->exposure->default_value);
		break;
	case V4L2_CID_HBLANK:
		/* HTS tracks the request: line length = width + hblank */
		break;
	case V4L2_CID_LINK_FREQ:
		/* single-entry menu, nothing to propagate */
		break;
	}

	if (!pm_runtime_get_if_active(s5kgd2->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = cci_write(s5kgd2->regmap, S5KGD2_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(s5kgd2->regmap, S5KGD2_REG_ANALOG_GAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(s5kgd2->regmap, S5KGD2_REG_VTS,
				ctrl->val + mode->height, NULL);
		break;
	case V4L2_CID_HBLANK:
		ret = cci_write(s5kgd2->regmap, S5KGD2_REG_HTS,
				ctrl->val + mode->width, NULL);
		break;
	default:
		/* Not a hardware register ctrl (ranges already updated in the
		 * first switch): __v4l2_ctrl_handler_setup() walks ALL ctrls
		 * during streamon — returning -EINVAL here poisons the whole
		 * setup and trips the enable_streams error path (the r103
		 * ctrl-setup poison lesson, see s5k3p9sx.c). */
		ret = 0;
		break;
	}

	pm_runtime_put(s5kgd2->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5kgd2_ctrl_ops = {
	.s_ctrl = s5kgd2_set_ctrl,
};

static inline u64 s5kgd2_freq_to_pixel_rate(const u64 freq)
{
	return div_u64(freq * 2 * S5KGD2_DATA_LANES, 10);
}

static int s5kgd2_init_controls(struct s5kgd2 *s5kgd2)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &s5kgd2->ctrl_handler;
	const struct s5kgd2_mode *mode = s5kgd2->mode;
	s64 pixel_rate, hblank, vblank, exposure_max;

	v4l2_ctrl_handler_init(ctrl_hdlr, 6);

	s5kgd2->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr,
					&s5kgd2_ctrl_ops, V4L2_CID_LINK_FREQ,
					ARRAY_SIZE(s5kgd2_link_freq_menu) - 1,
					mode->link_freq_idx,
					s5kgd2_link_freq_menu);

	pixel_rate = s5kgd2_freq_to_pixel_rate(
			s5kgd2_link_freq_menu[mode->link_freq_idx]);
	s5kgd2->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &s5kgd2_ctrl_ops,
						V4L2_CID_PIXEL_RATE,
						0, pixel_rate, 1, pixel_rate);
	if (s5kgd2->pixel_rate)
		s5kgd2->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* HBLANK writable (libcamera simple-pipeline hard requirement, found
	 * via the 2026-11-21 plasma-camera probe on the 3P9 chain). */
	hblank = mode->hts - mode->width;
	s5kgd2->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5kgd2_ctrl_ops,
					   V4L2_CID_HBLANK, hblank,
					   S5KGD2_VTS_MAX - mode->width, 1,
					   hblank);

	vblank = mode->vts - mode->height;
	s5kgd2->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5kgd2_ctrl_ops,
					   V4L2_CID_VBLANK, vblank,
					   S5KGD2_VTS_MAX - mode->height,
					   1, vblank);

	exposure_max = mode->vts - S5KGD2_EXPOSURE_MARGIN;
	s5kgd2->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &s5kgd2_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     S5KGD2_EXPOSURE_MIN,
					     exposure_max,
					     S5KGD2_EXPOSURE_STEP,
					     mode->vts / 2);

	s5kgd2->analog_gain = v4l2_ctrl_new_std(ctrl_hdlr, &s5kgd2_ctrl_ops,
						V4L2_CID_ANALOGUE_GAIN,
						S5KGD2_ANALOG_GAIN_MIN,
						S5KGD2_ANALOG_GAIN_MAX,
						S5KGD2_ANALOG_GAIN_STEP,
						S5KGD2_ANALOG_GAIN_MIN);

	s5kgd2->sd.ctrl_handler = ctrl_hdlr;

	return 0;
}

static int s5kgd2_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct s5kgd2 *s5kgd2 = to_s5kgd2(sd);
	const struct s5kgd2_reg_list *reg_list = &s5kgd2->mode->reg_list;
	int ret;

	ret = pm_runtime_resume_and_get(s5kgd2->dev);
	if (ret)
		return ret;

	/* Vendor boot stream (bin g7 head): page 0x4000 + ID verify +
	 * patch trigger. +5ms delay comes from the vendor sequence. */
	cci_multi_reg_write(s5kgd2->regmap, s5kgd2_boot_seq,
			    ARRAY_SIZE(s5kgd2_boot_seq), &ret);
	if (ret)
		goto error;

	usleep_range(5000, 6000);

	/* PLL kick (0x6214/0x6218), then +11ms (vendor delays) */
	cci_multi_reg_write(s5kgd2->regmap, s5kgd2_pll_kick_seq,
			    ARRAY_SIZE(s5kgd2_pll_kick_seq), &ret);
	if (ret)
		goto error;

	usleep_range(11000, 12000);

	/* Sensor FW patch table (g7 tail, page 0x2001 @ 0xd604+) */
	cci_multi_reg_write(s5kgd2->regmap, s5kgd2_patch_seq,
			    ARRAY_SIZE(s5kgd2_patch_seq), &ret);
	if (ret)
		goto error;

	usleep_range(5 * USEC_PER_MSEC, 6 * USEC_PER_MSEC);

	cci_multi_reg_write(s5kgd2->regmap, reg_list->regs,
			    reg_list->num_regs, &ret);
	if (ret)
		goto error;

	ret = __v4l2_ctrl_handler_setup(s5kgd2->sd.ctrl_handler);

	/* Vendor stream-on orchestration (bin g5/g6 + Volla adaptor
	 * streaming_control()->set_dummy() equivalent): the sensor's
	 * stream-start FW transition reprograms the MIPI domain from its
	 * internal state, wiping direct pre-stream register writes (VTS/LLP/
	 * MIPI TX read back 0 after stream-on, 一百一十六). The working
	 * reference re-commits frame length (VTS) inside a group hold right
	 * before stream-on; the group-hold close is where the sensor FW
	 * ingests register changes. */
	cci_write(s5kgd2->regmap, S5KGD2_REG_GROUP_HOLD, 0x0001, &ret);
	cci_write(s5kgd2->regmap, CCI_REG16(0x0340), s5kgd2->mode->vts, &ret);
	cci_write(s5kgd2->regmap, S5KGD2_REG_GROUP_HOLD, 0x0000, &ret);
	if (ret)
		goto error;

	/* Stream on (bin g3: bare 0x0100 <- 0x0001, no page-select prefix) */
	cci_write(s5kgd2->regmap, S5KGD2_REG_MODE_SELECT,
		  S5KGD2_MODE_STREAMING, &ret);
	if (ret)
		goto error;

	usleep_range(5 * USEC_PER_MSEC, 6 * USEC_PER_MSEC);

	return 0;

error:
	dev_err(s5kgd2->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(s5kgd2->dev);

	return ret;
}

static int s5kgd2_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5kgd2 *s5kgd2 = to_s5kgd2(sd);
	int ret;

	ret = cci_write(s5kgd2->regmap, S5KGD2_REG_MODE_SELECT,
			S5KGD2_MODE_STANDBY, NULL);
	if (ret)
		dev_err(s5kgd2->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(s5kgd2->dev);

	return ret;
}

static void s5kgd2_update_pad_format(struct s5kgd2 *s5kgd2,
				     const struct s5kgd2_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = s5kgd2_mbus_format;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5kgd2_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct s5kgd2 *s5kgd2 = to_s5kgd2(sd);
	s64 hblank, vblank, exposure_max;
	const struct s5kgd2_mode *mode;
	u64 new_pixel_rate;

	mode = v4l2_find_nearest_size(s5kgd2_supported_modes,
				      ARRAY_SIZE(s5kgd2_supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	s5kgd2_update_pad_format(s5kgd2, mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY || s5kgd2->mode == mode)
		goto set_format;

	hblank = mode->hts - mode->width;
	__v4l2_ctrl_modify_range(s5kgd2->hblank, hblank,
				 S5KGD2_VTS_MAX - mode->width, 1, hblank);
	__v4l2_ctrl_s_ctrl(s5kgd2->hblank, hblank);

	vblank = mode->vts - mode->height;
	__v4l2_ctrl_modify_range(s5kgd2->vblank, vblank,
				 S5KGD2_VTS_MAX - mode->height, 1, vblank);
	__v4l2_ctrl_s_ctrl(s5kgd2->vblank, vblank);

	exposure_max = mode->vts - S5KGD2_EXPOSURE_MARGIN;
	__v4l2_ctrl_modify_range(s5kgd2->exposure, S5KGD2_EXPOSURE_MIN,
				 exposure_max, S5KGD2_EXPOSURE_STEP,
				 mode->vts / 2);
	__v4l2_ctrl_s_ctrl(s5kgd2->exposure, mode->vts / 2);

	new_pixel_rate = s5kgd2_freq_to_pixel_rate(
				s5kgd2_link_freq_menu[mode->link_freq_idx]);
	__v4l2_ctrl_modify_range(s5kgd2->pixel_rate, 0, new_pixel_rate,
				 1, new_pixel_rate);
	__v4l2_ctrl_s_ctrl_int64(s5kgd2->pixel_rate, new_pixel_rate);
	__v4l2_ctrl_s_ctrl(s5kgd2->link_freq, mode->link_freq_idx);

	if (s5kgd2->sd.ctrl_handler->error)
		return s5kgd2->sd.ctrl_handler->error;

	s5kgd2->mode = mode;

set_format:
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int s5kgd2_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = s5kgd2_mbus_format;

	return 0;
}

static int s5kgd2_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(s5kgd2_supported_modes))
		return -EINVAL;

	if (fse->code != s5kgd2_mbus_format)
		return -EINVAL;

	fse->min_width = s5kgd2_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5kgd2_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int s5kgd2_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct s5kgd2 *s5kgd2 = to_s5kgd2(sd);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = s5kgd2->mode->width;
		sel->r.height = s5kgd2->mode->height;
		return 0;
	default:
		return -EINVAL;
	}
}

static int s5kgd2_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct s5kgd2 *s5kgd2 = to_s5kgd2(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = s5kgd2->mode->width,
			.height = s5kgd2->mode->height,
		},
	};

	s5kgd2_set_pad_format(sd, state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops s5kgd2_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5kgd2_pad_ops = {
	.set_fmt = s5kgd2_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = s5kgd2_get_selection,
	.enum_mbus_code = s5kgd2_enum_mbus_code,
	.enum_frame_size = s5kgd2_enum_frame_size,
	.enable_streams = s5kgd2_enable_streams,
	.disable_streams = s5kgd2_disable_streams,
};

static const struct v4l2_subdev_ops s5kgd2_subdev_ops = {
	.video = &s5kgd2_video_ops,
	.pad = &s5kgd2_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5kgd2_internal_ops = {
	.init_state = s5kgd2_init_state,
};

static const struct media_entity_operations s5kgd2_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* Diagnostic: keep rails/MCLK on after a failed probe so the sensor's I2C
 * can be scanned with power applied (i2cdetect with the client unbound).
 * Survives only this GD2 bring-up round — remove when capture works. */
static bool keep_power;
module_param(keep_power, bool, 0444);

/* Diagnostic: skip the chip-id check so probe SUCCEEDS — pinctrl (MCLK4 pad
 * mux), reset GPIO and rails all stay claimed while bound; autosuspend
 * disabled. Combined with i2cdetect this gives a fully-powered bus scan.
 * Bring-up round only — remove when capture works. */
static bool no_probe;
module_param(no_probe, bool, 0444);

static int s5kgd2_identify_sensor(struct s5kgd2 *s5kgd2)
{
	u64 val;
	int ret, i;

	dev_dbg(s5kgd2->dev, "identify: probing chip id\n");
	for (i = 0; i < 3; i++) {
		ret = cci_read(s5kgd2->regmap, S5KGD2_REG_CHIP_ID, &val, NULL);
		dev_info(s5kgd2->dev,
			 "identify try %d: read 0x0000 -> ret=%d val=%#llx\n",
			 i, ret, ret ? 0 : val);
		if (!ret)
			break;
		usleep_range(50000, 60000);
	}
	if (ret) {
		dev_err(s5kgd2->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (val != S5KGD2_CHIP_ID) {
		dev_err(s5kgd2->dev, "chip id mismatch: %llx!=%x\n", val,
			S5KGD2_CHIP_ID);
		return -ENODEV;
	}

	return 0;
}

static int s5kgd2_check_hwcfg(struct s5kgd2 *s5kgd2)
{
	struct fwnode_handle *fwnode = dev_fwnode(s5kgd2->dev), *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus = {
			.mipi_csi2 = {
				.num_data_lanes = S5KGD2_DATA_LANES,
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

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != S5KGD2_DATA_LANES) {
		dev_err(s5kgd2->dev, "Invalid number of data lanes: %u\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto endpoint_free;
	}

	ret = v4l2_link_freq_to_bitmap(s5kgd2->dev,
				       bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       s5kgd2_link_freq_menu,
				       ARRAY_SIZE(s5kgd2_link_freq_menu),
				       &freq_bitmap);

endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int s5kgd2_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5kgd2 *s5kgd2 = to_s5kgd2(sd);
	int ret;

	/*
	 * Stock powerSetting (bin): rails vio -> vdig -> vana, reset pulse,
	 * MCLK 19.2MHz + 70ms. Samsung A54 order: rst-low -> VANA -> VDDD ->
	 * VDDIO -> rst-high -> i2c on -> MCLK. Same rails as 3P9 on this
	 * board; keep the proven mainline order (clock then reset release).
	 */
	ret = regulator_enable(s5kgd2->vddio);
	if (ret)
		return ret;

	ret = regulator_enable(s5kgd2->vddd);
	if (ret)
		goto disable_vddio;

	ret = regulator_enable(s5kgd2->vdda);
	if (ret)
		goto disable_vddd;

	usleep_range(5000, 6000);

	ret = clk_prepare_enable(s5kgd2->mclk);
	if (ret)
		goto disable_vdda;

	gpiod_set_value_cansleep(s5kgd2->reset_gpio, 0);
	/* stock bin powerSetting: MCLK on, +70ms settle before i2c; 12ms was
	 * insufficient on first bring-up (r109: identify -110), stock says 70. */
	usleep_range(75000, 80000);

	return 0;

disable_vdda:
	regulator_disable(s5kgd2->vdda);
disable_vddd:
	regulator_disable(s5kgd2->vddd);
disable_vddio:
	regulator_disable(s5kgd2->vddio);

	return ret;
}

static int s5kgd2_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5kgd2 *s5kgd2 = to_s5kgd2(sd);

	gpiod_set_value_cansleep(s5kgd2->reset_gpio, 1);

	clk_disable_unprepare(s5kgd2->mclk);

	regulator_disable(s5kgd2->vdda);
	regulator_disable(s5kgd2->vddd);
	regulator_disable(s5kgd2->vddio);

	return 0;
}

static int s5kgd2_probe(struct i2c_client *client)
{
	struct s5kgd2 *s5kgd2;
	unsigned long freq;
	int ret;

	s5kgd2 = devm_kzalloc(&client->dev, sizeof(*s5kgd2), GFP_KERNEL);
	if (!s5kgd2)
		return -ENOMEM;

	s5kgd2->dev = &client->dev;
	v4l2_i2c_subdev_init(&s5kgd2->sd, client, &s5kgd2_subdev_ops);

	s5kgd2->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5kgd2->regmap))
		return dev_err_probe(s5kgd2->dev, PTR_ERR(s5kgd2->regmap),
				     "failed to init CCI\n");

	s5kgd2->mclk = devm_v4l2_sensor_clk_get(s5kgd2->dev, NULL);
	if (IS_ERR(s5kgd2->mclk))
		return dev_err_probe(s5kgd2->dev, PTR_ERR(s5kgd2->mclk),
				     "failed to get MCLK clock\n");

	freq = clk_get_rate(s5kgd2->mclk);
	if (freq != S5KGD2_MCLK_FREQ_19MHZ)
		return dev_err_probe(s5kgd2->dev, -EINVAL,
				     "MCLK clock frequency %lu is not supported\n",
				     freq);

	ret = s5kgd2_check_hwcfg(s5kgd2);
	if (ret)
		return dev_err_probe(s5kgd2->dev, ret,
				     "failed to check HW configuration\n");

	s5kgd2->reset_gpio = devm_gpiod_get_optional(s5kgd2->dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(s5kgd2->reset_gpio))
		return dev_err_probe(s5kgd2->dev,
				     PTR_ERR(s5kgd2->reset_gpio),
				     "cannot get reset GPIO\n");

	s5kgd2->vdda = devm_regulator_get_optional(s5kgd2->dev, "vdda");
	if (IS_ERR(s5kgd2->vdda))
		return dev_err_probe(s5kgd2->dev, PTR_ERR(s5kgd2->vdda),
				     "cannot get vdda regulator\n");

	s5kgd2->vddd = devm_regulator_get_optional(s5kgd2->dev, "vddd");
	if (IS_ERR(s5kgd2->vddd))
		return dev_err_probe(s5kgd2->dev, PTR_ERR(s5kgd2->vddd),
				     "cannot get vddd regulator\n");

	s5kgd2->vddio = devm_regulator_get_optional(s5kgd2->dev, "vddio");
	if (IS_ERR(s5kgd2->vddio))
		return dev_err_probe(s5kgd2->dev, PTR_ERR(s5kgd2->vddio),
				     "cannot get vddio regulator\n");

	/* The sensor must be powered on to read the CHIP_ID register */
	ret = s5kgd2_power_on(s5kgd2->dev);
	if (ret)
		return ret;

	if (!no_probe) {
		ret = s5kgd2_identify_sensor(s5kgd2);
		if (ret) {
			dev_err_probe(s5kgd2->dev, ret, "failed to find sensor\n");
			goto power_off;
		}
	} else {
		dev_err(s5kgd2->dev,
			"no_probe=1: skipping chip-id, holding full power state\n");
	}

	s5kgd2->mode = &s5kgd2_supported_modes[0];
	ret = s5kgd2_init_controls(s5kgd2);
	if (ret) {
		dev_err_probe(s5kgd2->dev, ret, "failed to init controls\n");
		goto power_off;
	}

	s5kgd2->sd.state_lock = s5kgd2->ctrl_handler.lock;
	s5kgd2->sd.internal_ops = &s5kgd2_internal_ops;
	s5kgd2->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5kgd2->sd.entity.ops = &s5kgd2_subdev_entity_ops;
	s5kgd2->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5kgd2->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&s5kgd2->sd.entity, 1, &s5kgd2->pad);
	if (ret) {
		dev_err_probe(s5kgd2->dev, ret,
			      "failed to init media entity pads\n");
		goto v4l2_ctrl_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&s5kgd2->sd);
	if (ret < 0) {
		dev_err_probe(s5kgd2->dev, ret,
			      "failed to init v4l2 subdev\n");
		goto media_entity_cleanup;
	}

	pm_runtime_set_active(s5kgd2->dev);
	pm_runtime_enable(s5kgd2->dev);

	ret = v4l2_async_register_subdev_sensor(&s5kgd2->sd);
	if (ret < 0) {
		dev_err_probe(s5kgd2->dev, ret,
			      "failed to register V4L2 subdev\n");
		goto subdev_cleanup;
	}

	pm_runtime_set_autosuspend_delay(s5kgd2->dev,
					 no_probe ? -1 : 1000);
	pm_runtime_use_autosuspend(s5kgd2->dev);
	pm_runtime_idle(s5kgd2->dev);

	return 0;

subdev_cleanup:
	v4l2_subdev_cleanup(&s5kgd2->sd);
	pm_runtime_disable(s5kgd2->dev);
	pm_runtime_set_suspended(s5kgd2->dev);

media_entity_cleanup:
	media_entity_cleanup(&s5kgd2->sd.entity);

v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(s5kgd2->sd.ctrl_handler);

power_off:
	if (!keep_power)
		s5kgd2_power_off(s5kgd2->dev);
	else
		dev_err(s5kgd2->dev,
			"keep_power=1: rails/MCLK left ON for i2cdetect diagnostics\n");

	return ret;
}

static void s5kgd2_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5kgd2 *s5kgd2 = to_s5kgd2(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(s5kgd2->dev);

	if (!pm_runtime_status_suspended(s5kgd2->dev)) {
		s5kgd2_power_off(s5kgd2->dev);
		pm_runtime_set_suspended(s5kgd2->dev);
	}
}

static const struct dev_pm_ops s5kgd2_pm_ops = {
	SET_RUNTIME_PM_OPS(s5kgd2_power_off, s5kgd2_power_on, NULL)
};

static const struct of_device_id s5kgd2_of_match[] = {
	{ .compatible = "samsung,s5kgd2sp03" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5kgd2_of_match);

static struct i2c_driver s5kgd2_i2c_driver = {
	.driver = {
		.name = "s5kgd2sp03",
		.pm = &s5kgd2_pm_ops,
		.of_match_table = s5kgd2_of_match,
	},
	.probe = s5kgd2_probe,
	.remove = s5kgd2_remove,
};

module_i2c_driver(s5kgd2_i2c_driver);

MODULE_AUTHOR("Nura Meizu 20 porting crew");
MODULE_DESCRIPTION("Samsung S5KGD2SP03 image sensor driver");
MODULE_LICENSE("GPL");
