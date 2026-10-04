// SPDX-License-Identifier: GPL-2.0-only
/*
 * Awinic AW8697 LRA haptic driver (minimal ff-memless, CONT mode)
 *
 * Copyright (C) 2026 postmarketOS meizu20 port
 *
 * Partially based on the Awinic vendor driver (haptic_hv / aw8697_haptic).
 * Board values (170 Hz LRA, drive levels, BEMF config) are taken from the
 * Meizu 20 (m2381) factory DT haptic_hv@5a node.
 *
 * The motor is driven in vendor CONT mode: the chip generates the LRA drive
 * waveform itself at the preset resonance frequency (closed-loop zero-cross
 * tracking enabled), ff-memless only varies DRV_LVL and the input core's
 * timer stops playback by driving the level back to zero.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define AW8697_ID_REG				0x00
#define AW8697_CHIPID				0x97

#define AW8697_SYSINT_REG			0x02
#define AW8697_SYSINTM_REG			0x03
#define AW8697_SYSINTM_UVLO_MASK		BIT(5)
#define AW8697_SYSINTM_UVLO_EN			0
#define AW8697_SYSINTM_UVLO_OFF			1
#define AW8697_SYSINTM_OCD_MASK			BIT(2)
#define AW8697_SYSINTM_OCD_EN			0
#define AW8697_SYSINTM_OT_MASK			BIT(1)
#define AW8697_SYSINTM_OT_EN			0

#define AW8697_SYSCTRL_REG			0x04
#define AW8697_SYSCTRL_PLAY_MODE_MASK		GENMASK(3, 2)
#define AW8697_SYSCTRL_PLAY_MODE_CONT		(2 << 2)
#define AW8697_SYSCTRL_BST_MODE_MASK		BIT(1)
#define AW8697_SYSCTRL_BST_MODE_BYPASS		0
#define AW8697_SYSCTRL_WORK_MODE_MASK		BIT(0)
#define AW8697_SYSCTRL_ACTIVE			0
#define AW8697_SYSCTRL_STANDBY			1

#define AW8697_GO_REG				0x05
#define AW8697_GO_BIT				BIT(0)

#define AW8697_DBGCTRL_REG			0x20
#define AW8697_DBGCTRL_INT_MODE_MASK		BIT(2)
#define AW8697_DBGCTRL_INT_MODE_EDGE		1

#define AW8697_PWMPRC_REG			0x2d
#define AW8697_PWMPRC_PRC_MASK			BIT(7)
#define AW8697_PWMPRC_PRC_DISABLE		0

#define AW8697_DATCTRL_REG			0x2b
#define AW8697_DATCTRL_FC_MASK			GENMASK(7, 6)
#define AW8697_DATCTRL_FC_1000HZ		(3 << 6)
#define AW8697_DATCTRL_LPF_ENABLE_MASK		BIT(5)
#define AW8697_DATCTRL_LPF_ENABLE		1

#define AW8697_BSTDBG1_REG			0x31
#define AW8697_ANADBG_REG			0x35
#define AW8697_ANADBG_IOC_MASK			GENMASK(3, 2)
#define AW8697_ANADBG_IOC_4P65A			(3 << 2)
#define AW8697_ANACTRL_REG			0x36
#define AW8697_ANACTRL_LRA_SRC_MASK		BIT(5)
#define AW8697_ANACTRL_LRA_SRC_REG		1

#define AW8697_PRLVL_REG			0x3e
#define AW8697_PRLVL_PR_ENABLE			BIT(7)
#define AW8697_PRLVL_DEFAULT			0x3f
#define AW8697_PRTIME_REG			0x3f
#define AW8697_PRTIME_DEFAULT			0x12

#define AW8697_GLB_STATE_REG			0x46
#define AW8697_GLB_STATE_STANDBY		0

#define AW8697_CONT_CTRL_REG			0x48
#define AW8697_CONT_CTRL_ZC_DETEC_MASK		BIT(7)
#define AW8697_CONT_CTRL_ZC_DETEC_ENABLE	1
#define AW8697_CONT_CTRL_WAIT_PERIOD_MASK	GENMASK(6, 5)
#define AW8697_CONT_CTRL_WAIT_1PERIOD		0
#define AW8697_CONT_CTRL_MODE_MASK		BIT(4)
#define AW8697_CONT_CTRL_BY_GO_SIGNAL		0
#define AW8697_CONT_CTRL_EN_CLOSE_MASK		BIT(3)
#define AW8697_CONT_CTRL_CLOSE_PLAYBACK		1
#define AW8697_CONT_CTRL_F0_DETECT_MASK		BIT(2)
#define AW8697_CONT_CTRL_F0_DETECT_DISABLE	0
#define AW8697_CONT_CTRL_O2C_MASK		BIT(1)
#define AW8697_CONT_CTRL_O2C_DISABLE		0
#define AW8697_CONT_CTRL_AUTO_BRK_MASK		BIT(0)
#define AW8697_CONT_CTRL_AUTO_BRK_ENABLE	1

#define AW8697_F_PRE_H_REG			0x49
#define AW8697_TD_H_REG				0x4b
#define AW8697_TSET_REG				0x4d
#define AW8697_R_SPARE_REG			0x5d
#define AW8697_ADCTEST_REG			0x66
#define AW8697_ADCTEST_VBAT_MODE_MASK		BIT(6)
#define AW8697_ADCTEST_VBAT_HW_COMP		1
#define AW8697_ZC_THRSH_H_REG			0x72
#define AW8697_BEMF_VTHH_H_REG			0x74
#define AW8697_BEMF_NUM_REG			0x78
#define AW8697_BEMF_NUM_BRK_MASK		GENMASK(3, 0)
#define AW8697_TIME_NZC_REG			0x7a
#define AW8697_DRV_LVL_REG			0x7b
#define AW8697_DRV_LVL_OV_REG			0x7c

/*
 * Board values from the Meizu 20 factory DT (haptic_hv@5a):
 *   f0_pre=1700 (170.0 Hz X-axis LRA), f0_coeff=260
 *     -> F_PRE = 1e9 / (1700 * 260) = 2262
 *   cont_drv_lev=0x35 (steady drive), cont_drv_lvl_ov=0x7d (startup overdrive),
 *   cont_td=0x9a, cont_zc_thr=0xff1, cont_num_brk=3, tset=0x12,
 *   r_spare=0x68, bstdbg={0x30, 0xeb, 0xd4}, bemf={0x10, 0x08, 0x03, 0xf8}
 */
#define AW8697_F_PRE				2262
#define AW8697_DRV_LVL_MAX			0x35
#define AW8697_DRV_LVL_OVERDRIVE		0x7d
#define AW8697_CONT_TD				0x9a
#define AW8697_CONT_ZC_THR			0x0ff1
#define AW8697_CONT_NUM_BRK			3
#define AW8697_TSET				0x12
#define AW8697_R_SPARE				0x68
#define AW8697_TIME_NZC				0x23

struct aw8697 {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	struct input_dev *input;
	struct work_struct play_work;
	struct mutex lock;
	u16 level;
};

static const struct regmap_config aw8697_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
	.max_register = 0xff,
};

static int aw8697_start_cont(struct aw8697 *haptics)
{
	unsigned int val;
	int err;

	/* CONT mode, boost bypass */
	err = regmap_update_bits(haptics->regmap, AW8697_SYSCTRL_REG,
				 AW8697_SYSCTRL_PLAY_MODE_MASK |
				 AW8697_SYSCTRL_BST_MODE_MASK,
				 AW8697_SYSCTRL_PLAY_MODE_CONT |
				 FIELD_PREP(AW8697_SYSCTRL_BST_MODE_MASK,
					    AW8697_SYSCTRL_BST_MODE_BYPASS));
	if (err)
		return err;

	/* leave standby */
	err = regmap_update_bits(haptics->regmap, AW8697_SYSCTRL_REG,
				 AW8697_SYSCTRL_WORK_MODE_MASK,
				 AW8697_SYSCTRL_ACTIVE);
	if (err)
		return err;

	/* hw vbat compensation for the continuous drive */
	err = regmap_update_bits(haptics->regmap, AW8697_ADCTEST_REG,
				 AW8697_ADCTEST_VBAT_MODE_MASK,
				 FIELD_PREP(AW8697_ADCTEST_VBAT_MODE_MASK,
					    AW8697_ADCTEST_VBAT_HW_COMP));
	if (err)
		return err;

	/* RTP/LPF data rate */
	err = regmap_update_bits(haptics->regmap, AW8697_DATCTRL_REG,
				 AW8697_DATCTRL_FC_MASK |
				 AW8697_DATCTRL_LPF_ENABLE_MASK,
				 AW8697_DATCTRL_FC_1000HZ |
				 FIELD_PREP(AW8697_DATCTRL_LPF_ENABLE_MASK,
					    AW8697_DATCTRL_LPF_ENABLE));
	if (err)
		return err;

	/* closed-loop CONT config: zero-cross tracking + auto brake */
	val = FIELD_PREP(AW8697_CONT_CTRL_ZC_DETEC_MASK,
			 AW8697_CONT_CTRL_ZC_DETEC_ENABLE) |
	      FIELD_PREP(AW8697_CONT_CTRL_WAIT_PERIOD_MASK,
			 AW8697_CONT_CTRL_WAIT_1PERIOD) |
	      FIELD_PREP(AW8697_CONT_CTRL_MODE_MASK,
			 AW8697_CONT_CTRL_BY_GO_SIGNAL) |
	      FIELD_PREP(AW8697_CONT_CTRL_EN_CLOSE_MASK,
			 AW8697_CONT_CTRL_CLOSE_PLAYBACK) |
	      FIELD_PREP(AW8697_CONT_CTRL_F0_DETECT_MASK,
			 AW8697_CONT_CTRL_F0_DETECT_DISABLE) |
	      FIELD_PREP(AW8697_CONT_CTRL_O2C_MASK,
			 AW8697_CONT_CTRL_O2C_DISABLE) |
	      FIELD_PREP(AW8697_CONT_CTRL_AUTO_BRK_MASK,
			 AW8697_CONT_CTRL_AUTO_BRK_ENABLE);
	err = regmap_update_bits(haptics->regmap, AW8697_CONT_CTRL_REG,
				 AW8697_CONT_CTRL_ZC_DETEC_MASK |
				 AW8697_CONT_CTRL_WAIT_PERIOD_MASK |
				 AW8697_CONT_CTRL_MODE_MASK |
				 AW8697_CONT_CTRL_EN_CLOSE_MASK |
				 AW8697_CONT_CTRL_F0_DETECT_MASK |
				 AW8697_CONT_CTRL_O2C_MASK |
				 AW8697_CONT_CTRL_AUTO_BRK_MASK, val);
	if (err)
		return err;

	/* braking/tracking timings */
	err = regmap_write(haptics->regmap, AW8697_TD_H_REG,
			   AW8697_CONT_TD >> 8);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8697_TD_H_REG + 1,
			   AW8697_CONT_TD & 0xff);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8697_TSET_REG, AW8697_TSET);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8697_ZC_THRSH_H_REG,
			   AW8697_CONT_ZC_THR >> 8);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8697_ZC_THRSH_H_REG + 1,
			   AW8697_CONT_ZC_THR & 0xff);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8697_BEMF_NUM_REG,
				 AW8697_BEMF_NUM_BRK_MASK,
				 AW8697_CONT_NUM_BRK);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8697_TIME_NZC_REG,
			   AW8697_TIME_NZC);
	if (err)
		return err;

	/* drive level (scaled by ff-memless) + startup overdrive level */
	err = regmap_write(haptics->regmap, AW8697_DRV_LVL_REG,
			   haptics->level * AW8697_DRV_LVL_MAX / 0xffff);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8697_DRV_LVL_OV_REG,
			   AW8697_DRV_LVL_OVERDRIVE);
	if (err)
		return err;

	/* go */
	return regmap_write(haptics->regmap, AW8697_GO_REG, AW8697_GO_BIT);
}

static int aw8697_stop(struct aw8697 *haptics)
{
	unsigned int val;
	int err;

	err = regmap_write(haptics->regmap, AW8697_GO_REG, 0);
	if (err)
		return err;

	err = regmap_read_poll_timeout(haptics->regmap, AW8697_GLB_STATE_REG,
				       val,
				       (val & GENMASK(3, 0)) ==
					AW8697_GLB_STATE_STANDBY,
				       2500, 2500 * 100);
	if (err)
		dev_warn(haptics->dev, "did not enter standby: %d\n", err);

	/* standby; re-mask UVLO like the vendor standby path */
	return regmap_update_bits(haptics->regmap, AW8697_SYSCTRL_REG,
				  AW8697_SYSCTRL_WORK_MODE_MASK,
				  AW8697_SYSCTRL_STANDBY);
}

static void aw8697_play_work(struct work_struct *work)
{
	struct aw8697 *haptics = container_of(work, struct aw8697, play_work);
	int err;

	mutex_lock(&haptics->lock);

	if (haptics->level)
		err = aw8697_start_cont(haptics);
	else
		err = aw8697_stop(haptics);

	mutex_unlock(&haptics->lock);

	if (err)
		dev_err(haptics->dev, "play failed: %d\n", err);
}

static int aw8697_play(struct input_dev *input, void *data,
		       struct ff_effect *effect)
{
	struct aw8697 *haptics = input_get_drvdata(input);
	u16 level;

	level = max(effect->u.rumble.strong_magnitude,
		    effect->u.rumble.weak_magnitude);

	if (level == haptics->level)
		return 0;

	haptics->level = level;
	schedule_work(&haptics->play_work);

	return 0;
}

static void aw8697_close(struct input_dev *input)
{
	struct aw8697 *haptics = input_get_drvdata(input);

	cancel_work_sync(&haptics->play_work);

	mutex_lock(&haptics->lock);
	haptics->level = 0;
	aw8697_stop(haptics);
	mutex_unlock(&haptics->lock);
}

static void aw8697_hw_reset(struct aw8697 *haptics)
{
	/* the reset line is physically active-low (r56 device-day measured) */
	gpiod_set_value_cansleep(haptics->reset_gpio, 1);
	usleep_range(5000, 5500);
	gpiod_set_value_cansleep(haptics->reset_gpio, 0);
	/* ~8ms until I2C is accessible */
	usleep_range(8000, 8500);
}

static int aw8697_init(struct aw8697 *haptics)
{
	static const u8 bstdbg[3] = { 0x30, 0xeb, 0xd4 };
	static const u8 bemf[4] = { 0x10, 0x08, 0x03, 0xf8 };
	unsigned int val;
	int i, err;

	/* clear pending interrupts */
	err = regmap_read(haptics->regmap, AW8697_SYSINT_REG, &val);
	if (err)
		return err;

	/* edge interrupt mode, like the vendor setup */
	err = regmap_update_bits(haptics->regmap, AW8697_DBGCTRL_REG,
				 AW8697_DBGCTRL_INT_MODE_MASK,
				 FIELD_PREP(AW8697_DBGCTRL_INT_MODE_MASK,
					    AW8697_DBGCTRL_INT_MODE_EDGE));
	if (err)
		return err;

	/* unmask fault interrupts (UVLO/OCD/OT); FF/DONE stay masked */
	err = regmap_update_bits(haptics->regmap, AW8697_SYSINTM_REG,
				 AW8697_SYSINTM_UVLO_MASK,
				 AW8697_SYSINTM_UVLO_EN);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8697_SYSINTM_REG,
				 AW8697_SYSINTM_OCD_MASK,
				 AW8697_SYSINTM_OCD_EN);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8697_SYSINTM_REG,
				 AW8697_SYSINTM_OT_MASK,
				 AW8697_SYSINTM_OT_EN);
	if (err)
		return err;

	/* vendor misc init (bstdbg/tset/r_spare/analog config) */
	for (i = 0; i < ARRAY_SIZE(bstdbg); i++) {
		err = regmap_write(haptics->regmap,
				   AW8697_BSTDBG1_REG + i, bstdbg[i]);
		if (err)
			return err;
	}
	err = regmap_write(haptics->regmap, AW8697_TSET_REG, AW8697_TSET);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8697_R_SPARE_REG, AW8697_R_SPARE);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8697_ANADBG_REG,
				 AW8697_ANADBG_IOC_MASK,
				 AW8697_ANADBG_IOC_4P65A);
	if (err)
		return err;
	/* LRA frequency source = registers (not efuse) */
	err = regmap_update_bits(haptics->regmap, AW8697_ANACTRL_REG,
				 AW8697_ANACTRL_LRA_SRC_MASK,
				 FIELD_PREP(AW8697_ANACTRL_LRA_SRC_MASK,
					    AW8697_ANACTRL_LRA_SRC_REG));
	if (err)
		return err;
	for (i = 0; i < ARRAY_SIZE(bemf); i++) {
		err = regmap_write(haptics->regmap,
				   AW8697_BEMF_VTHH_H_REG + i, bemf[i]);
		if (err)
			return err;
	}

	/* motor protection: disable PWM rate correction, enable PR */
	err = regmap_update_bits(haptics->regmap, AW8697_PWMPRC_REG,
				 AW8697_PWMPRC_PRC_MASK,
				 FIELD_PREP(AW8697_PWMPRC_PRC_MASK,
					    AW8697_PWMPRC_PRC_DISABLE));
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8697_PRLVL_REG,
			   AW8697_PRLVL_PR_ENABLE | AW8697_PRLVL_DEFAULT);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8697_PRTIME_REG,
			   AW8697_PRTIME_DEFAULT);
	if (err)
		return err;

	/* preset resonance: F_PRE = 1e9 / (f0_pre * f0_coeff) */
	err = regmap_write(haptics->regmap, AW8697_F_PRE_H_REG,
			   (AW8697_F_PRE >> 8) & 0xff);
	if (err)
		return err;
	return regmap_write(haptics->regmap, AW8697_F_PRE_H_REG + 1,
			    AW8697_F_PRE & 0xff);
}

static irqreturn_t aw8697_irq(int irq, void *data)
{
	struct aw8697 *haptics = data;
	unsigned int reg_val;
	int err;

	err = regmap_read(haptics->regmap, AW8697_SYSINT_REG, &reg_val);
	if (err) {
		dev_err(haptics->dev, "Failed to read SYSINT: %d\n", err);
		return IRQ_NONE;
	}

	if (reg_val & BIT(7))
		dev_err(haptics->dev, "boost error interrupt\n");
	if (reg_val & BIT(6))
		dev_err(haptics->dev, "over voltage interrupt\n");
	if (reg_val & BIT(5))
		dev_err(haptics->dev, "under voltage lock out interrupt\n");
	if (reg_val & BIT(2))
		dev_err(haptics->dev, "over current interrupt\n");
	if (reg_val & BIT(1))
		dev_err(haptics->dev, "over temperature interrupt\n");

	return IRQ_HANDLED;
}

static int aw8697_probe(struct i2c_client *client)
{
	struct aw8697 *haptics;
	unsigned int id;
	int err;

	haptics = devm_kzalloc(&client->dev, sizeof(*haptics), GFP_KERNEL);
	if (!haptics)
		return -ENOMEM;

	haptics->dev = &client->dev;
	haptics->client = client;
	mutex_init(&haptics->lock);
	INIT_WORK(&haptics->play_work, aw8697_play_work);
	i2c_set_clientdata(client, haptics);

	haptics->regmap = devm_regmap_init_i2c(client, &aw8697_regmap_config);
	if (IS_ERR(haptics->regmap))
		return dev_err_probe(haptics->dev, PTR_ERR(haptics->regmap),
				     "Failed to allocate register map\n");

	haptics->reset_gpio = devm_gpiod_get(haptics->dev, "reset",
					     GPIOD_OUT_HIGH);
	if (IS_ERR(haptics->reset_gpio))
		return dev_err_probe(haptics->dev, PTR_ERR(haptics->reset_gpio),
				     "Failed to get reset gpio\n");

	aw8697_hw_reset(haptics);

	err = regmap_read(haptics->regmap, AW8697_ID_REG, &id);
	if (err)
		return dev_err_probe(haptics->dev, err,
				     "Failed to read chip id\n");
	if (id != AW8697_CHIPID) {
		dev_err(haptics->dev, "Unexpected chip id 0x%02x\n", id);
		return -ENODEV;
	}

	err = aw8697_init(haptics);
	if (err)
		return dev_err_probe(haptics->dev, err,
				     "Failed to init chip\n");

	err = devm_request_threaded_irq(haptics->dev, client->irq, NULL,
					aw8697_irq, IRQF_ONESHOT, NULL,
					haptics);
	if (err)
		return dev_err_probe(haptics->dev, err,
				     "Failed to request irq\n");

	haptics->input = devm_input_allocate_device(haptics->dev);
	if (!haptics->input)
		return -ENOMEM;

	haptics->input->name = "aw8697-haptics";
	haptics->input->close = aw8697_close;
	input_set_drvdata(haptics->input, haptics);
	input_set_capability(haptics->input, EV_FF, FF_RUMBLE);

	err = input_ff_create_memless(haptics->input, NULL, aw8697_play);
	if (err)
		return dev_err_probe(haptics->dev, err,
				     "Failed to create FF device\n");

	err = input_register_device(haptics->input);
	if (err)
		return dev_err_probe(haptics->dev, err,
				     "Failed to register input device\n");

	return 0;
}

static const struct of_device_id aw8697_of_id[] = {
	{ .compatible = "awinic,aw8697" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, aw8697_of_id);

static struct i2c_driver aw8697_driver = {
	.driver = {
		.name = "aw8697-haptics",
		.of_match_table = aw8697_of_id,
	},
	.probe = aw8697_probe,
};
module_i2c_driver(aw8697_driver);

MODULE_AUTHOR("postmarketOS meizu20 port");
MODULE_DESCRIPTION("Awinic AW8697 LRA haptic driver");
MODULE_LICENSE("GPL");
