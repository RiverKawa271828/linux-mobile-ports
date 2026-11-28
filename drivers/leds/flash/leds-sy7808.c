// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Silergy SY7808 flash LED driver.
 *
 * Bring-up provenance (Meizu 20 m2381 dual ring-flash, 2026-11-27): register
 * map, mode bits and current-code formulas were reverse-engineered from the
 * stock camera.ko (si,sy7808 functions, symbols unstripped):
 *   init (first claim):    01<-00, 0a<-0f, 03/04/05/06/07/08<-00
 *   torch enable:          01 |= 0x0C | channels        (bit2 torch + bit3 rail)
 *   flash enable:          01 |= 0x08 | channels + strobe arm 05 |= 0x80
 *   disable:               01 &= ~channels; 05 &= ~0x80; full off -> 01<-00
 *   torch current code:    clamp(mA*100/1758, 1..60) - 1  -> 06/07/08
 *   flash current code:    clamp(mA*1000/35175, 1..30) - 1 -> 03/04
 *   channels:              bit0 = LED1, bit1 = LED2
 * Board truth (stock FDT): two chips at i2c hubs 3 and 2, addr 0x63, LED
 * current is supplied by the chip-internal boost (the "anode mystery" of the
 * S2 ring flash: no PMIC rail carries it). Chip enable = tlmm gpio49 (stock
 * qcom,camera-flash0/1/2 FLASH_STROBE line, shared by both chips; stock pins
 * cam_flash_rear_active = output-high from probe onward — without it the
 * chips hold on I2C, verified 2026-11-27: 0x63 NACK on both hub buses while
 * pm8008/retimer on the same buses ACK fine). The line is optional in the
 * driver: first probed chip claims it, the second relies on it already
 * being high (probe deferral bridges any order gap).
 * SAFETY: gpio49's exact on-chip function (EN vs FLEN strobe input) is not
 * pinned by RE — raising it may trigger a flash burst on hardware strobe
 * designs. Point-on tests run with the user present only.
 *
 * Software-strobe timeout like stock camx (duration-ms 400): the chip has no
 * duration register; the driver arms 0x05 bit7 and schedules auto-off.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/led-class-flash.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define SY7808_REG_ENABLE		0x01
#define SY7808_REG_FLASH_I_LED1		0x03
#define SY7808_REG_FLASH_I_LED2		0x04
#define SY7808_REG_STROBE		0x05
#define SY7808_REG_TORCH_I_LED1		0x06
#define SY7808_REG_TORCH_I_LED2		0x07
#define SY7808_REG_TORCH_I_JOINT	0x08
#define SY7808_REG_CFG			0x0a

/* mode field bits[3:2] — r150 user-space i2c matrix arbitration (live):
 * 0x8 = torch, 0xC = flash.  The 133-round REA verdict ("torch = 0xC,
 * flash = 0x8") was INVERTED: mode 0xC written without 0x05 bit7 is
 * auto-cleared by the chip (flash mode requires the third-sink arm),
 * and 0xC + bit7 (r150 first patch) = flash armed waiting for the
 * strobe line => everything dark.  m2391 stock source agrees
 * (enable(): mode==2 -> |=8, else |=12). */
#define SY7808_MODE_FLASH		(BIT(2) | BIT(3))
#define SY7808_MODE_TORCH		BIT(3)
#define SY7808_MODE_MASK		(BIT(2) | BIT(3))
#define SY7808_CH_MASK			GENMASK(1, 0)
#define SY7808_CH_ALL			SY7808_CH_MASK

/* 0x05 is dual-field (m2391 stock source + live 6/6-arbitrated):
 * bit7 = third-sink (LED3) enable — stock enable() channels&4 path
 * sets it in BOTH torch and flash; bits[6:0] = LED3 current in
 * torch-granularity code (torch currents for LED1/2 live in
 * 0x06/07/08, "JOINT" = LED3 torch code). */
#define SY7808_LED3_EN			BIT(7)
#define SY7808_FLASH_I_LED3_MASK	GENMASK(6, 0)

#define SY7808_TORCH_STEPS_MAX		60
#define SY7808_FLASH_STEPS_MAX		30

/* code = clamp(level_mA*100/1758, 1..60) - 1  => 1 code step ~ 17.58 mA */
#define SY7808_TORCH_STEP_UA		17580U
/* code = clamp(level_mA*1000/35175, 1..30) - 1 => 1 code step ~ 35.175 mA */
#define SY7808_FLASH_STEP_UA		35175U

#define SY7808_TIMEOUT_DEFAULT_MS	400U
#define SY7808_TIMEOUT_MAX_MS		1000U

struct sy7808 {
	struct device *dev;
	struct regmap *regmap;
	struct mutex mutex;

	struct gpio_desc *enable_gpio;
	struct led_classdev_flash fled;
	struct delayed_work strobe_off_work;
	u32 strobe_timeout_ms;
	u32 flash_max_ua;
	u32 torch_max_ua;
	bool strobe_active;
};

static struct sy7808 *flcdev_to_sy7808(struct led_classdev_flash *fled)
{
	return container_of(fled, struct sy7808, fled);
}

static const struct regmap_config sy7808_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SY7808_REG_CFG,
};

/* ---- stock init sequence (RE: sy7808_init first-claim branch) ---- */
static int sy7808_chip_init(struct sy7808 *chip)
{
	static const struct {
		u8 reg;
		u8 val;
	} seq[] = {
		{ SY7808_REG_ENABLE, 0x00 },
		{ SY7808_REG_CFG, 0x0f },
		{ SY7808_REG_FLASH_I_LED1, 0x00 },
		{ SY7808_REG_FLASH_I_LED2, 0x00 },
		{ SY7808_REG_STROBE, 0x00 },
		{ SY7808_REG_TORCH_I_LED1, 0x00 },
		{ SY7808_REG_TORCH_I_LED2, 0x00 },
		{ SY7808_REG_TORCH_I_JOINT, 0x00 },
	};
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(seq); i++) {
		ret = regmap_write(chip->regmap, seq[i].reg, seq[i].val);
		if (ret)
			return ret;
	}
	return 0;
}

static int sy7808_torch_brightness_set(struct led_classdev *lcdev,
				       enum led_brightness brightness)
{
	struct led_classdev_flash *fled = lcdev_to_flcdev(lcdev);
	struct sy7808 *chip = flcdev_to_sy7808(fled);
	unsigned int level_ua = brightness * SY7808_TORCH_STEP_UA;
	u8 code;
	int ret;

	mutex_lock(&chip->mutex);

	if (chip->strobe_active) {
		dev_warn(chip->dev, "torch rejected: strobe active\n");
		ret = -EBUSY;
		goto unlock;
	}

	if (brightness > lcdev->max_brightness) {
		ret = -EINVAL;
		goto unlock;
	}

	/* code = clamp(mA*100/1758, 1..60) - 1, brightness 0 = off */
	code = brightness ? clamp(level_ua / SY7808_TORCH_STEP_UA, 1U, SY7808_TORCH_STEPS_MAX) - 1 : 0;

	ret = regmap_write(chip->regmap, SY7808_REG_TORCH_I_LED1, code);
	if (ret)
		goto unlock;
	ret = regmap_write(chip->regmap, SY7808_REG_TORCH_I_LED2, code);
	if (ret)
		goto unlock;
	ret = regmap_write(chip->regmap, SY7808_REG_TORCH_I_JOINT, code);
	if (ret)
		goto unlock;

	if (brightness) {
		ret = regmap_update_bits(chip->regmap, SY7808_REG_ENABLE,
					 SY7808_MODE_MASK | SY7808_CH_MASK,
					 SY7808_MODE_TORCH | SY7808_CH_ALL);
		if (!ret)
			/* stock enable() channels&4 form (live 6/6 verdict):
			 * third sink = 0x05 bit7 + torch code in 0x05[6:0] */
			ret = regmap_update_bits(chip->regmap, SY7808_REG_STROBE,
						 SY7808_LED3_EN | SY7808_FLASH_I_LED3_MASK,
						 SY7808_LED3_EN | code);
	} else {
		ret = regmap_update_bits(chip->regmap, SY7808_REG_ENABLE,
					 SY7808_MODE_MASK | SY7808_CH_MASK, 0);
		if (!ret)
			ret = regmap_update_bits(chip->regmap, SY7808_REG_STROBE,
						 SY7808_LED3_EN | SY7808_FLASH_I_LED3_MASK, 0);
	}
	if (ret)
		goto unlock;

	dev_dbg(chip->dev, "torch: brightness=%u code=%u\n", brightness, code);

unlock:
	mutex_unlock(&chip->mutex);
	return ret;
}

static int sy7808_flash_brightness_set(struct led_classdev_flash *fled, u32 brightness)
{
	struct sy7808 *chip = flcdev_to_sy7808(fled);
	struct led_flash_setting *s = &fled->brightness;
	unsigned int level_ua = (brightness - s->min) * SY7808_FLASH_STEP_UA;
	u8 code;
	int ret;

	code = clamp(level_ua / SY7808_FLASH_STEP_UA, 1U, SY7808_FLASH_STEPS_MAX) - 1;

	mutex_lock(&chip->mutex);
	ret = regmap_write(chip->regmap, SY7808_REG_FLASH_I_LED1, code);
	if (!ret)
		ret = regmap_write(chip->regmap, SY7808_REG_FLASH_I_LED2, code);
	if (!ret)
		/* stock set_duty flash branch: 0x05[6:0] = LED3 flash current,
		 * torch-granularity 60-step code (bit7 armed later in strobe_set) */
		ret = regmap_update_bits(chip->regmap, SY7808_REG_STROBE,
					 SY7808_FLASH_I_LED3_MASK,
					 clamp(level_ua / SY7808_TORCH_STEP_UA,
					       1U, SY7808_TORCH_STEPS_MAX) - 1);
	mutex_unlock(&chip->mutex);

	return ret;
}

static void sy7808_strobe_off_work(struct work_struct *w)
{
	struct delayed_work *dw = to_delayed_work(w);
	struct sy7808 *chip = container_of(dw, struct sy7808, strobe_off_work);
	int ret;

	mutex_lock(&chip->mutex);
	ret = regmap_update_bits(chip->regmap, SY7808_REG_ENABLE,
				 SY7808_MODE_MASK | SY7808_CH_MASK, 0);
	if (!ret)
		ret = regmap_update_bits(chip->regmap, SY7808_REG_STROBE,
					 SY7808_LED3_EN, 0);
	chip->strobe_active = false;
	mutex_unlock(&chip->mutex);

	if (ret)
		dev_err(chip->dev, "strobe auto-off failed: %d\n", ret);
}

static int sy7808_strobe_set(struct led_classdev_flash *fled, bool state)
{
	struct sy7808 *chip = flcdev_to_sy7808(fled);
	int ret;

	mutex_lock(&chip->mutex);

	if (state) {
		/* stock enable order: third-sink arm (0x05 bit7) first, then
		 * flash mode + channels in 01 (flash auto-clears without bit7) */
		ret = regmap_update_bits(chip->regmap, SY7808_REG_STROBE,
					 SY7808_LED3_EN, SY7808_LED3_EN);
		if (ret)
			goto unlock;
		ret = regmap_update_bits(chip->regmap, SY7808_REG_ENABLE,
					 SY7808_MODE_MASK | SY7808_CH_MASK,
					 SY7808_MODE_FLASH | SY7808_CH_ALL);
		if (ret)
			goto unlock;

		chip->strobe_active = true;
		schedule_delayed_work(&chip->strobe_off_work,
				      msecs_to_jiffies(chip->strobe_timeout_ms));
	} else {
		cancel_delayed_work_sync(&chip->strobe_off_work);
		ret = regmap_update_bits(chip->regmap, SY7808_REG_ENABLE,
					 SY7808_MODE_MASK | SY7808_CH_MASK, 0);
		if (!ret)
			ret = regmap_update_bits(chip->regmap, SY7808_REG_STROBE,
						 SY7808_LED3_EN, 0);
		chip->strobe_active = false;
	}

unlock:
	mutex_unlock(&chip->mutex);
	return ret;
}

static int sy7808_strobe_get(struct led_classdev_flash *fled, bool *state)
{
	struct sy7808 *chip = flcdev_to_sy7808(fled);

	mutex_lock(&chip->mutex);
	*state = chip->strobe_active;
	mutex_unlock(&chip->mutex);

	return 0;
}

static int sy7808_timeout_set(struct led_classdev_flash *fled, u32 timeout)
{
	struct sy7808 *chip = flcdev_to_sy7808(fled);
	struct led_flash_setting *s = &fled->timeout;

	chip->strobe_timeout_ms = timeout / USEC_PER_MSEC;
	s->val = chip->strobe_timeout_ms * USEC_PER_MSEC;

	return 0;
}

static const struct led_flash_ops sy7808_flash_ops = {
	.flash_brightness_set = sy7808_flash_brightness_set,
	.strobe_set = sy7808_strobe_set,
	.strobe_get = sy7808_strobe_get,
	.timeout_set = sy7808_timeout_set,
};

static void sy7808_init_flash_settings(struct sy7808 *chip)
{
	struct led_flash_setting *s;

	s = &chip->fled.brightness;
	s->min = 1;
	s->max = clamp(chip->flash_max_ua / SY7808_FLASH_STEP_UA, 1U,
		       SY7808_FLASH_STEPS_MAX);
	s->step = 1;
	s->val = s->min;

	s = &chip->fled.timeout;
	s->min = 32000;
	s->max = chip->strobe_timeout_ms * USEC_PER_MSEC;
	s->step = 1000;
	s->val = s->max;
}

static int sy7808_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct led_init_data init_data = {};
	struct fwnode_handle *child = NULL;
	struct fwnode_handle *alt_child = NULL;
	struct sy7808 *chip;
	int ret;

	device_for_each_child_node(dev, alt_child) {
		if (!child)
			child = alt_child;
		else
			fwnode_handle_put(alt_child);
	}
	if (!child)
		return dev_err_probe(dev, -EINVAL, "No LED node\n");

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	mutex_init(&chip->mutex);
	INIT_DELAYED_WORK(&chip->strobe_off_work, sy7808_strobe_off_work);

	chip->flash_max_ua = 1000000;
	chip->torch_max_ua = 300000;
	chip->strobe_timeout_ms = SY7808_TIMEOUT_DEFAULT_MS;
	device_property_read_u32(dev, "flash-max-microamp", &chip->flash_max_ua);
	device_property_read_u32(dev, "led-max-microamp", &chip->torch_max_ua);
	/* timeout limit: prefer the led child's flash-max-timeout-ms */
	fwnode_property_read_u32(child, "flash-max-timeout-ms",
				 &chip->strobe_timeout_ms);
	chip->strobe_timeout_ms = min(chip->strobe_timeout_ms, SY7808_TIMEOUT_MAX_MS);

	chip->regmap = devm_regmap_init_i2c(client, &sy7808_regmap_config);
	if (IS_ERR(chip->regmap)) {
		fwnode_handle_put(child);
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				     "Failed to allocate register map\n");
	}

	/* shared EN line (stock FLASH_STROBE gpio49): first probed chip
	 * claims and raises it; the second finds it already high. */
	chip->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	ret = PTR_ERR_OR_ZERO(chip->enable_gpio);
	if (ret) {
		fwnode_handle_put(child);
		return dev_err_probe(dev, ret, "Failed to request enable gpio\n");
	}
	if (chip->enable_gpio)
		usleep_range(200, 300);

	ret = sy7808_chip_init(chip);
	if (ret) {
		fwnode_handle_put(child);
		return dev_err_probe(dev, ret, "Chip init failed\n");
	}

	chip->fled.led_cdev.max_brightness =
		clamp(chip->torch_max_ua / SY7808_TORCH_STEP_UA, 1U,
		      SY7808_TORCH_STEPS_MAX);
	chip->fled.led_cdev.brightness_set_blocking = sy7808_torch_brightness_set;
	chip->fled.led_cdev.flags |= LED_DEV_CAP_FLASH;
	chip->fled.ops = &sy7808_flash_ops;
	sy7808_init_flash_settings(chip);

	init_data.fwnode = child;
	ret = devm_led_classdev_flash_register_ext(dev, &chip->fled, &init_data);
	fwnode_handle_put(child);
	if (ret)
		return dev_err_probe(dev, ret, "Couldn't register flash led\n");

	i2c_set_clientdata(client, chip);
	dev_info(dev, "SY7808 flash led registered (torch max %u, flash max %u steps)\n",
		 chip->fled.led_cdev.max_brightness, chip->fled.brightness.max);

	return 0;
}

static void sy7808_remove(struct i2c_client *client)
{
	struct sy7808 *chip = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&chip->strobe_off_work);
	/* full off like stock disable tail */
	regmap_write(chip->regmap, SY7808_REG_ENABLE, 0x00);
	regmap_write(chip->regmap, SY7808_REG_STROBE, 0x00);
}

static const struct of_device_id sy7808_leds_match[] = {
	{ .compatible = "silergy,sy7808" },
	{ .compatible = "si,sy7808" },
	{}
};
MODULE_DEVICE_TABLE(of, sy7808_leds_match);

static struct i2c_driver sy7808_driver = {
	.driver = {
		.name = "sy7808",
		.of_match_table = sy7808_leds_match,
	},
	.probe = sy7808_probe,
	.remove = sy7808_remove,
};
module_i2c_driver(sy7808_driver);

MODULE_AUTHOR("nura-meizu-m2381");
MODULE_DESCRIPTION("Silergy SY7808 flash LED driver");
MODULE_LICENSE("GPL");
