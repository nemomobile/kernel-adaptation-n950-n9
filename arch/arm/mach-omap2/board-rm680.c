/*
 * Board support file for Nokia RM-680/696.
 *
 * Copyright (C) 2010 Nokia
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <plat/mux.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c/twl.h>
#include <linux/input/atmel_mxt.h>
#include <linux/input/eci.h>
#include <linux/platform_device.h>
#include <linux/omapfb.h>
#include <linux/opp.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/wl12xx.h>
#include <linux/spi/spi.h>
#include <linux/spi/vibra.h>
#include <linux/cpu.h>
#include <linux/opp.h>
#include <linux/hsi/hsi.h>
#include <linux/cmt.h>
#include <linux/lis3lv02d.h>
#include <linux/leds-lp5521.h>
#include <linux/usb/musb.h>
#include <linux/mfd/wl1273-core.h>
#include <linux/mfd/aci.h>
#include <sound/tpa6130a2-plat.h>
#include <sound/tlv320dac33-plat.h>
#include <linux/i2c/apds990x.h>
#include <linux/i2c/ak8975.h>
#include <linux/nfc/pn544.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/system_info.h>

#include <plat/i2c.h>
#include <plat/mmc.h>
#include <plat/usb.h>
#include <plat/gpmc.h>
#include "common.h"
#include <plat/onenand.h>
#include <plat/display.h>
#include <plat/panel-nokia-dsi.h>
#include <plat/vram.h>
#include <plat/board-nokia.h>
#include <plat/omap-pm.h>

#include <linux/pvr.h>
#include <linux/printk.h>
#include <plat/mcspi.h>
#include <plat/omap_device.h>
#include <plat/ssi.h>

#include "mux.h"
#include "hsmmc.h"
#include "sdram-nokia.h"
#include "common-board-devices.h"
#include "atmel_mxt_config.h"
#include "pm.h"

#include "dss.h"

#if defined(CONFIG_SND_OMAP_SOC_DFL61_TWL4030) || \
	defined(CONFIG_SND_OMAP_SOC_DFL61_TWL4030_MODULE)
#include <plat/dfl61-audio.h>
#include <linux/mfd/twl4030-audio.h>
#endif

#define ATMEL_MXT_IRQ_GPIO		61
#define ATMEL_MXT_RESET_GPIO		81

#define LIS302_IRQ1_GPIO 180
#define LIS302_IRQ2_GPIO 181
#define RM696_FM_RESET_GPIO1 179
#define RM696_FM_RESET_GPIO2 178
#define RM696_FMRX_IRQ_GPIO 43

#define NFC_HOST_INT_GPIO 76
#define NFC_ENABLE_GPIO 77
#define NFC_FW_RESET_GPIO 78

#define RM696_DAC33_RESET_GPIO 60
#define RM696_DAC33_IRQ_GPIO 53

#define RM696_VIBRA_POWER_GPIO 182
#define RM696_VIBRA_POWER_UP_TIME 1000 /* usecs */

#define RM696_TVOUT_EN_GPIO	40
#define RM696_JACK_GPIO		(OMAP_MAX_GPIO_LINES + 0)

#define RM696_LP5521_CHIP_EN_GPIO 41

#define APDS990X_GPIO 83

static uint32_t rm696_keymap[] = {
	/* row, col, event */
	KEY(6, 8, KEY_VOLUMEUP),
	KEY(7, 8, KEY_VOLUMEDOWN),
};

static struct matrix_keymap_data rm696_keymap_data = {
	.keymap			= rm696_keymap,
	.keymap_size		= ARRAY_SIZE(rm696_keymap),
};

static struct twl4030_keypad_data rm696_kp_data = {
	.keymap_data 	= &rm696_keymap_data,
	.rows		= 8, /* Two last rows are used */
	.cols		= 8,
	.rep		= 1,
};

/* CMT init data */
static struct cmt_platform_data rm696_cmt_pdata = {
	.cmt_rst_ind_gpio = 34,
	.cmt_rst_ind_flags = IRQF_TRIGGER_RISING,
};

static struct platform_device rm696_cmt_device = {
	.name = "cmt",
	.id = -1,
	.dev = {
		.platform_data = &rm696_cmt_pdata,
	},
};

static struct eci_platform_data rm696_eci_platform_data = {
#if	defined(CONFIG_SND_OMAP_SOC_DFL61_TWL4030) || \
	defined(CONFIG_SND_OMAP_SOC_DFL61_TWL4030_MODULE)
	.register_hsmic_event_cb = dfl61_register_hsmic_event_cb,
	.jack_report = dfl61_jack_report,
#endif
};

static struct platform_device rm696_eci_device = {
	.name	= "ECI_accessory",
	.dev	= {
		.platform_data = &rm696_eci_platform_data,
	},
};

/* SSI init data */
static struct omap_ssi_port_config __initdata rm696_ssi_port_config[] = {
	[0] =	{
		.cawake_gpio	= 151,
		.ready_rx_gpio	= 154,
		},
};

static struct omap_ssi_board_config __initdata rm696_ssi_config = {
	.num_ports = ARRAY_SIZE(rm696_ssi_port_config),
	.port_config = rm696_ssi_port_config,
};

static struct hsi_board_info __initdata rm696_ssi_cl[] = {
	[0] =	{
		.name = "hsi_char",
		.hsi_id = 0,
		.port = 0,
		},
	[1] =	{
		.name = "ssi_protocol",
		.hsi_id = 0,
		.port = 0,
		.tx_cfg = {
			.mode = HSI_MODE_FRAME,
			.channels = 4,
			.speed = 96000,
			.arb_mode = HSI_ARB_RR,
			},
		.rx_cfg = {
			.mode = HSI_MODE_FRAME,
			.channels = 4,
			},
		},
	[2] =	{
		.name = "cmt_speech",
		.hsi_id = 0,
		.port = 0,
		},

};

static void __init rm696_ssi_init(void)
{
	omap_ssi_config(&rm696_ssi_config);
	hsi_register_board_info(rm696_ssi_cl, ARRAY_SIZE(rm696_ssi_cl));
}

/* CPU table initialization */
static int __init rm696_opp_init(void)
{
	int r = 0;
	struct device *mpu_dev, *iva_dev;

	r = omap3_opp_init();
	if (IS_ERR_VALUE(r) && (r != -EEXIST)) {
		pr_err("opp default init failed\n");
		return r;
	}

	mpu_dev = omap_device_get_by_hwmod_name("mpu");
	iva_dev = omap_device_get_by_hwmod_name("iva");
	if (IS_ERR(mpu_dev) || IS_ERR(iva_dev)) {
		pr_err("no mpu_dev/iva_dev error\n");
		return -ENODEV;
	}

	/* Enable MPU 800MHz and lower opps */
	r = opp_enable(mpu_dev, 800000000);
	if (r)
		pr_err("failed to enable higher (800MHz) opp\n");

	/* Enable MPU 1GHz and lower opps */
	r = opp_enable(mpu_dev, 1000000000);
	if (r)
		pr_err("failed to enable higher (1GHz) opp\n");

	r = opp_enable(iva_dev, 660000000);
	if (r)
		pr_err("failed to enable higher (660MHz) opp on DSP\n");
	
	r = opp_enable(iva_dev, 800000000);
	if (r)
		pr_err("failed to enable higher (800MHz) opp on DSP\n");

	return 0;
}
device_initcall(rm696_opp_init);


/* WL1271 SDIO/SPI */
#define RM696_WL1271_POWER_GPIO		35
#define RM696_WL1271_IRQ_GPIO		42
#define	RM696_WL1271_REF_CLOCK		2

static void rm696_wl1271_set_power(bool enable)
{
	gpio_set_value(RM696_WL1271_POWER_GPIO, enable);
}

static struct wl12xx_platform_data wl1271_pdata = {
	.set_power = rm696_wl1271_set_power,
	.board_ref_clock = RM696_WL1271_REF_CLOCK,
};

static inline bool board_is_rm680(void)
{
	return (system_rev & 0x00f0) == 0x0020;
}

static bool board_has_sdio_wlan(void)
{
	/* RM-696 - N950 using SPI */
	if (board_is_rm680())
		return false;

	return system_rev > 0x0301;
}

/* SPI for wl1271 */
static struct omap2_mcspi_device_config wl1271_mcspi_config = {
	.turbo_mode = 1,
};

static struct spi_board_info rm696_peripherals_spi_board_info[] = {
	[0] = {
		.modalias		= "wl1271_spi",
		.bus_num		= 4,
		.chip_select		= 0,
		.max_speed_hz		= 48000000,
		.mode			= SPI_MODE_0,
		.controller_data	= &wl1271_mcspi_config,
		.platform_data		= &wl1271_pdata,
	},
};

/* SDIO fixed regulator for WLAN */
static struct regulator_consumer_supply rm696_vsdio_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.2"),
};

static struct regulator_init_data rm696_vsdio_data = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_MODE,
	},
	.num_consumer_supplies	= ARRAY_SIZE(rm696_vsdio_consumers),
	.consumer_supplies	= rm696_vsdio_consumers,
};

static struct fixed_voltage_config rm696_vsdio_config = {
	.supply_name		= "vwl1271",
	.microvolts		= 1800000,
	.gpio			= RM696_WL1271_POWER_GPIO,
	.startup_delay		= 1000,
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data		= &rm696_vsdio_data,
};

static struct platform_device rm696_vsdio_device = {
	.name			= "reg-fixed-voltage",
	.id			= 1,
	.dev			= {
		.platform_data	= &rm696_vsdio_config,
	},
};


/* Fixed regulator for internal eMMC */
static struct regulator_consumer_supply rm680_vemmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.1"),
};

static struct regulator_init_data rm680_vemmc = {
	.constraints =	{
		.name			= "rm680_vemmc",
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_MODE,
	},
	.num_consumer_supplies		= ARRAY_SIZE(rm680_vemmc_consumers),
	.consumer_supplies		= rm680_vemmc_consumers,
};

static struct fixed_voltage_config rm680_vemmc_config = {
	.supply_name		= "VEMMC",
	.microvolts		= 2900000,
	.gpio			= 157,
	.startup_delay		= 150,
	.enable_high		= 1,
	.init_data		= &rm680_vemmc,
};

static struct platform_device rm680_vemmc_device = {
	.name			= "reg-fixed-voltage",
	.dev			= {
		.platform_data	= &rm680_vemmc_config,
	},
};

static void __init rm680_init_wl1271(void)
{
	int irq, ret;

	if (board_has_sdio_wlan()) {
		pr_info("wl1271 SDIO\n");
		platform_device_register(&rm696_vsdio_device);

		ret  = gpio_request(RM696_WL1271_IRQ_GPIO, "wl1271 irq");
		if (ret < 0)
			goto sdio_err;

		ret = gpio_direction_input(RM696_WL1271_IRQ_GPIO);
		if (ret < 0)
			goto sdio_err_irq;

		irq = gpio_to_irq(RM696_WL1271_IRQ_GPIO);
		if (ret < 0)
			goto sdio_err_irq;

		wl1271_pdata.irq = irq;
		wl12xx_set_platform_data(&wl1271_pdata);

		/* Set high power gpio - mmc3 need to be detected.
		   Next wl12xx driver will set this low */
		rm696_wl1271_set_power(true);

		omap_mux_init_signal("sdmmc2_dat4.sdmmc3_dat0",
				     OMAP_PIN_INPUT_PULLUP);
		omap_mux_init_signal("sdmmc2_dat5.sdmmc3_dat1",
				     OMAP_PIN_INPUT_PULLUP);
		omap_mux_init_signal("sdmmc2_dat6.sdmmc3_dat2",
				     OMAP_PIN_INPUT_PULLUP);
		omap_mux_init_signal("sdmmc2_dat7.sdmmc3_dat3",
				     OMAP_PIN_INPUT_PULLUP);

		return;
sdio_err:
		gpio_free(RM696_WL1271_IRQ_GPIO);
sdio_err_irq:
		pr_err("wl1271 sdio board initialisation failed\n");
		wl1271_pdata.set_power = NULL;
	} else {
		pr_info("wl1271 SPI\n");

		ret = gpio_request(RM696_WL1271_POWER_GPIO, "wl1271 power");
		if (ret < 0)
			goto spi_err;

		ret = gpio_direction_output(RM696_WL1271_POWER_GPIO, 0);
		if (ret < 0)
			goto spi_err_power;

		ret = gpio_request(RM696_WL1271_IRQ_GPIO, "wl1271 irq");
		if (ret < 0)
			goto spi_err_power;

		ret = gpio_direction_input(RM696_WL1271_IRQ_GPIO);
		if (ret < 0)
			goto spi_err_irq;

		irq = gpio_to_irq(RM696_WL1271_IRQ_GPIO);
		if (irq < 0)
			goto spi_err_irq;

		rm696_peripherals_spi_board_info[0].irq = irq;

		spi_register_board_info(rm696_peripherals_spi_board_info,
				ARRAY_SIZE(rm696_peripherals_spi_board_info));

		return;
spi_err_irq:
		gpio_free(RM696_WL1271_IRQ_GPIO);
spi_err_power:
		gpio_free(RM696_WL1271_POWER_GPIO);
spi_err:
		pr_err("wl1271 spi board initialisation failed\n");
		wl1271_pdata.set_power = NULL;

	}
}

/* GPIO0 AvPlugDet */
#define TWL_GPIOS_HIGH	BIT(0)
#define TWL_GPIOS_LOW	(BIT(2) | BIT(6) | BIT(8) | BIT(13) | BIT(15))

static int twlgpio_setup(struct device *dev, unsigned gpio, unsigned n)
{
	int err;

	err = gpio_request(gpio + 1, "TMP303_SOH");
	if (err) {
		printk(KERN_ERR "twl4030_gpio: gpio request failed\n");
		goto out;
	}

	err = gpio_direction_output(gpio + 1, 0);
	if (err)
		printk(KERN_ERR "twl4030_gpio: set gpio direction failed\n");

out:
	return 0;
}

static int twlgpio_teardown(struct device *dev, unsigned gpio, unsigned n)
{
	gpio_free(gpio + 1);
	return 0;
}

/* TWL */
static struct twl4030_gpio_platform_data rm680_gpio_data = {
	.gpio_base		= OMAP_MAX_GPIO_LINES,
	.irq_base		= TWL4030_GPIO_IRQ_BASE,
	.irq_end		= TWL4030_GPIO_IRQ_END,
	.pullups		= TWL_GPIOS_HIGH,
	.pulldowns		= TWL_GPIOS_LOW,
	.setup			= twlgpio_setup,
	.teardown		= twlgpio_teardown,
};

static struct twl4030_codec_data rm680_codec_data;

static struct twl4030_audio_data rm680_audio_data = {
	.audio_mclk = 38400000,
	.codec = &rm680_codec_data,	
	/*FIXME: vibra*/
};

static struct regulator_consumer_supply rm696_vio_consumers[] = {
	REGULATOR_SUPPLY("DVDD", "2-0019"),	/* TLV320DAC33 */
	REGULATOR_SUPPLY("IOVDD", "2-0019"),	/* TLV320DAC33 */
	REGULATOR_SUPPLY("VDDI", "display0"),	/* Himalaya */
	REGULATOR_SUPPLY("Vdd", "2-004b"),	/* Atmel mxt */
	REGULATOR_SUPPLY("vonenand", "omap2-onenand"), /* OneNAND flash */
	REGULATOR_SUPPLY("Vdd_IO", "3-001d"),	/* LIS302 */
	REGULATOR_SUPPLY("DVdd", "3-000f"),	/* AK8975 */
	REGULATOR_SUPPLY("Vdd_IO", "3-002b"),	/* PN544 */
	REGULATOR_SUPPLY("vmmc_aux", "mmci-omap-hs.1"),
	REGULATOR_SUPPLY("Vbat", "3-01fa"),	/* BCM4751_GPS */
	REGULATOR_SUPPLY("Vddio", "3-01fa"),	/* BCM4751_GPS */
};

static struct regulator_init_data rm696_vio_data = {
	.constraints =	{
		.name			= "rm696_vio",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_MODE,
	},
	.num_consumer_supplies		= ARRAY_SIZE(rm696_vio_consumers),
	.consumer_supplies		= rm696_vio_consumers,
};

static struct regulator_consumer_supply rm696_vsim_consumers[] = {
	REGULATOR_SUPPLY("VSim", "3-002b"),	/* PN544 */
};

static struct regulator_init_data rm696_vsim_data = {
	.constraints =	{
		.name			= "rm696_vsim",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_MODE,
	},
	.num_consumer_supplies		= ARRAY_SIZE(rm696_vsim_consumers),
	.consumer_supplies		= rm696_vsim_consumers,
};

static struct regulator_consumer_supply rm696_vbat_consumers[] = {
	REGULATOR_SUPPLY("Vled", "2-0039"),	/* APDS990x */
	REGULATOR_SUPPLY("AVdd", "2-0060"),	/* TPA6140A2 */
	REGULATOR_SUPPLY("VBat", "3-002b"),	/* PN544 */
};

static struct regulator_init_data rm696_vbat_data = {
	.num_consumer_supplies	= ARRAY_SIZE(rm696_vbat_consumers),
	.consumer_supplies	= rm696_vbat_consumers,
	.constraints		= {
		.always_on	= 1,
	},
};

static struct fixed_voltage_config rm696_vbat_config = {
	.supply_name = "vbat",
	.microvolts = 3700000,
	.gpio = -1,
	.init_data = &rm696_vbat_data,
};

static struct platform_device rm696_vbat = {
	.name			= "reg-fixed-voltage",
	.id			= -1,
	.dev			= {
		.platform_data	= &rm696_vbat_config,
	},
};

/*
 * According to public N9 schematics VPNL comes from battery not from
 * TWL MMC2
 */
static struct regulator_consumer_supply rm696_vmmc2_consumers[] = {
	REGULATOR_SUPPLY("VPNL", "display0"),	/* Himalaya */
};

static struct regulator_init_data rm696_vmmc2_data = {
	.constraints =	{
		.name			= "rm696_vmmc2",
		.min_uV			= 3000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_MODE,
	},
	.num_consumer_supplies		= ARRAY_SIZE(rm696_vmmc2_consumers),
	.consumer_supplies		= rm696_vmmc2_consumers,
};

static struct regulator_consumer_supply rm696_vaux1_consumers[] = {
	REGULATOR_SUPPLY("AVdd", "2-004b"),	/* Atmel mxt */
	REGULATOR_SUPPLY("Vdd", "3-001d"),	/* LIS302 */
	REGULATOR_SUPPLY("v28", "twl5031_aci"),
	REGULATOR_SUPPLY("Vdd", "2-0039"),	/* APDS990x */
	REGULATOR_SUPPLY("AVdd", "3-000f"),	/* AK8975 */
};

static struct regulator_init_data rm696_vaux1_data = {
	.constraints = {
		.name			= "rm696_vaux1",
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies		= ARRAY_SIZE(rm696_vaux1_consumers),
	.consumer_supplies		= rm696_vaux1_consumers,
};

static struct regulator_consumer_supply rm696_vaux2_consumers[] = {
	REGULATOR_SUPPLY("VDD_CSIPHY1", "omap3isp"),	/* OMAP ISP */
	REGULATOR_SUPPLY("VDD_CSIPHY2", "omap3isp"),	/* OMAP ISP */
};

static struct regulator_init_data rm696_vaux2_data = {
	.constraints = {
		.name			= "rm696_vaux2",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies		= ARRAY_SIZE(rm696_vaux2_consumers),
	.consumer_supplies		= rm696_vaux2_consumers,
};

static struct regulator_consumer_supply rm696_vaux3_consumers[] = {
	REGULATOR_SUPPLY("VANA", "2-0037"),	/* Main Camera Sensor */
	REGULATOR_SUPPLY("VANA", "2-000e"),	/* Main Camera Lens */
	REGULATOR_SUPPLY("VANA", "2-0010"),	/* Front Camera */
};

static struct regulator_init_data rm696_vaux3_data = {
	.constraints = {
		.name			= "rm696_vaux3",
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies		= ARRAY_SIZE(rm696_vaux3_consumers),
	.consumer_supplies		= rm696_vaux3_consumers,
};

static struct regulator_consumer_supply rm696_vaux4_consumers[] = {
	REGULATOR_SUPPLY("AVDD", "2-0019"),	/* TLV320DAC33 */
};

static struct regulator_init_data rm696_vaux4_data = {
	.constraints = {
		.name			= "rm696_vaux4",
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies		= ARRAY_SIZE(rm696_vaux4_consumers),
	.consumer_supplies		= rm696_vaux4_consumers,
};

static struct platform_device *rm680_peripherals_devices[] __initdata = {
	&rm696_vbat,
	&rm680_vemmc_device,
	&rm696_cmt_device,
	&rm696_eci_device,
};



/* ACI */
static struct regulator *rm696plug_regulator;
static bool rm696plug_regulator_enabled;
static int rm696_plug_resource_reserve(struct device *dev)
{
	rm696plug_regulator = regulator_get(dev, "v28");
	if (IS_ERR(rm696plug_regulator)) {
		dev_err(dev, "Unable to get v28 regulator for plug switch");
		return PTR_ERR(rm696plug_regulator);
	}

	rm696plug_regulator_enabled = 0;
	return 0;
}

static int rm696_plug_set_state(struct device *dev, bool plugged)
{
	int ret;

	if (rm696plug_regulator_enabled == plugged)
		return 0;

	if (plugged) {
		ret = regulator_enable(rm696plug_regulator);
		if (ret)
			dev_err(dev, "Failed to enable v28 regulator");
		else
			rm696plug_regulator_enabled = 1;
	} else {
		ret = regulator_disable(rm696plug_regulator);
		if (ret)
			dev_err(dev, "Failed to disable v28 regulator");
		else
			rm696plug_regulator_enabled = 0;
	}

	return ret;
}

static void rm696_plug_resource_release(void)
{
	if (rm696plug_regulator_enabled)
		regulator_disable(rm696plug_regulator);

	regulator_put(rm696plug_regulator);
	rm696plug_regulator = NULL;
}

static struct twl5031_aci_platform_data rm696_aci_data = {
	.tvout_gpio			= RM696_TVOUT_EN_GPIO,
	.jack_gpio			= RM696_JACK_GPIO,
	.avplugdet_plugged		= AVPLUGDET_WHEN_PLUGGED_HIGH,
	.hw_plug_set_state		= rm696_plug_set_state,
	.hw_plug_resource_reserve	= rm696_plug_resource_reserve,
	.hw_plug_resource_release	= rm696_plug_resource_release,
};

static struct twl4030_platform_data rm680_twl_data = {
	.gpio			= &rm680_gpio_data,
	.audio			= &rm680_audio_data,
	.aci			= &rm696_aci_data,
	.keypad			= &rm696_kp_data,
	/* add rest of the children here */
	/* LDOs */
	.vio			= &rm696_vio_data,
	.vmmc2			= &rm696_vmmc2_data,
	.vsim			= &rm696_vsim_data,
	.vaux1			= &rm696_vaux1_data,
	.vaux2			= &rm696_vaux2_data,
	.vaux3			= &rm696_vaux3_data,
	.vaux4			= &rm696_vaux4_data,
};

#if defined(CONFIG_RADIO_WL1273) || defined(CONFIG_RADIO_WL1273_MODULE)

static unsigned int wl1273_fm_reset_gpio;

static int rm696_wl1273_fm_request_resources(struct i2c_client *client)
{
	if (gpio_request(RM696_FMRX_IRQ_GPIO, "wl1273_fm irq gpio") < 0) {
		dev_err(&client->dev, "Request IRQ GPIO fails.\n");
		return -1;
	}

	if (system_rev < 0x0501)
		wl1273_fm_reset_gpio = RM696_FM_RESET_GPIO1;
	else
		wl1273_fm_reset_gpio = RM696_FM_RESET_GPIO2;

	if (gpio_request(wl1273_fm_reset_gpio, "wl1273_fm reset gpio") < 0) {
		dev_err(&client->dev, "Request for GPIO %d fails.\n",
			wl1273_fm_reset_gpio);
		return -1;
	}

	if (gpio_direction_output(wl1273_fm_reset_gpio, 0)) {
		dev_err(&client->dev, "Set GPIO Direction fails.\n");
		return -1;
	}

	client->irq = gpio_to_irq(RM696_FMRX_IRQ_GPIO);

	return 0;
}

static void rm696_wl1273_fm_free_resources(void)
{
	gpio_free(RM696_FMRX_IRQ_GPIO);
	gpio_free(wl1273_fm_reset_gpio);
}

static void rm696_wl1273_fm_enable(void)
{
	gpio_set_value(wl1273_fm_reset_gpio, 1);
}

static void rm696_wl1273_fm_disable(void)
{
	gpio_set_value(wl1273_fm_reset_gpio, 0);
}

static struct wl1273_fm_platform_data rm696_fm_data = {
	.request_resources = rm696_wl1273_fm_request_resources,
	.free_resources = rm696_wl1273_fm_free_resources,
	.enable = rm696_wl1273_fm_enable,
	.disable = rm696_wl1273_fm_disable,
#if defined(CONFIG_SND_SOC_WL1273) || defined(CONFIG_SND_SOC_WL1273_MODULE)
	.children = WL1273_CODEC_CHILD | WL1273_RADIO_CHILD,
#else
	.children = WL1273_RADIO_CHILD,
#endif
};

static struct platform_device rm696_wl1273_core_device = {
	.name		= "dfl61audio-wl1273",
	.id		= -1,
	.dev		= {
		.platform_data = &rm696_fm_data,
	},
};

static void __init rm696_wl1273_init(void)
{
	platform_device_register(&rm696_wl1273_core_device);
}
#else

static void __init rm696_wl1273_init(void)
{
}
#endif

static struct mxt_platform_data atmel_mxt_platform_data = {
	.reset_gpio = ATMEL_MXT_RESET_GPIO,
	.int_gpio = ATMEL_MXT_IRQ_GPIO,
	.rlimit_min_interval_us = 7000,
	.rlimit_bypass_time_us = 25000,
	.wakeup_interval_ms = 50,
	.config = &atmel_mxt_pyrenees_config,
};

#if	defined(CONFIG_SND_SOC_TLV320DAC33) || \
	defined(CONFIG_SND_SOC_TLV320DAC33_MODULE)
static struct tlv320dac33_platform_data rm696_dac33_platform_data = {
	.power_gpio = RM696_DAC33_RESET_GPIO,
	.mode1_latency = 10000, /* 10ms */
	/*FIXME there is no mode7lp_latency & fallback_to_bypass_time*/
	/*.mode7lp_latency = 10000,*/ /* 10ms */ 
	/*.fallback_to_bypass_time = 40000,*/ /* 40ms */
	.auto_fifo_config = 1,
	.keep_bclk = 1,
	.burst_bclkdiv = 3,
};
#endif


#if	defined(CONFIG_SND_SOC_TPA6130A2) || \
	defined(CONFIG_SND_SOC_TPA6130A2_MODULE)
/* We don't have GPIO allocated for the TPA6130A2 amplifier */
static struct tpa6130a2_platform_data rm696_tpa6130a2_platform_data = {
	.power_gpio = -1,
};
#endif

#if defined(CONFIG_LEDS_LP5521) || defined(CONFIG_LEDS_LP5521_MODULE)
#define RM696_LED_MAX_CURR 130 /* 13 mA */
#define RM696_LED_DEF_CURR 50 /* 5.0 mA */
static struct lp5521_led_config rm696_lp5521_led_config[] = {
	{
		.chan_nr	= 0,
		.led_current	= RM696_LED_DEF_CURR,
		.max_current	= RM696_LED_MAX_CURR,
	},
	{
		.chan_nr	= 1,
		.led_current	= 0,
        },
	{
		.chan_nr	= 2,
		.led_current	= 0,
	}
};

static int lp5521_setup(void)
{
        int err;
        int gpio = RM696_LP5521_CHIP_EN_GPIO;
        err = gpio_request(gpio, "lp5521_enable");
        if (err) {
                printk(KERN_ERR "lp5521: gpio request failed\n");
                return err;
        }
        gpio_direction_output(gpio, 0);
        return 0;
}

static void lp5521_release(void)
{
        gpio_free(RM696_LP5521_CHIP_EN_GPIO);
}

static void lp5521_enable(bool state)
{
        gpio_set_value(RM696_LP5521_CHIP_EN_GPIO, !!state);
}

static struct lp5521_platform_data rm696_lp5521_platform_data = {
	.led_config		= rm696_lp5521_led_config,
	.num_channels		= ARRAY_SIZE(rm696_lp5521_led_config),
	.clock_mode		= LP5521_CLOCK_EXT,
	.setup_resources	= lp5521_setup,
	.release_resources	= lp5521_release,
	.enable			= lp5521_enable,
	.update_config		= LP5521_PWRSAVE_EN | LP5521_CP_MODE_OFF | LP5521_R_TO_BATT,
};
#endif

#if defined(CONFIG_SENSORS_APDS990X) || defined(CONFIG_SENSORS_APDS990X_MODULE)
static int apds990x_setup(void)
{
	int err;

	err = gpio_request_one(APDS990X_GPIO, GPIOF_DIR_IN, "apds990x_irq");
	if (err)
		goto fail;

fail:
	return err;
}

static int apds990x_release(void)
{
	gpio_free(APDS990X_GPIO);
	return 0;
}

static struct apds990x_platform_data rm696_apds990x_data = {
	.cf.ga	 = 168834, /*  41.2194 * 4096 */
	.cf.cf1	 = 4096,
	.cf.irf1 = 7824,  /* 1.9102 * 4096 */
	.cf.cf2	 = 877,  /* 0.2140 * 4096 */
	.cf.irf2 = 1575,  /* 0.3846 * 4096 */
	.cf.df	 = 52,
	.pdrive = APDS_IRLED_CURR_25mA,
	.setup_resources   = apds990x_setup,
	.release_resources = apds990x_release,
};
#endif

static struct i2c_board_info rm696_peripherals_i2c_board_info_2[] /*__initdata */= {
	{
		/* keep this first */
		I2C_BOARD_INFO("atmel_mxt", 0x4b),
		.platform_data	= &atmel_mxt_platform_data,
	},

#if defined(CONFIG_SENSORS_APDS990X) || defined(CONFIG_SENSORS_APDS990X_MODULE)
	{
		/* keep this second */
		I2C_BOARD_INFO("apds990x", 0x39),
		.platform_data = &rm696_apds990x_data,
	},
#endif

#if	defined(CONFIG_SND_SOC_TPA6130A2) || \
	defined(CONFIG_SND_SOC_TPA6130A2_MODULE)
	{
		I2C_BOARD_INFO("tpa6140a2", 0x60),
		.platform_data	= &rm696_tpa6130a2_platform_data,
	},
#endif

#if	defined(CONFIG_SND_SOC_TLV320DAC33) || \
	defined(CONFIG_SND_SOC_TLV320DAC33_MODULE)
	{
		/*keep this third*/
		I2C_BOARD_INFO("tlv320dac33", 0x19),
		.platform_data = &rm696_dac33_platform_data,
	},
#endif
	
#if defined(CONFIG_LEDS_LP5521) || defined(CONFIG_LEDS_LP5521_MODULE)
        {
                I2C_BOARD_INFO("lp5521", 0x32),
                .platform_data = &rm696_lp5521_platform_data,
        },
#endif

};

#if defined(CONFIG_SENSORS_APDS990X) || defined(CONFIG_SENSORS_APDS990X_MODULE)
static void __init rm696_apds990x_init(void)
{
	if (system_rev < 0x0300) {
		rm696_apds990x_data.cf.ga = 19660; /* 0.48 / 10% * 4096 */
		rm696_apds990x_data.cf.irf1 = 7781;  /* 1.8996 * 4096 */
		rm696_apds990x_data.cf.cf2 = 1959;  /* 0.4783 * 4096 */
		rm696_apds990x_data.cf.irf2 = 3669;  /* 0.8957 * 4096 */
		rm696_apds990x_data.pdrive = APDS_IRLED_CURR_50mA;
	} else  if (system_rev < 0x1500) {
		rm696_apds990x_data.cf.ga = 102674; /*  25.067 * 4096 */
		rm696_apds990x_data.cf.irf1 = 7578;  /* 1.8502 * 4096 */
		rm696_apds990x_data.cf.cf2 = 1707;  /* 0.4168 * 4096 */
		rm696_apds990x_data.cf.irf2 = 2975;  /* 0.7264 * 4096 */
		rm696_apds990x_data.pdrive = APDS_IRLED_CURR_50mA;
	}

	rm696_peripherals_i2c_board_info_2[1].irq = gpio_to_irq(APDS990X_GPIO);
}
#else
static inline void rm696_apds990x_init(void) {}
#endif

#if defined(CONFIG_SENSORS_LIS3_I2C) || defined(CONFIG_SENSORS_LIS3_I2C_MODULE)
static int lis302_setup(void)
{
	int err;
	int irq1 = LIS302_IRQ1_GPIO;
	int irq2 = LIS302_IRQ2_GPIO;

	/* gpio for interrupt pin 1 */
	err = gpio_request(irq1, "lis3lv02dl_irq1");
	if (err) {
		printk(KERN_ERR "lis3lv02dl: gpio request failed\n");
		goto out;
	}

	/* gpio for interrupt pin 2 */
	err = gpio_request(irq2, "lis3lv02dl_irq2");
	if (err) {
		gpio_free(irq1);
		printk(KERN_ERR "lis3lv02dl: gpio request failed\n");
		goto out;
	}

	gpio_direction_input(irq1);
	gpio_direction_input(irq2);

out:
	return err;
}

static int lis302_release(void)
{
	gpio_free(LIS302_IRQ1_GPIO);
	gpio_free(LIS302_IRQ2_GPIO);

	return 0;
}

static struct lis3lv02d_platform_data rm696_lis302dl_data = {
	.click_flags	= LIS3_CLICK_SINGLE_X | LIS3_CLICK_SINGLE_Y |
			  LIS3_CLICK_SINGLE_Z,
	/* Limits are 0.5g * value */
	.click_thresh_x = 8,
	.click_thresh_y = 8,
	.click_thresh_z = 10,
	/* Click must be longer than time limit */
	.click_time_limit = 9,
	/* Kind of debounce filter */
	.click_latency	  = 50,

	/* Limits for all axis. millig-value / 18 to get HW values */
	.wakeup_flags = LIS3_WAKEUP_X_HI | LIS3_WAKEUP_Y_HI,
	.wakeup_thresh = 8,
	.wakeup_flags2 =  LIS3_WAKEUP_Z_HI,
	.wakeup_thresh2 = 10,

	.hipass_ctrl = LIS3_HIPASS_CUTFF_2HZ,

	/* Interrupt line 2 for click detection, line 1 for thresholds */
	.irq_cfg = LIS3_IRQ2_CLICK | LIS3_IRQ1_FF_WU_12,

#define LIS3_IRQ1_USE_BOTH_EDGES 1
#define LIS3_IRQ2_USE_BOTH_EDGES 2

	.irq_flags1 = LIS3_IRQ1_USE_BOTH_EDGES,
	.irq_flags2 = LIS3_IRQ2_USE_BOTH_EDGES,
	.duration1 = 8,
	.duration2 = 8,

	.axis_x = LIS3_DEV_X,
	.axis_y = LIS3_INV_DEV_Y,
	.axis_z = LIS3_INV_DEV_Z,
	.setup_resources = lis302_setup,
	.release_resources = lis302_release,
	.st_min_limits = {-46, 3, 3},
	.st_max_limits = {-3, 46, 46},
};

#endif

#if defined(CONFIG_SENSORS_AK8975) || defined(CONFIG_SENSORS_AK8975_MODULE)
static struct ak8975_platform_data rm696_ak8975_data = {
.axis_x = AK8975_DEV_Z,
.axis_y = AK8975_INV_DEV_X,
.axis_z = AK8975_INV_DEV_Y,
};
#endif

#if defined(CONFIG_PN544_NFC) || defined(CONFIG_PN544_NFC_MODULE)
static int rm696_pn544_nfc_request_resources(struct i2c_client *client)
{
	int ret;
	ret = gpio_request(NFC_HOST_INT_GPIO, "NFC INT");
	if (ret) {
		dev_err(&client->dev, "Request NFC INT GPIO fails %d\n", ret);
		return -1;
	}
	ret = gpio_direction_input(NFC_HOST_INT_GPIO);
	if (ret) {
		dev_err(&client->dev, "Set GPIO Direction fails %d\n", ret);
		goto err_int;
	}

	ret = gpio_request(NFC_ENABLE_GPIO, "NFC Enable");
	if (ret) {
		dev_err(&client->dev,
			"Request for NFC Enable GPIO fails %d\n", ret);
		goto err_int;
	}
	ret = gpio_direction_output(NFC_ENABLE_GPIO, 0);
	if (ret) {
		dev_err(&client->dev, "Set GPIO Direction fails %d\n", ret);
		goto err_enable;
	}

	ret = gpio_request(NFC_FW_RESET_GPIO, "NFC FW Reset");
	if (ret) {
		dev_err(&client->dev,
			"Request for NFC FW Reset GPIO fails %d\n", ret);
		goto err_enable;
	}
	ret = gpio_direction_output(NFC_FW_RESET_GPIO, 0);
	if (ret) {
		dev_err(&client->dev, "Set GPIO Direction fails %d\n", ret);
		goto err_fw;
	}

	return 0;
err_fw:
	gpio_free(NFC_FW_RESET_GPIO);
err_enable:
	gpio_free(NFC_ENABLE_GPIO);
err_int:
	gpio_free(NFC_HOST_INT_GPIO);
	return -1;
}

static void rm696_pn544_nfc_free_resources(void)
{
	gpio_free(NFC_HOST_INT_GPIO);
	gpio_free(NFC_ENABLE_GPIO);
	gpio_free(NFC_FW_RESET_GPIO);
}

static void rm696_pn544_nfc_enable(int fw)
{
	gpio_set_value(NFC_FW_RESET_GPIO, fw ? 1 : 0);
	msleep(PN544_GPIO4VEN_TIME);
	gpio_set_value(NFC_ENABLE_GPIO, 1);
}

static int rm696_pn544_nfc_test(void)
{
	int a, b;
	rm696_pn544_nfc_enable(0);
	a = gpio_get_value(NFC_FW_RESET_GPIO);
	rm696_pn544_nfc_enable(1);
	b = gpio_get_value(NFC_FW_RESET_GPIO);

	return (a == 0) && (b == 1);
}

static void rm696_pn544_nfc_disable(void)
{
	gpio_set_value(NFC_ENABLE_GPIO, 0);
}

static struct pn544_nfc_platform_data rm696_nfc_data = {
	.request_resources = rm696_pn544_nfc_request_resources,
	.free_resources = rm696_pn544_nfc_free_resources,
	.enable = rm696_pn544_nfc_enable,
	.test = rm696_pn544_nfc_test,
	.disable = rm696_pn544_nfc_disable,
};
#endif

static struct i2c_board_info rm696_peripherals_i2c_board_info_3[] /*__initdata */= {
#if defined(CONFIG_SENSORS_LIS3_I2C) || defined(CONFIG_SENSORS_LIS3_I2C_MODULE)
	{
		/* Keep this first */
		I2C_BOARD_INFO("lis3lv02d", 0x1d),
		.platform_data = &rm696_lis302dl_data,
	},
#endif

#if defined(CONFIG_PN544_NFC) || defined(CONFIG_PN544_NFC_MODULE)
	{
		/* Keep this second */
		I2C_BOARD_INFO(PN544_DRIVER_NAME, 0x2b),
		.platform_data = &rm696_nfc_data,
	},
#endif

#if defined(CONFIG_RADIO_WL1273) || defined(CONFIG_RADIO_WL1273_MODULE)
	{
		I2C_BOARD_INFO(WL1273_FM_DRIVER_NAME, RX71_FM_I2C_ADDR),
		.platform_data = &rm696_fm_data,
	},
#endif

#if defined(CONFIG_SENSORS_AK8975) || defined(CONFIG_SENSORS_AK8975_MODULE)
	{
		I2C_BOARD_INFO("ak8975", 0x0f),
		.platform_data = &rm696_ak8975_data,
	},
#endif
};

static void rm696_vibra_set_power(bool enable)
{
	gpio_set_value(RM696_VIBRA_POWER_GPIO, enable);
	if (enable)
		usleep_range(RM696_VIBRA_POWER_UP_TIME,
			     RM696_VIBRA_POWER_UP_TIME + 100);
}

static struct vibra_spi_platform_data vibra_pdata = {
	.set_power = rm696_vibra_set_power,
};

static struct omap2_mcspi_device_config spi_vibra_mcspi_config = {
	.turbo_mode	= 1,	
};

static struct spi_board_info rm696_vibra_spi_board_info = {
	.modalias		= "vibra_spi",
	.bus_num		= 2,
	.chip_select		= 0,
	.max_speed_hz   	= 750000,
	.mode			= SPI_MODE_0,
	.controller_data	= &spi_vibra_mcspi_config,
	.platform_data		= &vibra_pdata,
};

static void __init rm696_init_vibra(void)
{
	int ret;

	/* Vibra has been connected to SPI since S1.1 */
	if (system_rev < 0x0501)
		return ;

	ret = gpio_request(RM696_VIBRA_POWER_GPIO, "Vibra amplifier");
	if (ret < 0)
		goto error;

	ret = gpio_direction_output(RM696_VIBRA_POWER_GPIO, 0);
	if (ret < 0)
		goto err_power;

	spi_register_board_info(&rm696_vibra_spi_board_info, 1);
	return ;

err_power:
	gpio_free(RM696_VIBRA_POWER_GPIO);
error:
	printk(KERN_ERR "SPI Vibra board initialisation failed\n");
	vibra_pdata.set_power = NULL;
}

static void __init rm680_i2c_init(void)
{
	struct twl4030_codec_data *codec_data;

	omap3_pmic_get_config(&rm680_twl_data,
			      TWL_COMMON_PDATA_USB |
			      TWL_COMMON_PDATA_MADC |
			      TWL_COMMON_PDATA_BCI,
			      TWL_COMMON_REGULATOR_VDAC |
			      TWL_COMMON_REGULATOR_VPLL2);

	codec_data = rm680_twl_data.audio->codec;
	codec_data->ramp_delay_value = 2;
	codec_data->offset_cncl_path = TWL4030_OFFSET_CNCL_SEL_ARX2;
	codec_data->check_defaults = 0;
	codec_data->reset_registers = 0;
	codec_data->digimic_delay = 0;
	
#if defined(CONFIG_SENSORS_LIS3_I2C) || defined(CONFIG_SENSORS_LIS3_I2C_MODULE)
	rm696_lis302dl_data.irq2 = gpio_to_irq(LIS302_IRQ2_GPIO);
	rm696_peripherals_i2c_board_info_3[0].irq = gpio_to_irq(LIS302_IRQ1_GPIO);
#endif

#if defined(CONFIG_PN544_NFC) || defined(CONFIG_PN544_NFC_MODULE)
	rm696_peripherals_i2c_board_info_3[1].irq = gpio_to_irq(NFC_HOST_INT_GPIO);
#endif

	omap_pmic_init(1, 2900, "twl5031", INT_34XX_SYS_NIRQ, &rm680_twl_data);
	omap_register_i2c_bus(2, 400, rm696_peripherals_i2c_board_info_2,
			      ARRAY_SIZE(rm696_peripherals_i2c_board_info_2));
	omap_register_i2c_bus(3, 400, rm696_peripherals_i2c_board_info_3,
			      ARRAY_SIZE(rm696_peripherals_i2c_board_info_3));
}

#if defined(CONFIG_MTD_ONENAND_OMAP2) || \
	defined(CONFIG_MTD_ONENAND_OMAP2_MODULE)
static struct omap_onenand_platform_data board_onenand_data[] = {
	{
		.cs				= 0,
		.gpio_irq			= 65,
		.flags				= ONENAND_SYNC_READWRITE,
		.regulator_can_sleep		= 1,
		.skip_initial_unlocking		= 1,
	}
};
#endif

static struct omap2_hsmmc_info mmc[] __initdata = {
/* eMMC */
	{
		.name		= "internal",
		.mmc		= 2,
		/* FIXME:
		 * Setting MMC_CAP_8_BIT_DATA here causes wl1271 to break with message:
		 * wl1271_sdio mmc2:0001:2 sdio write failed (-84)
		 */
		.caps		= MMC_CAP_4_BIT_DATA, 
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable	= true,
		.power_saving	= true,
		.no_off		= true,
		.vcc_aux_disable_is_sleep	= true,
		/* FIXME: nomux is present in 2.6 but missing in 3.5 */
		/* .nomux	= 1,*/
	},
/* WLAN */
	{
		.name		= "wl1271",
		.mmc		= 3,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable	= true,
	},
	{ /* Terminator */ }
};

static struct nokia_dsi_panel_data rm696_panel_data = {
	.name = "pyrenees",
	.reset_gpio = 87,
	.use_ext_te = true,
	.ext_te_gpio = 62,
	.esd_timeout = 5000,
	.ulps_timeout = 500,
	.partial_area = {
		.offset = 0,
		.height = 854,
	},
	.rotate = 1,
};

static struct omap_dss_device rm696_dsi_display_data = {
	.type = OMAP_DISPLAY_TYPE_DSI,
	.name = "lcd",
	.driver_name = "panel-nokia-dsi",
	.phy.dsi = {
		.clk_lane = 2,
		.clk_pol = 0,
		.data1_lane = 3,
		.data1_pol = 0,
		.data2_lane = 1,
		.data2_pol = 0,
		.ext_te = true,
		.ext_te_gpio = 62,
	},

	.clocks = {
		.dss = {
			.fck_div = 5,
		},

		.dispc = {
			/* LCK 170.88 MHz */
			.lck_div = 1,
			/* PCK 42.72 MHz */
			.pck_div = 4,

			.fclk_from_dsi_pll = false,
		},

		.dsi = {
			/* DDR CLK 210.24 MHz */
			.regn = 10,
			.regm = 219,
			/* DISPC FCLK 170.88 MHz */
			.regm3 = 6,
			/* DSI FCLK 170.88 MHz */
			.regm4 = 6,

			/* LP CLK 8.760 MHz */
			.lp_clk_div = 8,

			.fclk_from_dsi_pll = false,
		},
	},

	.data = &rm696_panel_data,
};

static int rm696_tv_enable(struct omap_dss_device *dssdev)
{
	if (dssdev->reset_gpio != -1)
		gpio_set_value(dssdev->reset_gpio, 1);

	return 0;
}

static void rm696_tv_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->reset_gpio != -1)
		gpio_set_value(dssdev->reset_gpio, 0);
}

static struct omap_dss_device rm696_tv_display_data = {
	.type = OMAP_DISPLAY_TYPE_VENC,
	.name = "tv",
	.driver_name = "venc",
	/* was 40, handled by twl5031-aci */
	.reset_gpio = -1,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_COMPOSITE,
	.platform_enable = rm696_tv_enable,
	.platform_disable = rm696_tv_disable,
};

static struct omap_dss_device *rm696_dss_devices[] = {
	&rm696_dsi_display_data,
	&rm696_tv_display_data,
};

static struct omap_dss_board_info rm696_dss_data = {
	.num_devices = ARRAY_SIZE(rm696_dss_devices),
	.devices = rm696_dss_devices,
	.default_device = &rm696_dsi_display_data,
};

struct platform_device rm696_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &rm696_dss_data,
	},
};

static struct omapfb_platform_data rm696_omapfb_data = {
	.mem_desc = {
		.region_cnt = 1,
		.region[0] = {
			.format_used = true,
			.format = OMAPFB_COLOR_RGB565,
			.size = PAGE_ALIGN(856 * 512 * 2 * 3),
			.xres_virtual = 856,
			.yres_virtual = 512 * 3,
		}
	}
};

static void rm680_bt_set_pm_limits(struct device *dev, bool set)
{
	omap_pm_set_max_mpu_wakeup_lat(dev, set ? H4P_WAKEUP_LATENCY : -1);
}

static struct omap_bluetooth_config rm680_bt_config = {
	.chip_type		= BT_CHIP_TI,
	.bt_wakeup_gpio		= 37,
	.host_wakeup_gpio	= 101,
	.reset_gpio		= 26,
	.bt_uart		= 2,
	.bt_sysclk		= BT_SYSCLK_38_4,
	.set_pm_limits	= rm680_bt_set_pm_limits,
};

static void rm696_sgx_dev_release(struct device *pdev)
{
	pr_debug("%s: (%p)", __func__, pdev);
}

static struct sgx_platform_data rm696_sgx_platform_data = {
	.fclock_max	= 200000000,
};

static struct platform_device rm696_sgx_device = {
	.name		= "pvrsrvkm",
	.id		= -1,
	.dev		= {
		.platform_data = &rm696_sgx_platform_data,
		.release = rm696_sgx_dev_release,
	}
};

static int __init rm696_video_init(void)
{
	int r;

	omap_setup_dss_device(&rm696_dss_device);

	rm696_dss_devices[0] = &rm696_dsi_display_data;

	r = gpio_request(rm696_panel_data.reset_gpio, "pyrenees reset");
	if (r < 0)
		goto err0;

	r = gpio_direction_output(rm696_panel_data.reset_gpio, 1);

	rm696_dss_data.default_device = rm696_dss_devices[0];

	/* TV */
	if (rm696_tv_display_data.reset_gpio != -1) {
		r = gpio_request(rm696_tv_display_data.reset_gpio,
				 "TV-out enable");
		if (r < 0)
			goto err1;

		r = gpio_direction_output(rm696_tv_display_data.reset_gpio, 0);
		if (r < 0)
			goto err2;
	}

	r = platform_device_register(&rm696_dss_device);
	if (r < 0)
		goto err2;

	omapfb_set_platform_data(&rm696_omapfb_data);

	r = platform_device_register(&rm696_sgx_device);
	if (r < 0)
		goto err3;

	return 0;

err3:
	platform_device_unregister(&rm696_dss_device);
err2:
	if (rm696_tv_display_data.reset_gpio != -1) {
		gpio_free(rm696_tv_display_data.reset_gpio);
		rm696_tv_display_data.reset_gpio = -1;
	}
err1:
	gpio_free(rm696_panel_data.reset_gpio);
	rm696_panel_data.reset_gpio = -1;
err0:
	pr_err("%s failed (%d)\n", __func__, r);

	return r;
}

subsys_initcall(rm696_video_init);

static int __init rm696_atmel_mxt_init(void)
{
	int err;

	err = gpio_request_one(ATMEL_MXT_RESET_GPIO, GPIOF_OUT_INIT_HIGH,
			       "mxt_reset");
	if (err)
		goto err1;

	err = gpio_request_one(ATMEL_MXT_IRQ_GPIO, GPIOF_DIR_IN, "mxt_irq");
	if (err)
		goto err2;

	rm696_peripherals_i2c_board_info_2[0].irq = gpio_to_irq(ATMEL_MXT_IRQ_GPIO);

	return 0;
err2:
	gpio_free(ATMEL_MXT_RESET_GPIO);
err1:

	return err;
}

#if defined(CONFIG_SND_OMAP_SOC_DFL61_TWL4030) || \
	defined(CONFIG_SND_OMAP_SOC_DFL61_TWL4030_MODULE)
static struct dfl61audio_twl4030_platform_data rm696_twl4030_data;

static struct platform_device rm696_twl4030_device = {
	.name          = "dfl61audio-twl4030",
	.id            = -1,
	.dev            = {
		.platform_data = &rm696_twl4030_data,
	},
};

static int __init rm696_audio_init(void)
{
	if (!machine_is_nokia_rm680() && !machine_is_nokia_rm696())
		return -ENODEV;

	rm696_twl4030_data.audio_config = AUDIO_CONFIG4;
	rm696_twl4030_data.freq = 38400000;

	platform_device_register(&rm696_twl4030_device);
	return 0;
}
#else
static inline void __init rm696_audio_init(void)
{
}
#endif

#if	defined(CONFIG_SND_SOC_TLV320DAC33) || \
	defined(CONFIG_SND_SOC_TLV320DAC33_MODULE)

static struct platform_device rm696_tlv320dac33_device = {
	.name          = "dfl61audio-tlv320dac33",
	.id            = -1,
};

static int __init rm696_tlv320dac33_init(void)
{
	int r;

	r = gpio_request(RM696_DAC33_IRQ_GPIO, "tlv320dac33 IRQ");
	if (r < 0) {
		printk(KERN_ERR "Failed to request IRQ gpio "
			"for tlv320dac33 chip\n");
	}

	rm696_peripherals_i2c_board_info_2[2].irq = gpio_to_irq(RM696_DAC33_IRQ_GPIO);

	gpio_direction_input(RM696_DAC33_IRQ_GPIO);
	
	platform_device_register(&rm696_tlv320dac33_device);

	return 0;
}
#else
static inline void __init rm696_tlv320dac33_init(void)
{
}
#endif

static void __init rm696_avplugdet_init(void)
{
	/* Prior to S0.2, the pin was as in previous platforms */
	if (system_rev < 0x0200)
		rm696_aci_data.avplugdet_plugged = AVPLUGDET_WHEN_PLUGGED_LOW;
}

static void __init rm680_peripherals_init(void)
{
	rm680_init_wl1271();
	rm696_init_vibra();

	platform_add_devices(rm680_peripherals_devices,
				ARRAY_SIZE(rm680_peripherals_devices));

	rm696_atmel_mxt_init();
	rm696_apds990x_init();
	rm696_avplugdet_init();
	rm680_i2c_init();
	gpmc_onenand_init(board_onenand_data);

	/* FIXME: gpio_hw_reset is present in 2.6 but missing in 3.5 */
	/* if (system_rev > 0x1300) {
                mmc[0].hw_reset_connected = 1;
                mmc[0].gpio_hw_reset = 39;
        }*/
	omap_hsmmc_init(mmc);
	rm696_ssi_init();
	omap_bt_init(&rm680_bt_config);
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static struct omap_musb_board_data rm696_musb_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode 			= MUSB_PERIPHERAL,
	.power			= 100,
};

static int __init rm696_set_l3_opp(void)
{
	struct device *l3_dev;
	struct clk *sdrc_ck;
	int r = 0;
	int opp1_clk  = 0, opp2_clk = 0;
	u32 sdrc_rate;

	l3_dev = omap_device_get_by_hwmod_name("l3_main");
	if (IS_ERR(l3_dev)) {
		pr_err("no l3_dev error\n");
		return -ENODEV;
	}

	/*
         * Adjust L3 OPPs according to the SDRC frequency initialized
         * by the boot loader
         */
	sdrc_ck = clk_get(NULL, "sdrc_ick");
        if (sdrc_ck) {
		sdrc_rate = clk_get_rate(sdrc_ck);
		switch (sdrc_rate) {
		case 185000000:
			opp1_clk = 92500000;
			opp2_clk = 185000000;
			break;
		case 195200000:
			opp1_clk = 97600000;
			opp2_clk = 195200000;
			break;
		case 200000000:
			pr_warning("Running device under out of "
				"spec clocking on L3\n");
			break;
		default:
			pr_warning("Booting with unknown L3 clock\n");
			break;
		}

		if (opp1_clk) {
			/* We need to add new rates*/
			r = opp_add(l3_dev, opp1_clk, 1000000);
			r |= opp_add(l3_dev, opp2_clk, 1200000);
			if (r)
			{
				/* We din't succed in adding new rates */
				pr_warning("failed to add l3 opp\n");
				goto err1;
			}

			/* We now have new rates, let's enable them  */
			r = opp_enable(l3_dev, opp1_clk);
			r |= opp_enable(l3_dev, opp2_clk);
			if (r)
			{
				/* We didin't succed in enabling rates */
				pr_warning("failed to enable l3 opp\n");
				goto err2;
			}

			/* New rates are added and enabled, let's disable default ones */
			opp_disable(l3_dev, 100000000);
			opp_disable(l3_dev, 200000000);

		}
		clk_put(sdrc_ck);
	}
        
        return 0;
err2:
	opp_disable(l3_dev, opp1_clk);
	opp_disable(l3_dev, opp2_clk);
err1:
	clk_put(sdrc_ck);
	
	return r;
}

static void __init rm680_init(void)
{
	struct omap_sdrc_params *sdrc_params;

	pr_info("RM-680/696 board, rev %04x\n", system_rev);
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	omap_serial_init();

	sdrc_params = nokia_get_sdram_timings();
	omap_sdrc_init(sdrc_params, sdrc_params);

	rm696_set_l3_opp();

	usb_musb_init(&rm696_musb_data);
	rm680_peripherals_init();

	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);

	rm696_audio_init();
	rm696_tlv320dac33_init();
	rm696_wl1273_init();
}

static void __init rx680_reserve(void)
{
	omap_vram_set_sdram_vram(PAGE_ALIGN(856 * 512 * 2 * 3) +
			PAGE_ALIGN(1280 * 720 * 2 * 6), 0);
	omap_reserve();
}

MACHINE_START(NOKIA_RM680, "Nokia RM-680 board")
	.atag_offset	= 0x100,
	.reserve	= rx680_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap3630_init_early,
	.init_irq	= omap3_init_irq,
	.handle_irq	= omap3_intc_handle_irq,
	.init_machine	= rm680_init,
	.init_late	= omap3630_init_late,
	.timer		= &omap3_timer,
	.restart	= omap_prcm_restart,
MACHINE_END

MACHINE_START(NOKIA_RM696, "Nokia RM-696 board")
	.atag_offset	= 0x100,
	.reserve	= rx680_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap3630_init_early,
	.init_irq	= omap3_init_irq,
	.handle_irq	= omap3_intc_handle_irq,
	.init_machine	= rm680_init,
	.init_late	= omap3630_init_late,
	.timer		= &omap3_timer,
	.restart	= omap_prcm_restart,
MACHINE_END
