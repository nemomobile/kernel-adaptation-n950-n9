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
#include <linux/platform_device.h>
#include <linux/omapfb.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/wl12xx.h>
#include <linux/spi/spi.h>
#include <linux/cpu.h>
#include <linux/opp.h>
#include <linux/hsi/hsi.h>
#include <linux/cmt.h>
#include <linux/i2c/ak8975.h>
#include <plat/gpio.h>
#include <linux/lis3lv02d.h>

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

#define ATMEL_MXT_IRQ_GPIO		61
#define ATMEL_MXT_RESET_GPIO		81

#define LIS302_IRQ1_GPIO 180
#define LIS302_IRQ2_GPIO 181  /* Not yet in use */



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
	struct device *mpu_dev;

	r = omap3_opp_init();
	if (IS_ERR_VALUE(r) && (r != -EEXIST)) {
		pr_err("opp default init failed\n");
		return r;
	}

	mpu_dev = omap_device_get_by_hwmod_name("mpu");
	if (IS_ERR(mpu_dev)) {
		pr_err("no mpu_dev error\n");
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

static struct regulator_consumer_supply rm696_vbat_consumers[] = {
	REGULATOR_SUPPLY("Vled", "2-0039"),	/* APDS990x */	
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

static struct platform_device *rm680_peripherals_devices[] __initdata = {
	&rm696_vbat,
	&rm680_vemmc_device,
	&rm696_cmt_device,
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

/* TWL */
static struct twl4030_gpio_platform_data rm680_gpio_data = {
	.gpio_base		= OMAP_MAX_GPIO_LINES,
	.irq_base		= TWL4030_GPIO_IRQ_BASE,
	.irq_end		= TWL4030_GPIO_IRQ_END,
	.pullups		= BIT(0),
	.pulldowns		= BIT(1) | BIT(2) | BIT(8) | BIT(15),
};

static struct regulator_consumer_supply rm696_vio_consumers[] = {
	REGULATOR_SUPPLY("DVdd", "3-000f"),	/* AK8975 */
	REGULATOR_SUPPLY("Vdd_IO", "3-001d"),	/* LIS302 */
	REGULATOR_SUPPLY("VDDI", "display0"),	/* Himalaya */
	REGULATOR_SUPPLY("Vdd", "2-004b"),	/* Atmel mxt */	
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
	REGULATOR_SUPPLY("AVdd", "3-000f"),	/* AK8975 */
	REGULATOR_SUPPLY("Vdd", "3-001d"),	/* LIS302 */
	REGULATOR_SUPPLY("Vdd", "2-0039"),	/* APDS990x */
	REGULATOR_SUPPLY("AVdd", "2-004b"),	/* Atmel mxt */
	REGULATOR_SUPPLY("v28", "twl5031_aci"),	
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

static struct twl4030_clock_init_data rm696_clock_init_data = {
	.ck32k_lowpwr_enable = 1,
};

static struct twl4030_platform_data rm680_twl_data = {
	.irq_base		= TWL4030_IRQ_BASE,
	.irq_end		= TWL4030_IRQ_END,
	.clock			= &rm696_clock_init_data,

	.gpio			= &rm680_gpio_data,
	
	/* LDOs */
	.vio			= &rm696_vio_data,
	.vmmc2			= &rm696_vmmc2_data,
	.vaux1			= &rm696_vaux1_data,
};

static struct mxt_platform_data atmel_mxt_platform_data = {
	.reset_gpio = ATMEL_MXT_RESET_GPIO,
	.int_gpio = ATMEL_MXT_IRQ_GPIO,
	.rlimit_min_interval_us = 7000,
	.rlimit_bypass_time_us = 25000,
	.wakeup_interval_ms = 50,
	.config = &atmel_mxt_pyrenees_config,
};

static struct i2c_board_info rm696_peripherals_i2c_board_info_2[] /*__initdata */= {
	{
		/* keep this first */
		I2C_BOARD_INFO("atmel_mxt", 0x4b),
		.platform_data	= &atmel_mxt_platform_data,
	},
};

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

#define OMAP_GPIO_IRQ(nr)	(OMAP_GPIO_IS_MPUIO(nr) ? \
				 IH_MPUIO_BASE + ((nr) & 0x0f) : \
				 IH_GPIO_BASE + (nr))

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
	.irq2 = OMAP_GPIO_IRQ(LIS302_IRQ2_GPIO),
};

#endif

#if defined(CONFIG_SENSORS_AK8975) || defined(CONFIG_SENSORS_AK8975_MODULE)
static struct ak8975_platform_data rm696_ak8975_data = {
	.axis_x = AK8975_DEV_Z,
	.axis_y = AK8975_INV_DEV_X,
	.axis_z = AK8975_INV_DEV_Y,
};
#endif

static struct i2c_board_info rm696_peripherals_i2c_board_info_3[] /*__initdata */= {
#if defined(CONFIG_SENSORS_LIS3_I2C) || defined(CONFIG_SENSORS_LIS3_I2C_MODULE)
	{
		I2C_BOARD_INFO("lis3lv02d", 0x1d),
		.platform_data = &rm696_lis302dl_data,
		.irq = OMAP_GPIO_IRQ(LIS302_IRQ1_GPIO),
	},
#endif
#if defined(CONFIG_SENSORS_AK8975) || defined(CONFIG_SENSORS_AK8975_MODULE)
	{
		I2C_BOARD_INFO("ak8975", 0x0f),
		.platform_data = &rm696_ak8975_data,
	},
#endif
};

static void __init rm680_i2c_init(void)
{
	omap3_pmic_get_config(&rm680_twl_data, TWL_COMMON_PDATA_USB,
			      TWL_COMMON_REGULATOR_VDAC |
			      TWL_COMMON_REGULATOR_VPLL2);
	omap_pmic_init(1, 2900, "twl5031", INT_34XX_SYS_NIRQ, &rm680_twl_data);
	omap_register_i2c_bus(2, 400, rm696_peripherals_i2c_board_info_2,
			      ARRAY_SIZE(rm696_peripherals_i2c_board_info_2));
	omap_register_i2c_bus(3, 400, rm696_peripherals_i2c_board_info_3, ARRAY_SIZE(rm696_peripherals_i2c_board_info_3));
}

#if defined(CONFIG_MTD_ONENAND_OMAP2) || \
	defined(CONFIG_MTD_ONENAND_OMAP2_MODULE)
static struct omap_onenand_platform_data board_onenand_data[] = {
	{
		.gpio_irq	= 65,
		.flags		= ONENAND_SYNC_READWRITE,
	}
};
#endif

static struct omap2_hsmmc_info mmc[] __initdata = {
/* eMMC */
	{
		.name		= "internal",
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
/* WLAN */
	{
		.name		= "wl1271",
		.mmc		= 3,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_NONREMOVABLE,
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

static void __init rm680_peripherals_init(void)
{
	rm680_init_wl1271();

	platform_add_devices(rm680_peripherals_devices,
				ARRAY_SIZE(rm680_peripherals_devices));
	rm696_atmel_mxt_init();
	rm680_i2c_init();
	gpmc_onenand_init(board_onenand_data);
	omap_hsmmc_init(mmc);
	rm696_ssi_init();
	omap_bt_init(&rm680_bt_config);
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static void __init rm680_init(void)
{
	struct omap_sdrc_params *sdrc_params;

	pr_info("RM-680/696 board, rev %04x\n", system_rev);
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	omap_serial_init();

	sdrc_params = nokia_get_sdram_timings();
	omap_sdrc_init(sdrc_params, sdrc_params);

	usb_musb_init(NULL);
	rm680_peripherals_init();
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
