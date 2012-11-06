/*
 *  arch/arm/plat-omap/include/mach/board-nokia.h
 *
 *  Information structures for Nokia-specific board config data
 *
 *  Copyright (C) 2005	Nokia Corporation
 */

#ifndef __ASM_ARCH_OMAP_NOKIA_H
#define __ASM_ARCH_OMAP_NOKIA_H

#include <linux/types.h>

struct omap_bluetooth_config;
extern void omap_bt_init(struct omap_bluetooth_config *bt_config);

#define BT_CHIP_CSR		1
#define BT_CHIP_TI		2
#define BT_CHIP_BCM		3

#define BT_SYSCLK_12		1
#define BT_SYSCLK_38_4		2

/* Allow C6 state {1, 3120, 5788, 10000} */
#define H4P_WAKEUP_LATENCY	5700

struct omap_bluetooth_config {
	u8    chip_type;
	u8    bt_wakeup_gpio;
	u8    host_wakeup_gpio;
	u8    reset_gpio;
	u8    reset_gpio_shared;
	u8    bt_uart;
	u8    bd_addr[6];
	u8    bt_sysclk;
	void  (*set_pm_limits)(struct device *dev, bool set);
};

#endif
