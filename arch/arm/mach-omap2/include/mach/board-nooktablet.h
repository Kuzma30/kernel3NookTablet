/*
 * linux/arch/arm/mach-omap2/board-4430-acclaim.h
 *
 * Copyright (C) 2011 Barnes & Noble.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _BOARD_NOOKTABLET_H
#define _BOARD_NOOKTABLET_H

#include <linux/regulator/consumer.h>

#ifdef CONFIG_MACH_OMAP4_NOOKTABLET
#define EVT0  0
#define EVT1A 1
#define EVT1B 2
#define EVT2  3
#define DVT   4
#define PVT   5

#include <asm/system.h>

static inline int acclaim_board_type(void)
{
	return system_rev;  // This is set by U-Boot
}

extern ulong get_sdram_size(void);

#define BN_USB_VENDOR_ID            0x2080
#define BN_USB_PRODUCT_ID_ACCLAIM   0x0004
#define BN_USB_MANUFACTURER_NAME    "B&N"
#define BN_USB_PRODUCT_NAME	    "NOOK Tablet"

#define NOOKTABLET_RAM_CONSOLE_SIZE         (1 << CONFIG_LOG_BUF_SHIFT)
// This address is the beginning of the 2nd bank
// if SDRAM size > 512MB
#define NOOKTABLET_RAM_CONSOLE_START        0xA0000000
#define NOOKTABLET_RAM_CONSOLE_END          (NOOKTABLET_RAM_CONSOLE_START + NOOKTABLET_RAM_CONSOLE_SIZE - 1)
// if SDRAM size = 512MB
#define NOOKTABLET_RAM_CONSOLE_512MB_START  (NOOKTABLET_RAM_CONSOLE_512MB_END - NOOKTABLET_RAM_CONSOLE_SIZE + 1)
#define NOOKTABLET_RAM_CONSOLE_512MB_END    (0x9C000000 - 1)

struct boxer_panel_data {
	struct regulator *vlcd;
};

void omap4_create_board_props(void);
#endif
#endif
