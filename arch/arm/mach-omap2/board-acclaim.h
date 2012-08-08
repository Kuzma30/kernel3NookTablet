/*
 * arch/arm/mach-omap2/board-acclaim.h
 *
 * Copyright (C) 2011 Texas Instruments
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _MACH_OMAP_BOARD_ACCLAIM_H
#define _MACH_OMAP_BOARD_ACCLAIM_H

int acclaim_touch_init(void);
void omap4_create_board_props(void);
int acclaim_panel_init(void);
int acclaim_button_init(void);

#endif
