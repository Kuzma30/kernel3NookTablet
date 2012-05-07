/*
 * linux/includes/linux/max8903.h
 *
 * Copyright (C) 2011 Barnes & Noble, Inc.
 * Intrinsyc Software International, Inc. on behalf of Barnes & Nobles, Inc.
 *
 * Max8903 Battery Charger driver for Linux
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef __MAX8903_H__
#define __MAX8903_H__

#define MAX8903_UOK_GPIO_FOR_IRQ       82
#define MAX8903_DOK_GPIO_FOR_IRQ       81
#define MAX8903_GPIO_CHG_EN            60
#define MAX8903_GPIO_CHG_FLT           62
#define MAX8903_GPIO_CHG_IUSB          83

#define MAX8903_GPIO_CHG_USUS_EVT1A    96
#define MAX8903_GPIO_CHG_ILM_EVT1A     97

#define MAX8903_GPIO_CHG_USUS_EVT1B    63
#define MAX8903_GPIO_CHG_ILM_EVT1B     64

#define MAX8903_GPIO_CHG_USUS_DVT      63
#define MAX8903_GPIO_CHG_ILM_DVT       173

#define MAX8903_TOKEN_GPIO_CHG_EN     "max8903_gpio_chg_en"
#define MAX8903_TOKEN_GPIO_CHG_FLT    "max8903_gpio_chg_flt"
#define MAX8903_TOKEN_GPIO_CHG_IUSB   "max8903_gpio_chg_iusb"
#define MAX8903_TOKEN_GPIO_CHG_USUS   "max8903_gpio_chg_usus"
#define MAX8903_TOKEN_GPIO_CHG_ILM    "max8903_gpio_chg_ilm"
#define MAX8903_TOKEN_GPIO_CHG_UOK    "max8903_gpio_chg_uok"
#define MAX8903_TOKEN_GPIO_CHG_DOK    "max8903_gpio_chg_dok"


#endif  /* __MAX8903_H__ */


