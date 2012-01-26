/*
 * linux/includes/linux/max17042.h
 *
 * Copyright (C) 2010 Barnes & Noble, Inc.
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

#ifndef __MAX17042_H__
#define __MAX17042_H__

#ifdef 	MAX17042_DEBUG
#define aprintk(fmt, args...)	printk(fmt, ##args)
#else
#define aprintk(fmt, args...)
#endif

#define MAX17042_DEVICE_ID			"max17042"
#define MAX17042_I2C_SLAVE_ADDRESS	0x36

#define VOLT_RESOLUTION	625
#define CURR_RESOLUTION	15625	//rate is 100 as it is supposed to be 156.25
#define TEMP_RESOLUTION	3900
#define TTE_RESOLUTION		5625

#define	 MAX17042_STATUS			0x00
#define	 MAX17042_ValertThreshold		0x01
#define	 MAX17042_TalertThreshold		0x02
#define	 MAX17042_SOCalertThreshold		0x03
#define	 MAX17042_AtRate			0x04
#define	 MAX17042_RemCapREP			0x05
#define	 MAX17042_SOCREP			0x06
#define	 MAX17042_Age 				0x07
#define	 MAX17042_Temperature			0x08
#define	 MAX17042_Vcell				0x09	
#define	 MAX17042_Current			0x0A	
#define	 MAX17042_AverageCurrent		0x0B
#define	 MAX17042_SOCmix			0x0D
#define	 MAX17042_SOCav				0x0E
#define	 MAX17042_RemCapmix			0x0F
#define	 MAX17042_FullCAP			0x10
#define	 MAX17042_TTE				0x11
#define	 MAX17042_Vempty			0x12
#define	 MAX17042_AverageTemperature		0x16
#define	 MAX17042_Cycles			0x17
#define	 MAX17042_DesignCap			0x18
#define	 MAX17042_AverageVcell			0x19
#define	 MAX17042_MaxMinTemperature		0x1A
#define	 MAX17042_MaxMinVoltage			0x1B
#define	 MAX17042_MaxMinCurrent			0x1C
#define	 MAX17042_CONFIG			0x1D
#define	 MAX17042_ICHGTerm			0x1E
#define	 MAX17042_RemCapav			0x1F
#define	 MAX17042_Version			0x21
#define	 MAX17042_FullCAPNom			0x23
#define	 MAX17042_TempNom			0x24
#define	 MAX17042_TempLim			0x25
#define	 MAX17042_AIN				0x27
#define	 MAX17042_FilterCFG			0x29
#define	 MAX17042_RelaxCFG			0x2A
#define	 MAX17042_MiscCFG			0x2B
#define	 MAX17042_TGAIN				0x2C
#define	 MAX17042_TOFF				0x2D
#define	 MAX17042_CGAIN				0x2E
#define	 MAX17042_COFF				0x2F
#define	 MAX17042_SOCempty			0x33
#define	 MAX17042_Iavg_empty			0x36
#define	 MAX17042_FCTC				0x37
#define	 MAX17042_RCOMP0			0x38
#define	 MAX17042_TempCo			0x39
#define	 MAX17042_Empty_TempCo			0x3A
#define	 MAX17042_k_empty0			0x3B
#define	 MAX17042_FSTAT				0x3D
#define	 MAX17042_SHDNTIMER			0x3F
#define	 MAX17042_dQacc				0x45
#define	 MAX17042_dPacc				0x46
#define	 MAX17042_Characterization_Table	0x50
#define	 MAX17042_OCV				0xEE


#define  MAX17042_CONFIG_Ts         (1<<13)
#define  MAX17042_CONFIG_Ten        (1<<9)
#define  MAX17042_CONFIG_Aen        (1<<2)
#define  MAX17042_CONFIG_Etherm     (1<<4)

#ifdef __KERNEL__
struct max17042_platform_data {
	int val_FullCAP;
	int val_Cycles;
	int val_FullCAPNom;	
	int val_SOCempty;
	int val_Iavg_empty;
	int val_RCOMP0;
	int val_TempCo;
	int val_k_empty0;
	int val_dQacc;
	int val_dPacc;
	int val_Empty_TempCo;
	int val_ICHGTerm;
	int val_Vempty;
	int val_FilterCFG;
	int val_TempNom;
	int val_DesignCap;
	int val_Capacity;
	int val_SOCREP;
	
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	int gpio;
};
#endif /* __KERNEL__ */

//example custom model from Maxim, pending on actual values 
extern const u16 custom_0x80[];
extern const u16 custom_0x90[]; 
extern const u16 custom_0xA0[];

#endif  /* __MAX17042_H__ */


