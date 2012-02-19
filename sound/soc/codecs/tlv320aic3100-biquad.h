/*
 * linux/sound/soc/codecs/tlv320aic3100-biquad.h
 *
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 *
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 * Rev 0.1	Adatptive filter support                21-Apr-2010
 *
 * Rev 0.2	Updated with the biquad related defines
 *						        21-Jul-2010
 * Rev 0.3      Added the aic3100_parse_data struct
 *                                                      29-Jul-2010
 */

#ifndef _TLV320AIC3100_BIQUAD_H
#define _TLV320AIC3100_BIQUAD_H


/* defines */

#define FILT_CTL_NAME_DAC				"DAC adaptive filter(0=Disable, 1=Enable)"
#define COEFF_CTL_NAME_DAC				"DAC coeff Buffer(0=Buffer A, 1=Buffer B)"

#define BUFFER_PAGE_DAC					0x08
#define DAC_RAM_CNTRL					1
#define ADAPTIVE_CONTROL				0x4
#define ADAPTIVE_BUFFER_SWITCH				0x1

#define ADAPTIVE_MAX_CONTROLS			2

#define AIC3100_8BITS_MASK           	0xFF


/*******************************************************************************
 * AIC3100 Equalizer related defines and structs
 ******************************************************************************/
#define AIC3100_SPK_EQ_SELECT		0
#define AIC3100_HP_EQ_SELECT		1

#define MAX_MUX_CONTROLS		2
#define MIN_MUX_CTRL			0
#define MAX_MUX_CTRL			65535
#define MUX_CTRL_REG_SIZE		2

/* @struct aic3100_parse_data - . Parser info structure
 *
 * Structure used to parse through the EQ Array defined in the
 * tlv320aic3100_eq_coeff.h This structure will be initialized
 * during the Driver Intitialization time and will be populated
 * with the information related to Coefficient Pages. At run-time
 * depending on the Headset Insertion/Removal the value stored
 * in the burst_array will be programmed into the relevant Page
 * which is represented by the page_num member of this structure.
 */
typedef struct {
	char page_num;
	char burst_array[129];
	int burst_size;
	int current_loc;
} aic3100_parse_data;

#endif
