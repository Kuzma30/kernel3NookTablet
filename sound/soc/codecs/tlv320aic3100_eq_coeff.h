/*
 * linux/sound/soc/codecs/tlv320aic3100_eq_coeff.h
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
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
 * Rev 0.1 	Created the file to hold the EQ coefficient values	26-Jul-2010
 *
 * Rev 0.2      Updated the header file with the EQ Values provided by TI
 *                                                                     29-Jul-2010
 * Rev 0.3      Updated the header file with the default ALLPASS EQ Values
 *              provided by TI for Headset Configuration	       02-Aug-2010
 */

#ifndef _TLV320AIC3100_EQ_COEFF_H
#define _TLV320AIC3100_EQ_COEFF_H


typedef char * string;

/* @struct aic3100_control
 *
 * Used to maintain the information about the Coefficients and its Locations.
 * This structure will be useful when we are working with the coefficients
 * in Non-Adaptive Mode. Also, the structure will be useful when there
 * are several Coefficient related configurations to be supported.
 */
typedef struct {
	u8 control_page;			// coefficient page location
	u8 control_base;			// coefficient base address within page
	u8 control_mute_flag;		// non-zero means muting required
	u8 control_string_index;	// string table indes
}aic3100_control;

/* @struct reg_value
 *
 * Structure derived from the PPS GDE which is used to represent
 * Register Offset and its Value as a pair.
 */

typedef struct {
	u8 reg_off;
	u8 reg_val;
} reg_value;


static aic3100_control	MUX_aic3100_controls[] = {
	{8, 2, 1, 0},
	{9, 2, 1, 1}
};


static string MUX_aic3100_control_names[] = {
	"Stereo_Mux_1",
	"Stereo_Mux_2"
};

/*
 * miniDSP_D_reg_Values
 *
 * B&N Speaker EQ biquad values.  These are only the bytes required to
 * setup the biquads.  Note that only the biaquads from 1:63 are
 * utilized and hence Configuration into Page 8 and 12 are sufficient
 * for B&N Please note page 8 will contain the Speaker EQ
 * Configuration and Page 12 will contain the default ALLPASS Filter
 * EQ Configuration for the Headset.
 */
reg_value miniDSP_D_reg_values[] = {
    {  0, 0x04},
    { 14, 0x7E},
    { 15, 0x13},
    { 16, 0x81},
    { 17, 0xED},
    { 18, 0x7E},
    { 19, 0x13},
    { 20, 0x7E},
    { 21, 0x0F},
    { 22, 0x83},
    { 23, 0xD1},
    { 24, 0x7E},
    { 25, 0x13},
    { 26, 0x81},
    { 27, 0xED},
    { 28, 0x7E},
    { 29, 0x13},
    { 30, 0x7E},
    { 31, 0x0F},
    { 32, 0x83},
    { 33, 0xD1},
    { 34, 0x15},
    { 35, 0x5B},
    { 36, 0x15},
    { 37, 0x5B},
    { 38, 0x15},
    { 39, 0x5B},
    { 40, 0x23},
    { 41, 0xD6},
    { 42, 0xE2},
    { 43, 0xE2},
    { 44, 0x15},
    { 45, 0x5B},
    { 46, 0x15},
    { 47, 0x5B},
    { 48, 0x15},
    { 49, 0x5B},
    { 50, 0x23},
    { 51, 0xD6},
    { 52, 0xE2},
    { 53, 0xE2},
    {  0,0x08},
    {  2,0x7B},
    {  3,0x97},
    {  4,0xC2},
    {  5,0x35},
    {  6,0x00},
    {  7,0x00},
    {  8,0x3B},
    {  9,0x98},
    { 10,0x00},
    { 11,0x00},
    { 12,0x79},
    { 13,0x16},
    { 14,0xC3},
    { 15,0x75},
    { 16,0x00},
    { 17,0x00},
    { 18,0x39},
    { 19,0x16},
    { 20,0x00},
    { 21,0x00},
    { 22,0x7B},
    { 23,0xC2},
    { 24,0x8B},
    { 25,0xFE},
    { 26,0x73},
    { 27,0x3D},
    { 28,0x74},
    { 29,0x02},
    { 30,0x91},
    { 31,0x00},
    { 32,0x7F},
    { 33,0x73},
    { 34,0x91},
    { 35,0x71},
    { 36,0x7A},
    { 37,0x6B},
    { 38,0x6F},
    { 39,0xAD},
    { 40,0x83},
    { 41,0x9A},
    { 42,0x6F},
    { 43,0xA3},
    { 44,0xF8},
    { 45,0xE3},
    { 46,0x4E},
    { 47,0xC3},
    { 48,0x07},
    { 49,0x1D},
    { 50,0xC1},
    { 51,0x99},
    { 52,0x7F},
    { 53,0xFF},
    { 54,0x00},
    { 55,0x00},
    { 56,0x00},
    { 57,0x00},
    { 58,0x00},
    { 59,0x00},
    { 60,0x00},
    { 61,0x00},
    { 66,0x7B},
    { 67,0x97},
    { 68,0xC2},
    { 69,0x35},
    { 70,0x00},
    { 71,0x00},
    { 72,0x3B},
    { 73,0x98},
    { 74,0x00},
    { 75,0x00},
    { 76,0x79},
    { 77,0x16},
    { 78,0xC3},
    { 79,0x75},
    { 80,0x00},
    { 81,0x00},
    { 82,0x39},
    { 83,0x16},
    { 84,0x00},
    { 85,0x00},
    { 86,0x7B},
    { 87,0xC2},
    { 88,0x8B},
    { 89,0xFE},
    { 90,0x73},
    { 91,0x3D},
    { 92,0x74},
    { 93,0x02},
    { 94,0x91},
    { 95,0x00},
    { 96,0x7F},
    { 97,0x73},
    { 98,0x91},
    { 99,0x71},
    {100,0x7A},
    {101,0x6B},
    {102,0x6F},
    {103,0xAD},
    {104,0x83},
    {105,0x9A},
    {106,0x6F},
    {107,0xA3},
    {108,0xF8},
    {109,0xE3},
    {110,0x4E},
    {111,0xC3},
    {112,0x07},
    {113,0x1D},
    {114,0xC1},
    {115,0x99},
    {116,0x7F},
    {117,0xFF},
    {118,0x00},
    {119,0x00},
    {120,0x00},
    {121,0x00},
    {122,0x00},
    {123,0x00},
    {124,0x00},
    {125,0x00},
    {  0,0x09},
    { 0x0e,0x7f},
    { 0x0f,0xab},
    { 0x10, 0x80},
    { 0x11, 0x55},
    { 0x12, 0x7f},
    { 0x13, 0x56},
    { 0x14, 0},
    { 0x15, 0x02},
    { 0x16, 0x00},
    { 0x17, 0x02},
    { 0x18, 0x7f},
    { 0x19, 0xfb},
};

#endif /* #ifdef _TLV320AIC3100_EQ_COEFF_H */
