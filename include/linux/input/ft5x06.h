/* Header file for:
 * Focaltech 5x06 touch screen controller. Some parts of the code are based on the Cypress 
 * TrueTouch(TM) Standard Product I2C touchscreen driver.
 *
 * drivers/input/touchscreen/ft5x06-i2c.c
 *
 * Copyright (C) 2009, 2010 Cypress Semiconductor, Inc.
 *
 * Copyright (c) 2010 Barnes & Noble.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 */


#ifndef __FT5x06_H__
#define __FT5x06_H__

#include <linux/input.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#define FT_NAME      "ft5x06"
#define FT_I2C_NAME  "ft5x06-i2c"

/********************************************************************
 * Register Offsets (Working Mode)
 */
#define FT5x06_WMREG_DEVICE_MODE        0x00
#define FT5x06_WMREG_GEST_ID            0x01
#define FT5x06_WMREG_TD_STATUS          0x02
#define FT5x06_WMREG_P1_XH              0x03
#define FT5x06_WMREG_P1_XL              0x04
#define FT5x06_WMREG_P1_YH              0x05
#define FT5x06_WMREG_P1_YL              0x06
#define FT5x06_WMREG_P2_XH              0x09
#define FT5x06_WMREG_P2_XL              0x0A
#define FT5x06_WMREG_P2_YH              0x0B
#define FT5x06_WMREG_P2_YL              0x0C
#define FT5x06_WMREG_TH_TOUCH           0x80
#define FT5x06_WMREG_RPT_RATE           0x88
#define FT5x06_WMREG_OFFSET_LEFT_RIGHT  0x92
#define FT5x06_WMREG_DISTANCE_ZOOM      0x97
#define FT5x06_WMREG_LIB_VER_H          0xA1
#define FT5x06_WMREG_LIB_VER_L          0xA2
#define FT5x06_WMREG_PWR_MODE           0xA5
#define FT5x06_WMREG_FW_VER             0xA6
#define FT5x06_WMREG_FOCALTECH_ID       0xA8
#define FT5x06_WMREG_RESET              0xFC

/********************************************************************
 * Register Offsets (Factory Mode)
 */
#define FT5x06_FMREG_DEVICE_MODE        0x00
#define FT5x06_FMREG_ROW_ADDR           0x01
#define FT5x06_FMREG_CALIBRATE          0x02
#define FT5x06_FMREG_TX_NUM             0x03
#define FT5x06_FMREG_RX_NUM             0x04
#define FT5x06_FMREG_DRIVER_VOLTAGE     0x05
#define FT5x06_FMREG_START_RX           0x06
#define FT5x06_FMREG_GAIN               0x07
#define FT5x06_FMREG_ORIGIN_XH          0x08
#define FT5x06_FMREG_ORIGIN_XL          0x09
#define FT5x06_FMREG_ORIGIN_YH          0x0A
#define FT5x06_FMREG_ORIGIN_YL          0x0B
#define FT5x06_FMREG_RES_WH             0x0C
#define FT5x06_FMREG_RES_WL             0x0D
#define FT5x06_FMREG_RES_HH             0x0E
#define FT5x06_FMREG_RES_HL             0x0F
#define FT5x06_FMREG_RAWDATA_0_H        0x10
#define FT5x06_FMREG_RAWDATA_0_L        0x11
#define FT5x06_FMREG_RAWDATA_1_H        0x12
#define FT5x06_FMREG_RAWDATA_1_L        0x13
#define FT5x06_FMREG_RAWDATA_2_H        0x14
#define FT5x06_FMREG_RAWDATA_2_L        0x15
#define FT5x06_FMREG_RAWDATA_3_H        0x16
#define FT5x06_FMREG_RAWDATA_3_L        0x17
#define FT5x06_FMREG_RAWDATA_4_H        0x18
#define FT5x06_FMREG_RAWDATA_4_L        0x19
#define FT5x06_FMREG_RAWDATA_5_H        0x1A
#define FT5x06_FMREG_RAWDATA_5_L        0x1B
#define FT5x06_FMREG_RAWDATA_6_H        0x1C
#define FT5x06_FMREG_RAWDATA_6_L        0x1D
#define FT5x06_FMREG_RAWDATA_7_H        0x1E
#define FT5x06_FMREG_RAWDATA_7_L        0x1F
#define FT5x06_FMREG_RAWDATA_8_H        0x20
#define FT5x06_FMREG_RAWDATA_8_L        0x21
#define FT5x06_FMREG_RAWDATA_9_H        0x22
#define FT5x06_FMREG_RAWDATA_9_L        0x23
#define FT5x06_FMREG_RAWDATA_10_H       0x24
#define FT5x06_FMREG_RAWDATA_10_L       0x25
#define FT5x06_FMREG_RAWDATA_11_H       0x26
#define FT5x06_FMREG_RAWDATA_11_L       0x27
#define FT5x06_FMREG_RAWDATA_12_H       0x28
#define FT5x06_FMREG_RAWDATA_12_L       0x29
#define FT5x06_FMREG_RAWDATA_13_H       0x2A
#define FT5x06_FMREG_RAWDATA_13_L       0x2B
#define FT5x06_FMREG_RAWDATA_14_H       0x2C
#define FT5x06_FMREG_RAWDATA_14_L       0x2D
#define FT5x06_FMREG_RAWDATA_15_H       0x2E
#define FT5x06_FMREG_RAWDATA_15_L       0x2F
#define FT5x06_FMREG_RAWDATA_16_H       0x30
#define FT5x06_FMREG_RAWDATA_16_L       0x31
#define FT5x06_FMREG_RAWDATA_17_H       0x32
#define FT5x06_FMREG_RAWDATA_17_L       0x33
#define FT5x06_FMREG_RAWDATA_18_H       0x34
#define FT5x06_FMREG_RAWDATA_18_L       0x35
#define FT5x06_FMREG_RAWDATA_19_H       0x36
#define FT5x06_FMREG_RAWDATA_19_L       0x37
#define FT5x06_FMREG_RAWDATA_20_H       0x38
#define FT5x06_FMREG_RAWDATA_20_L       0x39
#define FT5x06_FMREG_RAWDATA_21_H       0x3A
#define FT5x06_FMREG_RAWDATA_21_L       0x3B
#define FT5x06_FMREG_RAWDATA_22_H       0x3C
#define FT5x06_FMREG_RAWDATA_22_L       0x3D
#define FT5x06_FMREG_RAWDATA_23_H       0x3E
#define FT5x06_FMREG_RAWDATA_23_L       0x3F
#define FT5x06_FMREG_RAWDATA_24_H       0x40
#define FT5x06_FMREG_RAWDATA_24_L       0x41
#define FT5x06_FMREG_RAWDATA_25_H       0x42
#define FT5x06_FMREG_RAWDATA_25_L       0x43
#define FT5x06_FMREG_RAWDATA_26_H       0x44
#define FT5x06_FMREG_RAWDATA_26_L       0x45
#define FT5x06_FMREG_RAWDATA_27_H       0x46
#define FT5x06_FMREG_RAWDATA_27_L       0x47
#define FT5x06_FMREG_RAWDATA_28_H       0x48
#define FT5x06_FMREG_RAWDATA_28_L       0x49
#define FT5x06_FMREG_RAWDATA_29_H       0x4A
#define FT5x06_FMREG_RAWDATA_29_L       0x4B
#define FT5x06_FMREG_TH_POINT_NUM       0x4C
#define FT5x06_FMREG_BASELINE_ENABLE    0x4D
#define FT5x06_FMREG_IC_PARTNO          0x4E
#define FT5x06_FMREG_INTERRUPT_TOGGLE   0x4F
#define FT5x06_FMREG_TX_ORDER_0         0x50
#define FT5x06_FMREG_TX_ORDER_1         0x51
#define FT5x06_FMREG_TX_ORDER_2         0x52
#define FT5x06_FMREG_TX_ORDER_3         0x53
#define FT5x06_FMREG_TX_ORDER_4         0x54
#define FT5x06_FMREG_TX_ORDER_5         0x55
#define FT5x06_FMREG_TX_ORDER_6         0x56
#define FT5x06_FMREG_TX_ORDER_7         0x57
#define FT5x06_FMREG_TX_ORDER_8         0x58
#define FT5x06_FMREG_TX_ORDER_9         0x59
#define FT5x06_FMREG_TX_ORDER_10        0x5A
#define FT5x06_FMREG_TX_ORDER_11        0x5B
#define FT5x06_FMREG_TX_ORDER_12        0x5C
#define FT5x06_FMREG_TX_ORDER_13        0x5D
#define FT5x06_FMREG_TX_ORDER_14        0x5E
#define FT5x06_FMREG_TX_ORDER_15        0x5F
#define FT5x06_FMREG_TX_ORDER_16        0x60
#define FT5x06_FMREG_TX_ORDER_17        0x61
#define FT5x06_FMREG_TX_ORDER_18        0x62
#define FT5x06_FMREG_TX_ORDER_19        0x63
#define FT5x06_FMREG_TX_ORDER_20        0x64
#define FT5x06_FMREG_TX_ORDER_21        0x65
#define FT5x06_FMREG_TX_ORDER_22        0x66
#define FT5x06_FMREG_TX_ORDER_23        0x67
#define FT5x06_FMREG_TX_ORDER_24        0x68
#define FT5x06_FMREG_TX_ORDER_25        0x69
#define FT5x06_FMREG_TX_ORDER_26        0x6A
#define FT5x06_FMREG_TX_ORDER_27        0x6B
#define FT5x06_FMREG_TX_ORDER_28        0x6C
#define FT5x06_FMREG_TX_ORDER_29        0x6D
#define FT5x06_FMREG_TX_ORDER_30        0x6E
#define FT5x06_FMREG_TX_ORDER_31        0x6F
#define FT5x06_FMREG_TX_ORDER_32        0x70
#define FT5x06_FMREG_TX_ORDER_33        0x71
#define FT5x06_FMREG_TX_ORDER_34        0x72
#define FT5x06_FMREG_TX_ORDER_35        0x73
#define FT5x06_FMREG_TX_ORDER_36        0x74
#define FT5x06_FMREG_TX_ORDER_37        0x75
#define FT5x06_FMREG_TX_ORDER_38        0x76
#define FT5x06_FMREG_TX_ORDER_39        0x77
#define FT5x06_FMREG_TX_0_CAC           0x78
#define FT5x06_FMREG_TX_1_CAC           0x79
#define FT5x06_FMREG_TX_2_CAC           0x7A
#define FT5x06_FMREG_TX_3_CAC           0x7B
#define FT5x06_FMREG_TX_4_CAC           0x7C
#define FT5x06_FMREG_TX_5_CAC           0x7D
#define FT5x06_FMREG_TX_6_CAC           0x7E
#define FT5x06_FMREG_TX_7_CAC           0x7F
#define FT5x06_FMREG_TX_8_CAC           0x80
#define FT5x06_FMREG_TX_9_CAC           0x81
#define FT5x06_FMREG_TX_10_CAC          0x82
#define FT5x06_FMREG_TX_11_CAC          0x83
#define FT5x06_FMREG_TX_12_CAC          0x84
#define FT5x06_FMREG_TX_13_CAC          0x85
#define FT5x06_FMREG_TX_14_CAC          0x86
#define FT5x06_FMREG_TX_15_CAC          0x87
#define FT5x06_FMREG_TX_16_CAC          0x88
#define FT5x06_FMREG_TX_17_CAC          0x89
#define FT5x06_FMREG_TX_18_CAC          0x8A
#define FT5x06_FMREG_TX_19_CAC          0x8B
#define FT5x06_FMREG_TX_20_CAC          0x8C
#define FT5x06_FMREG_TX_21_CAC          0x8D
#define FT5x06_FMREG_TX_22_CAC          0x8E
#define FT5x06_FMREG_TX_23_CAC          0x8F
#define FT5x06_FMREG_TX_24_CAC          0x90
#define FT5x06_FMREG_TX_25_CAC          0x91
#define FT5x06_FMREG_TX_26_CAC          0x92
#define FT5x06_FMREG_TX_27_CAC          0x93
#define FT5x06_FMREG_TX_28_CAC          0x94
#define FT5x06_FMREG_TX_29_CAC          0x95
#define FT5x06_FMREG_TX_30_CAC          0x96
#define FT5x06_FMREG_TX_31_CAC          0x97
#define FT5x06_FMREG_TX_32_CAC          0x98
#define FT5x06_FMREG_TX_33_CAC          0x99
#define FT5x06_FMREG_TX_34_CAC          0x9A
#define FT5x06_FMREG_TX_35_CAC          0x9B
#define FT5x06_FMREG_TX_36_CAC          0x9C
#define FT5x06_FMREG_TX_37_CAC          0x9D
#define FT5x06_FMREG_TX_38_CAC          0x9E
#define FT5x06_FMREG_TX_39_CAC          0x9F
#define FT5x06_FMREG_RX_0_CAC           0xA0
#define FT5x06_FMREG_RX_1_CAC           0xA1
#define FT5x06_FMREG_RX_2_CAC           0xA2
#define FT5x06_FMREG_RX_3_CAC           0xA3
#define FT5x06_FMREG_RX_4_CAC           0xA4
#define FT5x06_FMREG_RX_5_CAC           0xA5
#define FT5x06_FMREG_RX_6_CAC           0xA6
#define FT5x06_FMREG_RX_7_CAC           0xA7
#define FT5x06_FMREG_RX_8_CAC           0xA8
#define FT5x06_FMREG_RX_9_CAC           0xA9
#define FT5x06_FMREG_RX_10_CAC          0xAA
#define FT5x06_FMREG_RX_11_CAC          0xAB
#define FT5x06_FMREG_RX_12_CAC          0xAC
#define FT5x06_FMREG_RX_13_CAC          0xAD
#define FT5x06_FMREG_RX_14_CAC          0xAE
#define FT5x06_FMREG_RX_15_CAC          0xAF
#define FT5x06_FMREG_RX_16_CAC          0xB0
#define FT5x06_FMREG_RX_17_CAC          0xB1
#define FT5x06_FMREG_RX_18_CAC          0xB2
#define FT5x06_FMREG_RX_19_CAC          0xB3
#define FT5x06_FMREG_RX_20_CAC          0xB4
#define FT5x06_FMREG_RX_21_CAC          0xB5
#define FT5x06_FMREG_RX_22_CAC          0xB6
#define FT5x06_FMREG_RX_23_CAC          0xB7
#define FT5x06_FMREG_RX_24_CAC          0xB8
#define FT5x06_FMREG_RX_25_CAC          0xB9
#define FT5x06_FMREG_RX_26_CAC          0xBA
#define FT5x06_FMREG_RX_27_CAC          0xBB
#define FT5x06_FMREG_RX_28_CAC          0xBC
#define FT5x06_FMREG_RX_29_CAC          0xBD
#define FT5x06_FMREG_RESERVED           0xBE
#define FT5x06_FMREG_TX_0_1_OFFSET      0xBF
#define FT5x06_FMREG_TX_2_3_OFFSET      0xC0
#define FT5x06_FMREG_TX_4_5_OFFSET      0xC1
#define FT5x06_FMREG_TX_6_7_OFFSET      0xC2
#define FT5x06_FMREG_TX_8_9_OFFSET      0xC3
#define FT5x06_FMREG_TX_10_11_OFFSET    0xC4
#define FT5x06_FMREG_TX_12_13_OFFSET    0xC5
#define FT5x06_FMREG_TX_14_15_OFFSET    0xC6
#define FT5x06_FMREG_TX_16_17_OFFSET    0xC7
#define FT5x06_FMREG_TX_18_19_OFFSET    0xC8
#define FT5x06_FMREG_TX_20_21_OFFSET    0xC9
#define FT5x06_FMREG_TX_22_23_OFFSET    0xCA
#define FT5x06_FMREG_TX_24_25_OFFSET    0xCB
#define FT5x06_FMREG_TX_26_27_OFFSET    0xCC
#define FT5x06_FMREG_TX_28_29_OFFSET    0xCD
#define FT5x06_FMREG_TX_30_31_OFFSET    0xCE
#define FT5x06_FMREG_TX_32_33_OFFSET    0xCF
#define FT5x06_FMREG_TX_34_35_OFFSET    0xD0
#define FT5x06_FMREG_TX_36_37_OFFSET    0xD1
#define FT5x06_FMREG_TX_38_39_OFFSET    0xD2
#define FT5x06_FMREG_RX_0_1_OFFSET      0xD3
#define FT5x06_FMREG_RX_2_3_OFFSET      0xD4
#define FT5x06_FMREG_RX_4_5_OFFSET      0xD5
#define FT5x06_FMREG_RX_6_7_OFFSET      0xD6
#define FT5x06_FMREG_RX_8_9_OFFSET      0xD7
#define FT5x06_FMREG_RX_10_11_OFFSET    0xD8
#define FT5x06_FMREG_RX_12_13_OFFSET    0xD9
#define FT5x06_FMREG_RX_14_15_OFFSET    0xDA
#define FT5x06_FMREG_RX_16_17_OFFSET    0xDB
#define FT5x06_FMREG_RX_18_19_OFFSET    0xDC
#define FT5x06_FMREG_RX_20_21_OFFSET    0xDD
#define FT5x06_FMREG_RX_22_23_OFFSET    0xDE
#define FT5x06_FMREG_RX_24_25_OFFSET    0xDF
#define FT5x06_FMREG_RX_26_27_OFFSET    0xE0
#define FT5x06_FMREG_RX_28_29_OFFSET    0xE1
#define FT5x06_FMREG_RESERVEDBLK_START  0xE2
#define FT5x06_FMREG_RESERVEDBLK_END    0xFD
#define FT5x06_FMREG_LOG_MSG_CNT        0xFE
#define FT5x06_FMREG_LOG_CUR_CHA        0xFF
#define FT5x06_FMREG_MAX                0xFF

/********************************************************************
 * Register Bits & Masks: DEVICE_MODE
 */
#define FT5x06_MODE_WORKING             0x00
#define FT5x06_MODE_FACTORY             0x40
#define FT5x06_MODE_MASK                0x70

#define FT5x06_SCAN_START               0x80
#define FT5x06_SCAN_DONE                0x00
#define FT5x06_SCAN_MASK                0x80

/********************************************************************
 * Register Bits & Masks: CALIBRATION
 */
#define FT5x06_CALIBRATE_START          0x04
#define FT5x06_CALIBRATE_SAVE_TO_FLASH  0x05

/********************************************************************
 * Register Bits & Masks: DRIVER_VOLTAGE
 */
#define FT5x06_VOLTAGE_MASK             0x07

/********************************************************************
 * Register Bits & Masks: BASELINE_ENABLE
 */
#define FT5x06_BASELINE_ENABLE          0x01
#define FT5x06_BASELINE_DISABLE         0x00
#define FT5x06_BASELINE_MASK            0x01

/********************************************************************
 * Register Bits & Masks: IC_PARTNO
 */
#define FT5x06_PART_FT5206              0x26
#define FT5x06_PART_FT5306              0x36
#define FT5x06_PART_FT5406              0x46
#define FT5x06_PART_FT5202              0x22
#define FT5x06_PART_FT5302              0x32


/********************************************************************
 * CTPM Commands
 */
#define FT5x06_CMD_RESET_CTPM_H         0xAA
#define FT5x06_CMD_RESET_CTPM_L         0x55
#define FT5x06_CMD_GET_CHECKSUM         0xCC
#define FT5x06_CMD_RESET_FW             0x07
#define FT5x06_CMD_ERASE_FW             0x61
#define FT5x06_CMD_GET_ID               0x90
#define FT5x06_CMD_GET_ID_P1            0x00
#define FT5x06_CMD_GET_ID_P2            0x00
#define FT5x06_CMD_GET_ID_P3            0x00


/******************************************************************************
 * Global Control, Used to control the behavior of the driver
 */

/* define if MT signals are desired */
#define FT_USE_MT_SIGNALS

/* End of the Global Control section
 ******************************************************************************
 */

/* reduce extra signals in MT only build
 * be careful not to lose backward compatibility for pre-MT apps
 */
#ifdef FT_USE_ST_SIGNALS
    #define FT_USE_ST   1
#else
    #define FT_USE_ST   0
#endif /* FT_USE_ST_SIGNALS */

/* rely on kernel input.h to define Multi-Touch capability */
/* if input.h defines the Multi-Touch signals, then use MT */
#if defined(ABS_MT_TOUCH_MAJOR) && defined(FT_USE_MT_SIGNALS)
    #define FT_USE_MT           1
    #define FT_MT_SYNC(input)   input_mt_sync(input)
#else
    #define FT_USE_MT           0
    #define FT_MT_SYNC(input)
    /* the following includes are provided to ensure a compile;
     * the code that compiles with these defines will not be executed if
     * the FT_USE_MT is properly used in the platform structure init
     */
    #ifndef ABS_MT_TOUCH_MAJOR
    #define ABS_MT_TOUCH_MAJOR  0x30    /* touching ellipse */
    #define ABS_MT_TOUCH_MINOR  0x31    /* (omit if circular) */
    #define ABS_MT_WIDTH_MAJOR  0x32    /* approaching ellipse */
    #define ABS_MT_WIDTH_MINOR  0x33    /* (omit if circular) */
    #define ABS_MT_ORIENTATION  0x34    /* Ellipse orientation */
    #define ABS_MT_POSITION_X   0x35    /* Center X ellipse position */
    #define ABS_MT_POSITION_Y   0x36    /* Center Y ellipse position */
    #define ABS_MT_TOOL_TYPE    0x37    /* Type of touching device */
    #define ABS_MT_BLOB_ID      0x38    /* Group set of pkts as blob */
    #endif /* ABS_MT_TOUCH_MAJOR */
#endif /* ABS_MT_TOUCH_MAJOR and FT_USE_MT_SIGNALS */

#if defined(ABS_MT_TRACKING_ID)  && defined(FT_USE_MT_TRACK_ID)
    #define FT_USE_TRACKING_ID  1
#else
    #define FT_USE_TRACKING_ID  0
/* define only if not defined already by system;
 * value based on linux kernel 2.6.30.10
 */
#ifndef ABS_MT_TRACKING_ID
    #define ABS_MT_TRACKING_ID  (ABS_MT_BLOB_ID+1)
#endif

#endif /* ABS_MT_TRACKING_ID */

#ifdef FT_USE_DEEP_SLEEP
    #define FT_USE_DEEP_SLEEP_SEL   0x80
#else
    #define FT_USE_DEEP_SLEEP_SEL   0x00
#endif

#ifdef FT_USE_LOW_POWER
    #define FT_USE_SLEEP    (FT_USE_DEEP_SLEEP_SEL | 0x01)
#else
    #define FT_USE_SLEEP    0x00
#endif /* FT_USE_LOW_POWER */

/* helper macros */
#define GET_NUM_TOUCHES(x)      ((x) & 0x0F)
#define GET_TOUCH1_ID(x)        (((x) & 0xF0) >> 4)
#define GET_TOUCH2_ID(x)        ((x) & 0x0F)
#define GET_TOUCH3_ID(x)        (((x) & 0xF0) >> 4)
#define GET_TOUCH4_ID(x)        ((x) & 0x0F)
#define IS_LARGE_AREA(x)        (((x) & 0x10) >> 4)
#define FLIP_DATA_FLAG          0x01
#define REVERSE_X_FLAG          0x02
#define REVERSE_Y_FLAG          0x04
#define FLIP_DATA(flags)        ((flags) & FLIP_DATA_FLAG)
#define REVERSE_X(flags)        ((flags) & REVERSE_X_FLAG)
#define REVERSE_Y(flags)        ((flags) & REVERSE_Y_FLAG)
#define FLIP_XY(x, y)       { \
                                u16 tmp; \
                                tmp = (x); \
                                (x) = (y); \
                                (y) = tmp; \
                            }
#define INVERT_X(x, xmax)       ((xmax) - (x))
#define INVERT_Y(y, ymax)       ((ymax) - (y))
#define SET_HSTMODE(reg, mode)  ((reg) & (mode))
#define GET_HSTMODE(reg)        ((reg & 0x70) >> 4)
#define GET_BOOTLOADERMODE(reg) ((reg & 0x10) >> 4)

/* constant definitions */
/* maximum number of concurrent ST track IDs */
#define FT_NUM_ST_TCH_ID        2

/* maximum number of concurrent MT track IDs */
#define FT_NUM_MT_TCH_ID        4

/* maximum number of track IDs */
#define FT_NUM_TRK_ID           16

#define FT_NTCH                 0   /* no touch (lift off) */
#define FT_TCH                  1   /* active touch (touchdown) */
#define FT_ST_FNGR1_IDX         0
#define FT_ST_FNGR2_IDX         1
#define FT_MT_TCH1_IDX          0
#define FT_MT_TCH2_IDX          1
#define FT_MT_TCH3_IDX          2
#define FT_MT_TCH4_IDX          3
#define FT_XPOS                 0
#define FT_YPOS                 1
#define FT_IGNR_TCH             (-1)
#define FT_SMALL_TOOL_WIDTH     10
#define FT_LARGE_TOOL_WIDTH     255
#define FT_MAXZ                 255


struct i2c_client;

struct ft5x06_platform_data {
	u32 maxx;
	u32 maxy;
	u32 flags;
    u32 reset_gpio;
	u8 gen;
	u8 use_st;
	u8 use_mt;
	u8 use_hndshk;
	u8 use_trk_id;
	u8 use_sleep;
	u8 use_gestures;
	u8 gest_set;
	u8 act_intrvl;
	u8 tch_tmout;
	u8 lp_intrvl;
	u8 power_state;
	s32 (*init)(struct i2c_client *client);
	s32 (*resume)(struct i2c_client *client);
	void (*platform_suspend)(void);
	void (*platform_resume)(void);
};

struct ft5x06_xydata_t {
    u8 gest_id;
    u16 x1 __attribute__ ((packed));
    u16 y1 __attribute__ ((packed));
	u16 x2 __attribute__ ((packed));
	u16 y2 __attribute__ ((packed));
};
#endif /* __FT5x06_H__ */
