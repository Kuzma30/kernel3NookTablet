/*
 * LPDDR2 data as per JESD209-2
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 *
 * Kamel Slimani <k-slimani@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <mach/emif.h>

const struct lpddr2_timings timings_samsung_400_mhz = {
	.max_freq	= 400000000,
	.RL		= 6,
	.tRPab		= 21,
	.tRCD		= 18,
	.tWR		= 15,
	.tRASmin	= 42,
	.tRRD		= 10,
	.tWTRx2		= 15,
	.tXSR		= 140,
	.tXPx2		= 15,
	.tRFCab		= 130,
	.tRTPx2		= 15,
	.tCKE		= 3,
	.tCKESR		= 15,
	.tZQCS		= 90,
	.tZQCL		= 360,
	.tZQINIT	= 1000,
	.tDQSCKMAXx2	= 11,
	.tRASmax	= 70,
	.tFAW		= 50
};

const struct lpddr2_timings timings_samsung_200_mhz = {
	.max_freq	= 200000000,
	.RL		= 3,
	.tRPab		= 21,
	.tRCD		= 18,
	.tWR		= 15,
	.tRASmin	= 42,
	.tRRD		= 10,
	.tWTRx2		= 20,
	.tXSR		= 140,
	.tXPx2		= 15,
	.tRFCab		= 130,
	.tRTPx2		= 15,
	.tCKE		= 3,
	.tCKESR		= 15,
	.tZQCS		= 90,
	.tZQCL		= 360,
	.tZQINIT	= 1000,
	.tDQSCKMAXx2	= 11,
	.tRASmax	= 70,
	.tFAW		= 50
};

const struct lpddr2_min_tck min_tck_samsung = {
	.tRL		= 3,
	.tRP_AB		= 3,
	.tRCD		= 3,
	.tWR		= 3,
	.tRAS_MIN	= 3,
	.tRRD		= 2,
	.tWTR		= 2,
	.tXP		= 2,
	.tRTP		= 2,
	.tCKE		= 3,
	.tCKESR		= 3,
	.tFAW		= 8
};

struct lpddr2_device_info samsung_4G_S4 = {
	.device_timings = {
		&timings_samsung_200_mhz,
		&timings_samsung_400_mhz
	},
	.min_tck	= &min_tck_samsung,
	.type		= LPDDR2_TYPE_S4,
	.density	= LPDDR2_DENSITY_4Gb,
	.io_width	= LPDDR2_IO_WIDTH_32
};


/*
 * Timing regs for Samsung K3PE4E400P
 */

const struct lpddr2_timings timings_samsung_2G_400_mhz = {
	.max_freq       = 400000000,
	.RL             = 6,  /* TODO: Read Latency = 8 */
	.tRPab          = 21, /* Row Precharge Time(all banks) */
	.tRCD           = 18, /* RAS to CAS Delay */
	.tWR            = 15, /* Write Recovery Time */
	.tRASmin        = 42, /* Row Active Time */
	.tRRD           = 10, /* Active bank A to Active bank B */
	.tWTRx2         = 15, /* (Internal Write to Read Command Delay) * 2 */
	.tXSR           = 140, /* tRFCab + 10 */
	.tXPx2          = 15,  /* tXP(Exit power down to next valid command delay) * 2 */
	.tRFCab         = 130, /* Refresh Cycle time */
	.tRTPx2         = 15,  /* Internal Read to Precharge command delay * 2 */
	.tCKE           = 3,   /* CKE min. pulse width (high and low pulse width) */
	.tCKESR         = 15,  /* CKE min. pulse width during self-refresh */
	.tZQCS          = 90,  /* Short Calibration Time*14 */
	.tZQCL          = 360, /* Full Calibration Time*14 */
	.tZQINIT        = 1000, /* Initialization Calibration Time*14  */
	.tDQSCKMAXx2    = 11,   /* DQS output access time from CK/CK# * 2 / 1000 */
	.tRASmax        = 70,  /* Row Active Time MAX */
	.tFAW           = 50 /* LPDDR2 ns (Four Bank Activate Window) */
};

const struct lpddr2_timings timings_samsung_2G_200_mhz = {
	.max_freq       = 200000000,
	.RL             = 3,
	.tRPab          = 21,
	.tRCD           = 18,
	.tWR            = 15,
	.tRASmin        = 42,
	.tRRD           = 10,
	.tWTRx2         = 20,
	.tXSR           = 140,
	.tXPx2          = 15,
	.tRFCab         = 130,
	.tRTPx2         = 15,
	.tCKE           = 3,
	.tCKESR         = 15,
	.tZQCS          = 90,
	.tZQCL          = 360,
	.tZQINIT        = 1000,
	.tDQSCKMAXx2    = 11,
	.tRASmax        = 70,
	.tFAW           = 50 /* LPDDR2 ns (Four Bank Activate Window) */
};

const struct lpddr2_min_tck min_tck_samsung_2G = {
	.tRL		= 3, /* Read Latency */
	.tRP_AB		= 3, /* Row Precharge Time(all banks) */
	.tRCD		= 3, /* RAS to CAS Delay */
	.tWR		= 3, /* Write Recovery Time */
	.tRAS_MIN	= 3, /* Row Active Time */
	.tRRD		= 2, /* Active bank A to Active bank B */
	.tWTR		= 2, /* Internal Write to Read Command Delay */
	.tXP		= 2, /* Exit power down to next valid command delay */
	.tRTP		= 2, /* Internal Read to Precharge command delay */
	.tCKE		= 3, /* CKE min. pulse width (high and low pulse width) */
	.tCKESR		= 3, /* CKE min. pulse width during self-refresh */
	.tFAW		= 8  /* Four Bank Activate Window */
};

struct lpddr2_device_info samsung_2G_S4 = {
	.device_timings = {
		&timings_samsung_2G_200_mhz,
		&timings_samsung_2G_400_mhz
	},
	.min_tck	= &min_tck_samsung_2G,
	.type		= LPDDR2_TYPE_S4,
	.density	= LPDDR2_DENSITY_2Gb,
	.io_width	= LPDDR2_IO_WIDTH_32
};

