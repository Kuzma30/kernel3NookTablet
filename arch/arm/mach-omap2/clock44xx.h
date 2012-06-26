/*
 * OMAP4 clock function prototypes and macros
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 * Copyright (C) 2010 Nokia Corporation
 */

#ifndef __ARCH_ARM_MACH_OMAP2_CLOCK44XX_H
#define __ARCH_ARM_MACH_OMAP2_CLOCK44XX_H

/*
 * XXX Missing values for the OMAP4 DPLL_USB
 * XXX Missing min_multiplier values for all OMAP4 DPLLs
 */
#define OMAP4470_MAX_DPLL_MULT	4095
#define OMAP4470_MAX_DPLL_DIV	256
#define OMAP4430_MAX_DPLL_MULT	2047
#define OMAP4430_MAX_DPLL_DIV	128
#define OMAP4430_REGM4XEN_MULT 4

int omap4xxx_clk_init(void);
int omap4_core_dpll_m2_set_rate(struct clk *clk, unsigned long rate);

#ifdef CONFIG_OMAP4_DPLL_CASCADING
int omap4_core_dpll_set_rate(struct clk *clk, unsigned long rate);
#endif

#endif
