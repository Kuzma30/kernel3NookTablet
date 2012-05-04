#ifndef _TEMPHACK_H
#define _TEMPHACK_H

#define	TEMP_HACK //We use GPIO 121 as backlight output without PWM, and can use GPTimer 11 for Ducati

#ifdef	TEMP_HACK
#define	DUCATI_WDT_TIMER_1	9
#define	DUCATI_WDT_TIMER_2	11
#else
#define	DUCATI_WDT_TIMER_1	10
#define	DUCATI_WDT_TIMER_2	9
#endif

#endif