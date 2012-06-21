#ifndef _TEMPHACK_H
#define _TEMPHACK_H

//#define	TEMP_HACK //We use GPIO 121 as backlight output without PWM, and can use GPTimer 11 for Ducati
#undef TEMP_HACK
#ifdef	TEMP_HACK
#define	DUCATI_WDT_TIMER_1	9
#define	DUCATI_WDT_TIMER_2	11
#define	DUCATI_FIRMWARE_NAME	"ducati-m3.bin"
#else
#define	DUCATI_WDT_TIMER_1	10
#define	DUCATI_WDT_TIMER_2	9
#define	DUCATI_FIRMWARE_NAME	"ducati-m3.bin"
#endif

#endif