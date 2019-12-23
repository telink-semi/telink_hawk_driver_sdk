/********************************************************************************************************
 * @file     app_config.h 
 *
 * @brief    This is the source file for TLSR8232
 *
 * @author	 junwei.lu@telink-semi.com;
 * @date     May 8, 2018
 *
 * @par      Copyright (c) 2018, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *           The information contained herein is confidential property of Telink
 *           Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *           of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *           Co., Ltd. and the licensee or the terms described here-in. This heading
 *           MUST NOT be removed from this file.
 *
 *           Licensees are granted free, non-transferable use of the information in this
 *           file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 * @par      History:
 * 			 1.initial release(DEC. 26 2018)
 *
 * @version  A001
 *         
 *******************************************************************************************************/
#pragma once
#include "drivers.h"
/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

#define LED1     		        GPIO_PB0
#define LED2     		        GPIO_PA7
#define LED3     		        GPIO_PB1
#define LED4     		        GPIO_PB2

#define SW1      		        GPIO_PA1
#define SW2      		        GPIO_PA2


/* IDLE MODE */
#define IDLE_TIMER_WAKEUP				1
/* SUSPEND MODE */
#define SUSPEND_PAD_WAKEUP   			2		// 7.8uA
#define SUSPEND_32K_RC_WAKEUP   		3		// 7.8uA
#define SUSPEND_32K_XTAL_WAKEUP			4		// 10.8uA,
/* DEEPSLEEP MODE */
#define DEEP_PAD_WAKEUP		 			5     	// 3uA
#define DEEP_32K_RC_WAKEUP      		6		// 3uA
#define DEEP_32K_XTAL_WAKEUP      		7		// 5.7uA

#define DEEP_32K_RC_WAKEUP_LONG      	8		// 3uA
#define DEEP_32K_XTAL_WAKEUP_LONG      	9		// 5.7uA

#define PM_MODE			     			1


/* Define system clock */
#define CLOCK_SYS_CLOCK_HZ  	16000000		// define system clock

#if(CLOCK_SYS_CLOCK_HZ==12000000)
	#define SYS_CLK  	SYS_CLK_12M_XTAL
#elif (CLOCK_SYS_CLOCK_HZ==16000000)
	#define SYS_CLK  	SYS_CLK_16M_XTAL
#elif (CLOCK_SYS_CLOCK_HZ==24000000)
	#define SYS_CLK  	SYS_CLK_24M_XTAL
#elif (CLOCK_SYS_CLOCK_HZ==32000000)
	#define SYS_CLK  	SYS_CLK_32M_XTAL
#elif (CLOCK_SYS_CLOCK_HZ==48000000)
	#define SYS_CLK  	SYS_CLK_48M_XTAL
#endif

/* List tick per second/millisecond/microsecond */
enum{
	CLOCK_SYS_CLOCK_1S = CLOCK_SYS_CLOCK_HZ,				///< system tick per 1 second
	CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),		///< system tick per 1 millisecond
	CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),   ///< system tick per 1 microsecond
};












/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif
