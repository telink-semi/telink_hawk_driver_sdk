/********************************************************************************************************
 * @file     app_config.h 
 *
 * @brief    This is the header file for TLSR8232
 *
 * @author	 peng.sun@telink-semi.com;yang.ye@telink-semi.com
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

#define UART_DMA  		1     //uart use dma
#define UART_NDMA  		2     //uart not use dma

#define UART_MODE		2

#define NORMAL	   		1
#define USE_CTS    		2
#define USE_RTS    		3

#define FLOW_CTR  		1

#if( FLOW_CTR==USE_CTS)
	#define STOP_VOLT   	1			//0 :Low level stops TX.  1 :High level stops TX.
#endif

#if (FLOW_CTR==USE_RTS)
	#define RTS_MODE		UART_RTS_MODE_AUTO 		    //It can be UART_RTS_MODE_AUTO/UART_RTS_MODE_MANUAL.
	#define RTS_THRESH		4			//UART_RTS_MODE_AUTO need.It indicates RTS trigger threshold.
	#define RTS_INVERT		1			//UART_RTS_MODE_AUTO need.0 indicates RTS_pin will change from low to hign.
	#define RTS_POLARITY	0			//UART_RTS_MODE_MANUAL need. It indicates RTS_POLARITY .
#endif





/////////////////// Clock  /////////////////////////////////
#define CLOCK_SYS_CLOCK_HZ  	24000000

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

enum{
	CLOCK_SYS_CLOCK_1S = CLOCK_SYS_CLOCK_HZ,
	CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),
	CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),
};










/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif