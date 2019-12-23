/********************************************************************************************************
 * @file     putchar.h
 *
 * @brief    This is the head file for TLSR8258
 *
 * @author	 yang,ye@telink-semi.com;
 * @date     December 5, 2018
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

#include "clock.h"
#include "pm.h"
#include "register.h"

/**
 * qedc input channel type of A
 */
typedef enum
{
	PC1A,
	PC2A,
	PC3A,
	PB0A,
	PB1A,
	PB2A,
	PA1A,
	PA2A,

}QDEC_InputAchTypeDef;

/**
 * qedc input channel type of A
 */
typedef enum
{
	PC1B,
	PC2B,
	PC3B,
	PB0B,
	PB1B,
	PB2B,
	PA1B,
	PA2B,
}QDEC_InputBchTypeDef;


#define reg_qdec_set       0xd1
#define reg_qdec_channel_a 0xd2
#define reg_qdec_channel_b 0xd3

#define reg_qdec_mode 0xd7

/**
 * qedc mode
 */
typedef enum
{
	COMMON_MODE,
	DOUBLE_ACCURACY_MODE,
}QDEC_ModeTypeDef;

#define rge_qdec_load 	0xd8
#define reg_qdec_count 	0xd0
#define reg_qdec_reset 0xd6

/**
 * @brief      This function servers to set input port.
 * @param[in]  QDEC_InputAchTypeDef - input types of A channel.
 * @param[in]  QDEC_InputBchTypeDef - input types of A channel.
 * @return     none.
 */
void qdec_set_pin(QDEC_InputAchTypeDef channelA,QDEC_InputBchTypeDef channelB);

/**
 * @brief      This function servers to set qdec's mode.
 * @param[in]  QDEC_ModeTypeDef - mode type to select.
 * @return     none.
 */
void qdec_set_mode(QDEC_ModeTypeDef mode);

/**
 * @brief      This function servers to initials qedc source clock.
 * @param[in]  none.
 * @return     none.
 */
void qdec_clk_en(void);

/**
 * @brief      This function servers to read hardware counting value.
 * @param[in]  none.
 * @return     hardware counting value.
 */
unsigned char qdec_get_count_value(void);

/**
 * @brief      This function servers to reset the counter.
 * @param[in]  none.
 * @return     none.
 */
void qdec_clear_conuter(void);

/**
 * @brief      This function servers to set hardware debouncing.
 * @param[in]  thrsh - lower the value of thrsh,will be regard as jitter.
 * @return     none.
 */
void qdec_set_debouncing(char thrsh);

/** \defgroup GP9 QDEC Examples
 *
 * 	@{
 */

/*! \page qdec Table of Contents
	- [API-QDEC-CASE1:QDEC COMMON](#QDEC_COMMON)
	- [API-QDEC-CASE2:QDEC DOUBLE ACCURACY](#QDEC_DOUBLE_ACCURACY)
	- [API-QDEC-CASE3:QDEC SHUTTLE](#QDEC_SHUTTLE)

<h1 id=QDEC_COMMON> API-QDEC-CASE1:QDEC COMMON </h1>



<h1 id=QDEC_DOUBLE_ACCURACY> API-QDEC-CASE2:QDEC DOUBLE ACCURACY </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | gpio_set_func() | gpio_set_func(QDEC_CHA, AS_GPIO) | set the specified pin as GPIO input  | ^ |
| ^ | ^ | gpio_set_output_en() | gpio_set_output_en(QDEC_CHA,0) | ^ | ^ |
| ^ | ^ | gpio_set_input_en() | gpio_set_input_en(QDEC_CHA,1) | ^ | ^ |
| ^ | ^ | gpio_set_func() | gpio_set_func(QDEC_CHB, AS_GPIO) | set the specified pin as GPIO input | ^ |
| ^ | ^ | gpio_set_output_en() | gpio_set_output_en(QDEC_CHB,0) | ^ | ^ |
| ^ | ^ | gpio_set_input_en() | gpio_set_input_en(QDEC_CHB,1) | ^ | ^ |
| ^ | ^ | qdec_clk_en() || set the clock frequency of QDEC | ^ |
| ^ | ^ | qdec_set_mode() | qdec_set_mode(DOUBLE_ACCURACY_MODE) | set QDEC work in the double accuracy mode | ^ |
| ^ | ^ | qdec_set_pin() | qdec_set_pin(PB6A,PB7A) | select pin for QDEC | ^ |
| ^ | ^ | qdec_set_debouncing() | qdec_set_debouncing(1) | enable debouncing for QDEC | ^ |
| ^ | main_loop() | qdec_count = qdec_get_count_value() || get the count value of QDEC | ^ |

\n
Variables above are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
#define QDEC_CHA 	GPIO_PB6
#define QDEC_CHB 	GPIO_PB7

unsigned char qdec_count = 0;
~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=QDEC_SHUTTLE> API-QDEC-CASE3:QDEC SHUTTLE </h1>







<h1> History Record </h1>

| Date | Description | Author |
| :--- | :---------- | :----- |
| 2019-1-10 | initial release | LJW |


*/

 /** @}*/ //end of GP8
