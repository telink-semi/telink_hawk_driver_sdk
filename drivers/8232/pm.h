/********************************************************************************************************
 * @file     pm.h 
 *
 * @brief    This is the header file for TLSR8258
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

#include "bsp.h"
#include "gpio.h"

#define XTAL_EXCEPTION_FIX_EN              1

#define PM_LONG_SLEEP_WAKEUP_EN			0		//if user need to make MCU sleep for a long time that is more than 268s, this macro need to be enabled and use "pm_long_sleep_wakeup" function

#define PM_PAD_WAKEUP_16US_FILTER_EN	0
/**
 * @brief analog register below can store infomation when MCU in deepsleep mode
 * 	      store your information in these ana_regs before deepsleep by calling analog_write function
 * 	      when MCU wakeup from deepsleep, read the information by by calling analog_read function
 * 	      Reset these analog registers only by power cycle
 */

#define DEEP_ANA_REG0    0x34				//initial value =0x00
#define DEEP_ANA_REG1    0x35				//initial value =0x00
#define DEEP_ANA_REG2    0x36				//initial value =0x00
#define DEEP_ANA_REG3    0x37				//initial value =0x00
#define DEEP_ANA_REG4    0x38				//initial value =0x00
#define DEEP_ANA_REG5    0x39				//initial value =0xff
/**
 * @brief these analog register can store data in deepsleep mode or deepsleep with SRAM retention mode.
 * 	      Reset these analog registers by watchdog, chip reset, RESET Pin, power cycle
 */

#define DEEP_ANA_REG6    0x3a				//initial value =0x00
#define DEEP_ANA_REG7    0x3b				//initial value =0x00
#define DEEP_ANA_REG8    0x3c				//initial value =0x00
#define DEEP_ANA_REG9    0x3d				//initial value =0x00
#define DEEP_ANA_REG10   0x3e				//initial value =0x00
#define DEEP_ANA_REG11   0x3f				//initial value =0x6f
/**
 * @brief sleep mode.
 */
typedef enum {
	SUSPEND_MODE						= 0,
	DEEPSLEEP_MODE						= 0x80,
	SHUTDOWN_MODE						= 0xFF,
}SleepMode_TypeDef;


/**
 * @brief   wakeup source
 */

typedef enum {
	 PM_WAKEUP_PAD   = BIT(4),
	 PM_WAKEUP_CORE  = BIT(5),
	 PM_WAKEUP_TIMER = BIT(6),
}SleepWakeupSrc_TypeDef;

/**
 * @brief   wakeup status
 */

enum {
	 WAKEUP_STATUS_TIMER  			= BIT(1),
	 PM_STATUS_CORE  	        	= BIT(2),
	 WAKEUP_STATUS_PAD    			= BIT(3),
	 STATUS_GPIO_ERR_NO_ENTER_PM  	= BIT(7),

	 STATUS_ENTER_SUSPEND			= BIT(30),
};

/**
 * @brief   Deepsleep wakeup status
 */

enum{
	PM_DEEPSLEEP_NO_WAKEUP			= 0,// Firstly Power on
	PM_DEEPSLEEP_PAD_WAKEUP			= 1,
	PM_DEEPSLEEP_TIMER_WAKEUP		= 2,
};

/**
 * @brief   This function serves to wake up cpu from stall mode by timer0.
 * @param   tick - capture value of timer0.
 * @return  none.
 */
void mcu_stall_wakeup_by_timer0(unsigned int tick);

/**
 * @brief   This function serves to wake up cpu from stall mode by timer1.
 * @param   tick - capture value of timer1.
 * @return  none.
 */
void mcu_stall_wakeup_by_timer1(unsigned int tick);

/**
 * @brief   This function serves to wake up cpu from stall mode by timer2.
 * @param   tick - capture value of timer2.
 * @return  none.
 */
void mcu_stall_wakeup_by_timer2(unsigned int tick);

/**
 * @brief   This function serves to get the 32k tick.
 * @param   none
 * @return  variable of 32k tick.
 */
_attribute_ram_code_ unsigned int pm_get_32k_tick (void);

/**
 * @brief   This function serves to set the 32k tick.
 * @param   variable of 32k tick.
 * @return  none.
 */
_attribute_ram_code_ void pm_set_32k_tick (unsigned int tick);

/***
 * brief: this function can get the 32K count value in real time.
 * pm_get_32k_tick has while(flag),if 32K crystal not vibration, it will stop in while(flag).
 * but get_32k_tick not occur this situation.
 */
_attribute_ram_code_ unsigned int get_32k_xtal_tick(void);

_attribute_ram_code_ unsigned int get_32k_xtal_set_value(void);

/**
 * @brief      This function configures a GPIO pin as the wakeup pin.
 * @param[in]  pin - the pin needs to be configured as wakeup pin
 * @param[in]  pol - the wakeup polarity of the pad pin(0: low-level wakeup, 1: high-level wakeup)
 * @param[in]  en  - enable or disable the wakeup function for the pan pin(1: Enable, 0: Disable)
 * @return     none
 */
#if(BLE_SDK_EN)
void cpu_set_gpio_wakeup (GPIO_PinTypeDef pin, GPIO_LevelTypeDef pol, int en);
#else
void pm_set_gpio_wakeup (GPIO_PinTypeDef pin, GPIO_LevelTypeDef pol, int en);
#endif
/**
 * @brief      This function servers to wake up the cpu from sleep mode.
 * @param[in]  sleep_mode - sleep mode type select.
 * @param[in]  wakeup_src - wake up source select.
 * @param[in]  wakeup_tick - the time of wake up.
 * @return     indicate whether the cpu is wake up successful.
 */
#if(BLE_SDK_EN)
int cpu_sleep_wakeup (SleepMode_TypeDef sleep_mode, SleepWakeupSrc_TypeDef wakeup_src, unsigned int  wakeup_tick);
#else
int pm_sleep_wakeup (SleepMode_TypeDef sleep_mode, SleepWakeupSrc_TypeDef wakeup_src, unsigned int wakeup_tick);
#endif
/**
 * @brief      This function serves to determine whether wake up source is the specified wake up source.
 * @param[in]  none.
 * @return     refer to Deepsleep wakeup status .
 */
int pm_is_wakeup_status(void);


#if PM_LONG_SLEEP_WAKEUP_EN
/**
 * @brief      This function servers to wake up the cpu from sleep mode.
 * @param[in]  sleep_mode - sleep mode type select.
 * @param[in]  wakeup_src - wake up source select.
 * @param[in]  SleepDurationUs - the time of sleep.
 * @return     indicate whether the cpu is wake up successful.
 */
int pm_long_sleep_wakeup (SleepMode_TypeDef sleep_mode, SleepWakeupSrc_TypeDef wakeup_src, unsigned int  SleepDurationUs);
#endif


/** \defgroup GP7  PM Examples
 *
 * 	@{
 */

/*! \page pm Table of Contents
	- [API-PM-CASE1:IDLE TIMER WAKEUP](#IDLE_TIMER_WAKEUP)
	- [API-PM-CASE2:SUSPEND PAD WAKEUP](#SUSPEND_PAD_WAKEUP)
	- [API-PM-CASE3:SUSPEND 32K WAKEUP](#SUSPEND_32K_WAKEUP)
	- [API-PM-CASE4:DEEP PAD WAKEUP](#DEEP_PAD_WAKEUP)
	- [API-PM-CASE5:DEEP 32K WAKEUP](#DEEP_32K_WAKEUP)
	- [API-PM-CASE6:SHUT DOWN](#SHUTDOWN)

<h1 id=IDLE_TIMER_WAKEUP> API-PM-CASE1:IDLE TIMER WAKEUP </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | none || user initialization | ^ |
| ^ | main_loop() | mcu_stall_wakeup_by_timer0() | mcu_stall_wakeup_by_timer0(100*CLOCK_SYS_CLOCK_1MS) | let MCU enter idle mode and wait for being waked up by Timer0 | ^ |
| ^ | ^ | delay_ms() | delay_ms(3000) | delay 3000ms after MCU waking up  | ^ |

<h1 id=SUSPEND_PAD_WAKEUP> API-PM-CASE2:SUSPEND PAD WAKEUP </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | pm_set_gpio_wakeup() | pm_set_gpio_wakeup(GPIO_PB0, LEVEL_HIGH, 1) | set the specified pin as GPIO wakeup source | ^ |
| ^ | ^ | gpio_set_up_down_resistor() | gpio_set_up_down_resistor(GPIO_PB0, PM_PIN_PULLDOWN_100K) | enable 100k pull-down resistor of the specified pin | ^ |
| ^ | main_loop() | pm_sleep_wakeup() | pm_sleep_wakeup(SUSPEND_MODE, PM_WAKEUP_PAD, 0) | let MCU enter suspend mode and wait for being waked up by PB0 | ^ |
| ^ | ^ | delay_ms() | delay_ms(3000) | delay 3000ms after MCU waking up  | ^ |

<h1 id=SUSPEND_32K_WAKEUP> API-PM-CASE3:SUSPEND 32K WAKEUP </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | none || user initialization | ^ |
| ^ | main_loop() | pm_sleep_wakeup() | pm_sleep_wakeup(SUSPEND_MODE, PM_WAKEUP_TIMER, <br> get_sys_tick() + 100*CLOCK_SYS_TIMER_CLK_1MS) | let MCU enter suspend mode and wait for being waked up by 32k Timer | ^ |
| ^ | ^ | delay_ms() | delay_ms(3000) | delay 3000ms after MCU waking up  | ^ |

<h1 id=DEEP_PAD_WAKEUP> API-PM-CASE4:DEEP PAD WAKEUP </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | pm_set_gpio_wakeup() | pm_set_gpio_wakeup(GPIO_PB0, LEVEL_HIGH, 1) | set the specified pin as GPIO wakeup source | ^ |
| ^ | ^ | gpio_set_up_down_resistor() | gpio_set_up_down_resistor(GPIO_PB0, PM_PIN_PULLDOWN_100K) | enable 100k pull-down resistor of the specified pin | ^ |
| ^ | ^ | pm_sleep_wakeup() | pm_sleep_wakeup(DEEPSLEEP_MODE , PM_WAKEUP_PAD, 0) | let MCU enter deepsleep mode and wait for being waked up by PB0 | ^ |
| ^ | main_loop() | none || Main program loop | ^ |

<h1 id=DEEP_32K_WAKEUP> API-PM-CASE5:DEEP 32K WAKEUP </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | pm_sleep_wakeup() | pm_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_TIMER, <br> get_sys_tick() + 100*CLOCK_SYS_TIMER_CLK_1MS) |  let MCU enter deepsleep mode and wait for being waked up by 32k Timer | ^ |
| ^ | main_loop() | none || Main program loop | ^ |

<h1 id=SHUTDOWN> API-PM-CASE6:SHUT DOWN </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | pm_sleep_wakeup() | pm_sleep_wakeup(SHUTDOWN_MODE , 0,0) |  let MCU enter shutdown mode | ^ |
| ^ | main_loop() | None || Main program loop | ^ |

<h1> History Record </h1>

| Date | Description | Author |
| :--- | :---------- | :----- |
| 2019-1-10 | initial release | LJW |


*/

 /** @}*/ //end of GP7
