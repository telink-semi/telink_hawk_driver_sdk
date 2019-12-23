/********************************************************************************************************
 * @file     app.c 
 *
 * @brief    This is the source file for TLSR8258
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
#include "app_config.h"

#define WAKEUP_PAD						SW1
#define CURRENT_TEST	     			1

void user_init()
{
	delay_ms(2000);

#if CURRENT_TEST
	gpio_shutdown(GPIO_ALL);
#else
	//1.init the LED pin,for indication
	gpio_set_func(LED1 ,AS_GPIO);
	gpio_set_output_en(LED1, 1); 		//enable output
	gpio_set_input_en(LED1 ,0);			//disable input
	gpio_write(LED1, 0);              	//LED On

	gpio_set_func(LED2 ,AS_GPIO);
	gpio_set_output_en(LED2, 1); 		//enable output
	gpio_set_input_en(LED2 ,0);			//disable input
	gpio_write(LED2, 0);              	//LED On

	gpio_write(LED1,1);
	delay_ms(100);
	gpio_write(LED1,0);
	delay_ms(100);

	gpio_write(LED2,1);
	delay_ms(100);
	gpio_write(LED2,0);
	delay_ms(100);
#endif

#if(PM_MODE==IDLE_TIMER_WAKEUP)



#elif(PM_MODE==SUSPEND_PAD_WAKEUP)
	/* Caution: if wake-up source is only pad, 32K clock source MUST be 32K RC
	 * 			and CLK_32K_XTAL_EN in pm.h Must be disabled
	 * */
	clock_32k_init(CLK_32K_RC);						// 32k clock source is 32k RC by default

	pm_set_gpio_wakeup(WAKEUP_PAD, LEVEL_LOW, 1);
	gpio_set_up_down_resistor(WAKEUP_PAD, PM_PIN_PULLUP_10K);

#elif(PM_MODE==SUSPEND_32K_RC_WAKEUP)

#elif(PM_MODE==SUSPEND_32K_XTAL_WAKEUP)
	/* Caution:
	 * 			CLK_32K_XTAL_EN in pm.h Must be enabled
	 * */
	clock_32k_init(CLK_32K_XTAL);// Be set for ONLY one time

#elif(PM_MODE==DEEP_PAD_WAKEUP)
	/* Caution: if wake-up source is only pad, 32K clock source MUST be 32K RC
	 * 			and CLK_32K_XTAL_EN in pm.h Must be disabled
	 * */
	clock_32k_init(CLK_32K_RC);						// 32k clock source is 32k RC by default

	pm_set_gpio_wakeup(WAKEUP_PAD, LEVEL_LOW, 1);
	gpio_set_up_down_resistor(WAKEUP_PAD, PM_PIN_PULLUP_10K);

	pm_sleep_wakeup(DEEPSLEEP_MODE , PM_WAKEUP_PAD, 0);

#elif(PM_MODE==DEEP_32K_RC_WAKEUP)

    pm_sleep_wakeup(DEEPSLEEP_MODE , PM_WAKEUP_TIMER,(get_sys_tick() + 1000*CLOCK_SYS_TIMER_CLK_1MS));

#elif(PM_MODE==DEEP_32K_XTAL_WAKEUP)
	/* Caution:
	 * 			CLK_32K_XTAL_EN in pm.h Must be enabled
	 * */
	if(pm_is_wakeup_status()==PM_DEEPSLEEP_NO_WAKEUP)
	clock_32k_init(CLK_32K_XTAL);// Be set for ONLY one time

    pm_sleep_wakeup(DEEPSLEEP_MODE , PM_WAKEUP_TIMER,(get_sys_tick() + 1000*CLOCK_SYS_TIMER_CLK_1MS));

#elif(PM_MODE==DEEP_32K_RC_WAKEUP_LONG)

    pm_long_sleep_wakeup(DEEPSLEEP_MODE , PM_WAKEUP_TIMER,1000000);

#elif(PM_MODE==DEEP_32K_XTAL_WAKEUP_LONG)
	/* Caution:
	 * 			CLK_32K_XTAL_EN in pm.h Must be enabled
	 * */
	if(pm_is_wakeup_status()==PM_DEEPSLEEP_NO_WAKEUP)
	clock_32k_init(CLK_32K_XTAL);// Be set for ONLY one time

	pm_long_sleep_wakeup(DEEPSLEEP_MODE , PM_WAKEUP_TIMER,1000000);

#endif

}


/////////////////////////////////////////////////////////////////////
// main loop flow
/////////////////////////////////////////////////////////////////////
void main_loop (void)
{

#if(PM_MODE==IDLE_TIMER_WAKEUP)

	mcu_stall_wakeup_by_timer0(1000*CLOCK_SYS_CLOCK_1MS);

#elif(PM_MODE==SUSPEND_PAD_WAKEUP)

	pm_sleep_wakeup(SUSPEND_MODE, PM_WAKEUP_PAD, 0);

#elif(PM_MODE==SUSPEND_32K_RC_WAKEUP)
	gpio_write(LED2,1);
	pm_sleep_wakeup(SUSPEND_MODE, PM_WAKEUP_TIMER, get_sys_tick() + 3000*CLOCK_SYS_TIMER_CLK_1MS);
	gpio_write(LED2,0);
#elif(PM_MODE==SUSPEND_32K_XTAL_WAKEUP)
	gpio_write(LED2,1);
	pm_sleep_wakeup(SUSPEND_MODE, PM_WAKEUP_TIMER, get_sys_tick() + 3000*CLOCK_SYS_TIMER_CLK_1MS);
	gpio_write(LED2,0);
#endif

	delay_ms(3000);

}



