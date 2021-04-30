/********************************************************************************************************
 * @file	app.c
 *
 * @brief	This is the source file for TLSR8232
 *
 * @author	Driver Group
 * @date	May 8, 2018
 *
 * @par     Copyright (c) 2018, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *          All rights reserved.
 *
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions are met:
 *
 *              1. Redistributions of source code must retain the above copyright
 *              notice, this list of conditions and the following disclaimer.
 *
 *              2. Unless for usage inside a TELINK integrated circuit, redistributions
 *              in binary form must reproduce the above copyright notice, this list of
 *              conditions and the following disclaimer in the documentation and/or other
 *              materials provided with the distribution.
 *
 *              3. Neither the name of TELINK, nor the names of its contributors may be
 *              used to endorse or promote products derived from this software without
 *              specific prior written permission.
 *
 *              4. This software, with or without modification, must only be used with a
 *              TELINK integrated circuit. All other usages are subject to written permission
 *              from TELINK and different commercial license may apply.
 *
 *              5. Licensee shall be solely responsible for any claim to the extent arising out of or
 *              relating to such deletion(s), modification(s) or alteration(s).
 *
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *          ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *          WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *          DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
 *          DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *          LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *          ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *          (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *          SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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



