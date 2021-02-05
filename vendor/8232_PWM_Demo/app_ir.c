/********************************************************************************************************
 * @file	app_ir.c
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

#if(PWM_MODE==PWM_IR)
/*********************************************************************************
	PWM0   :  PA0.  PB3
	PWM0_N :  PB6	PC2
 *********************************************************************************/

#define PWM_PIN					GPIO_PA0
#define AS_PWMx					AS_PWM0
#define PWM_ID					PWM0
#define PWM_PULSE_NUM			4

#define TX_GROUP_NUM			6
volatile unsigned char n;

_attribute_ram_code_ void irq_handler(void)
{

   if(pwm_get_irq_status(PWM0_PNUM_IRQ))
   {
	  pwm_clr_irq_status(PWM0_PNUM_IRQ);
	  n++;
	  if(n==(TX_GROUP_NUM-1))
	  {
		pwm_set_mode(PWM_ID, PWM_COUNT_MODE);		//  5 group of IR and 1 group of count
	  }
	  gpio_toggle(LED3);
   }

}



void user_init()
{
	delay_ms(2000);
	gpio_set_func(LED1 ,AS_GPIO);
	gpio_set_output_en(LED1, 1);
	gpio_set_func(LED2 ,AS_GPIO);
	gpio_set_output_en(LED2, 1);
	gpio_set_func(LED3 ,AS_GPIO);
	gpio_set_output_en(LED3, 1);
	pwm_set_clk(CLOCK_SYS_CLOCK_HZ, CLOCK_SYS_CLOCK_HZ);

	gpio_set_func(PWM_PIN, AS_PWMx);
	pwm_set_mode(PWM_ID, PWM_IR_MODE);
	pwm_set_pulse_num(PWM_ID,PWM_PULSE_NUM);
	pwm_set_max_and_cmp(PWM_ID, 2*CLOCK_SYS_CLOCK_1MS,1*CLOCK_SYS_CLOCK_1MS);

	pwm_set_irq_en(PWM0_PNUM_IRQ,1);
	irq_set_mask(FLD_IRQ_SW_PWM_EN);
	irq_enable();
    pwm_start(PWM_ID);

}



void main_loop (void)
{
	delay_ms(500);

	gpio_toggle(LED2);
}

#endif


