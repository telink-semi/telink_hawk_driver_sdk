/********************************************************************************************************
 * @file     app_ir_fifo.c
 *
 * @brief    This is the source file for TLSR8258
 *
 * @author	 junyuan.zhang@telink-semi.com;junwei.lu@telink-semi.com
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

#if(PWM_MODE==PWM_IR_FIFO)
/*********************************************************************************
	PWM0   :  PA0.  PB3
	PWM0_N :  PB6	PC2
 *********************************************************************************/

#define PWM_PIN					GPIO_PA0
#define AS_PWMx					AS_PWM0
#define PWM_ID					PWM0
#define PWM_PULSE_NUM1			5
#define PWM_PULSE_NUM2			5
#define PWM_PULSE_NUM3			10
_attribute_ram_code_ void irq_handler(void)
{

	if(pwm_get_irq_status(PWM0_IR_FIFO_DONE_IRQ))
	{
        pwm_ir_fifo_set_data_entry(PWM_PULSE_NUM1,0,1);
		pwm_ir_fifo_set_data_entry(PWM_PULSE_NUM2,0,0);

		pwm_clr_irq_status(PWM0_IR_FIFO_DONE_IRQ);
        gpio_toggle(LED3);
		pwm_start(PWM_ID);

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
	pwm_set_max_and_cmp(PWM_ID, 2*CLOCK_SYS_CLOCK_1MS,1*CLOCK_SYS_CLOCK_1MS);
	pwm_set_shadow_max_and_cmp(PWM_ID,4, 2);
	pwm_set_mode(PWM_ID, PWM_IR_FIFO_MODE);

	pwm_ir_fifo_set_data_entry(PWM_PULSE_NUM1,0,1);
	pwm_ir_fifo_set_data_entry(PWM_PULSE_NUM2,0,0);
	pwm_ir_fifo_set_data_entry(PWM_PULSE_NUM3,1,1);
	pwm_ir_fifo_set_irq_trig_level(1);
	pwm_set_irq_en(PWM0_IR_FIFO_DONE_IRQ,1);
	irq_set_mask(FLD_IRQ_SW_PWM_EN);
	irq_enable();
    pwm_start(PWM_ID);
}



void main_loop (void)
{
	delay_ms(50);
}

#endif

