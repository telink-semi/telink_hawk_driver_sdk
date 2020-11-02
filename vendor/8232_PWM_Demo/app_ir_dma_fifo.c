/********************************************************************************************************
 * @file     app_ir_dma_fifo.c
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

#if(PWM_MODE==PWM_IR_DMA_FIFO)
/*********************************************************************************
	PWM0   :  PA0.  PB3
	PWM0_N :  PB6	PC2
 *********************************************************************************/

#define PWM_PIN					GPIO_PA0
#define AS_PWMx					AS_PWM0
#define PWM_ID					PWM0

#define IR_DMA_CARRIER_FREQ				38000
#define IR_DMA_MAX_TICK					(CLOCK_SYS_CLOCK_HZ/IR_DMA_CARRIER_FREQ)
#define IR_DMA_CMP_TICK					(IR_DMA_MAX_TICK/2)

#define IR_DMA_SHADOW_CARRIER_FREQ		56000
#define IR_DMA_SHADOW_MAX_TICK			(CLOCK_SYS_CLOCK_HZ/IR_DMA_SHADOW_CARRIER_FREQ)
#define IR_DMA_SHADOW_CMP_TICK			(IR_DMA_SHADOW_MAX_TICK/2)

unsigned short IR_DMA_Buff[64]={0};
unsigned short IRQ_IR_DMA_Buff[20]={0};

unsigned int ir_dma_fifo_cnt=0;
/**
 * @brief		This function serves to handle the interrupt of MCU
 * @param[in] 	none
 * @return 		none
 */
_attribute_ram_code_ void irq_handler(void)
{

	if(pwm_get_irq_status(PWM0_IR_DMA_FIFO_DONE_IRQ))
	{
		pwm_clr_irq_status(PWM0_IR_DMA_FIFO_DONE_IRQ);
		ir_dma_fifo_cnt++;
		gpio_toggle(LED3);
		unsigned char irq_index=2;
		IRQ_IR_DMA_Buff[irq_index++]= pwm_ir_dma_fifo_set_waveform(1, PWM0_PULSE_NORMAL, 9000 * CLOCK_SYS_CLOCK_1US/IR_DMA_MAX_TICK);
		IRQ_IR_DMA_Buff[irq_index++]= pwm_ir_dma_fifo_set_waveform(1, PWM0_PULSE_NORMAL, 4500 * CLOCK_SYS_CLOCK_1US/IR_DMA_MAX_TICK);

		unsigned int irq_length = (irq_index-2)*2;
		unsigned char* irq_buff = &IRQ_IR_DMA_Buff[0];
		irq_buff[0]= irq_length&0xff;
		irq_buff[1]= (irq_length>>8)&0xff;
		irq_buff[2]= (irq_length>>16)&0xff;
		irq_buff[3]= (irq_length>>24)&0xff;
		pwm_set_dma_addr(&IRQ_IR_DMA_Buff);
		pwm_ir_dma_fifo_start_tx();

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
	pwm_set_mode(PWM_ID, PWM_IR_DMA_FIFO_MODE);
	pwm_set_max_and_cmp(PWM_ID, IR_DMA_MAX_TICK, IR_DMA_CMP_TICK);
	pwm_set_shadow_max_and_cmp(PWM_ID,IR_DMA_SHADOW_MAX_TICK, IR_DMA_SHADOW_CMP_TICK);
	unsigned char index=2;
	IR_DMA_Buff[index++]= pwm_ir_dma_fifo_set_waveform(1, PWM0_PULSE_NORMAL, 9000 * CLOCK_SYS_CLOCK_1US/IR_DMA_MAX_TICK);
	IR_DMA_Buff[index++]= pwm_ir_dma_fifo_set_waveform(0, PWM0_PULSE_NORMAL, 4500 * CLOCK_SYS_CLOCK_1US/IR_DMA_MAX_TICK);
	IR_DMA_Buff[index++]= pwm_ir_dma_fifo_set_waveform(1, PWM0_PULSE_NORMAL, 560 * CLOCK_SYS_CLOCK_1US/IR_DMA_MAX_TICK);
	IR_DMA_Buff[index++]= pwm_ir_dma_fifo_set_waveform(0, PWM0_PULSE_NORMAL, 560 * CLOCK_SYS_CLOCK_1US/IR_DMA_MAX_TICK);
	IR_DMA_Buff[index++]= pwm_ir_dma_fifo_set_waveform(1, PWM0_PULSE_NORMAL, 560 * CLOCK_SYS_CLOCK_1US/IR_DMA_MAX_TICK);
	IR_DMA_Buff[index++]= pwm_ir_dma_fifo_set_waveform(0, PWM0_PULSE_NORMAL, 1690 * CLOCK_SYS_CLOCK_1US/IR_DMA_MAX_TICK);
	IR_DMA_Buff[index++]= pwm_ir_dma_fifo_set_waveform(1, PWM0_PULSE_NORMAL, 560 * CLOCK_SYS_CLOCK_1US/IR_DMA_MAX_TICK);

	IR_DMA_Buff[index++]= pwm_ir_dma_fifo_set_waveform(1, PWM0_PULSE_SHADOW, 9000 * CLOCK_SYS_CLOCK_1US/IR_DMA_SHADOW_MAX_TICK);
	IR_DMA_Buff[index++]= pwm_ir_dma_fifo_set_waveform(0, PWM0_PULSE_SHADOW, 4500 * CLOCK_SYS_CLOCK_1US/IR_DMA_SHADOW_MAX_TICK);
	IR_DMA_Buff[index++]= pwm_ir_dma_fifo_set_waveform(1, PWM0_PULSE_SHADOW, 560 * CLOCK_SYS_CLOCK_1US/IR_DMA_SHADOW_MAX_TICK);
	IR_DMA_Buff[index++]= pwm_ir_dma_fifo_set_waveform(0, PWM0_PULSE_SHADOW, 560 * CLOCK_SYS_CLOCK_1US/IR_DMA_SHADOW_MAX_TICK);
	IR_DMA_Buff[index++]= pwm_ir_dma_fifo_set_waveform(1, PWM0_PULSE_SHADOW, 560 * CLOCK_SYS_CLOCK_1US/IR_DMA_SHADOW_MAX_TICK);
	IR_DMA_Buff[index++]= pwm_ir_dma_fifo_set_waveform(0, PWM0_PULSE_SHADOW, 1690 * CLOCK_SYS_CLOCK_1US/IR_DMA_SHADOW_MAX_TICK);
	IR_DMA_Buff[index++]= pwm_ir_dma_fifo_set_waveform(1, PWM0_PULSE_SHADOW, 560 * CLOCK_SYS_CLOCK_1US/IR_DMA_SHADOW_MAX_TICK);

	unsigned int length = (index-2)*2;
	unsigned char* buff = &IR_DMA_Buff[0];
	buff[0]= length&0xff;
	buff[1]= (length>>8)&0xff;
	buff[2]= (length>>16)&0xff;
	buff[3]= (length>>24)&0xff;
	pwm_set_dma_addr(&IR_DMA_Buff);

	//enable pwm0 ir dma fifo done irq
	pwm_set_irq_en(PWM0_IR_DMA_FIFO_DONE_IRQ,1);
	irq_set_mask(FLD_IRQ_SW_PWM_EN);
	irq_enable();
	pwm_ir_dma_fifo_start_tx();
}

void main_loop (void)
{
	delay_ms(50);
	gpio_toggle(LED2);
}

#endif
