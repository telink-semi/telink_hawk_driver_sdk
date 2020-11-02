/********************************************************************************************************
 * @file     main.c 
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


extern void user_init();
extern void main_loop (void);
volatile unsigned int gpio_irq_cnt;

/**
 * @brief		This function serves to handle the interrupt of MCU
 * @param[in] 	none
 * @return 		none
 */
_attribute_ram_code_ void irq_handler(void)
{
#if (GPIO_MODE == GPIO_IRQ )

	if((reg_irq_src & FLD_IRQ_GPIO_EN)==FLD_IRQ_GPIO_EN){
		reg_irq_src |= FLD_IRQ_GPIO_EN; // clear the relevant irq
		if(gpio_read(SW1)== 0){ // press key with low level to flash light
			delay_ms(10);
			if(gpio_read(SW1)== 0){
				gpio_irq_cnt++;
				gpio_toggle(LED1);
			}
		}
	}

#elif(GPIO_MODE == GPIO_IRQ_RSIC0)

	if((reg_irq_src & FLD_IRQ_GPIO_RISC0_EN)==FLD_IRQ_GPIO_RISC0_EN){
		reg_irq_src |= FLD_IRQ_GPIO_RISC0_EN; // clear the relevant irq

		if(gpio_read(SW2)== 0){ // press key with low level to flash light
			delay_ms(10);
			if(gpio_read(SW2)== 0){
				gpio_irq_cnt++;
				gpio_toggle(LED1);
			}
		}
	}

#elif(GPIO_MODE == GPIO_IRQ_RSIC1)

	if((reg_irq_src & FLD_IRQ_GPIO_RISC1_EN)==FLD_IRQ_GPIO_RISC1_EN){
		reg_irq_src |= FLD_IRQ_GPIO_RISC1_EN; // clear the relevant irq

		if(gpio_read(SW2)== 0){ // press key with low level to flash light
			delay_ms(10);
			if(gpio_read(SW2)== 0){
				gpio_irq_cnt++;
				gpio_toggle(LED1);
			}
		}
	}

#elif(GPIO_MODE == GPIO_IRQ_RSIC2)

	if((reg_irq_src & FLD_IRQ_GPIO_RISC2_EN)==FLD_IRQ_GPIO_RISC2_EN){
		reg_irq_src |= FLD_IRQ_GPIO_RISC2_EN; // clear the relevant irq

		if(gpio_read(SW2)== 0){ // press key with low level to flash light
			delay_ms(10);
			if(gpio_read(SW2)== 0){
				gpio_irq_cnt++;
				gpio_toggle(LED1);
			}
		}
	}
#endif

}

/**
 * @brief		This is main function
 * @param[in]	none
 * @return      none
 */
int main (void)   //must on ramcode
{
	system_init(BSP_INTERNAL_CAP_ENABLE);

	clock_init(SYS_CLK);

//	rf_mode_init(RF_MODE_BLE_1M);

	gpio_init();

	user_init();

	irq_enable();

	while (1) {
		main_loop ();
	}

	return 0;
}
