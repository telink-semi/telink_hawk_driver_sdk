/********************************************************************************************************
 * @file     main.c 
 *
 * @brief    This is the source file for TLSR8232
 *
 * @author	 peng.sun@telink-semi.com;
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
 *         
 *******************************************************************************************************/
#include "app_config.h"

extern void user_init();
extern void main_loop (void);

int irq_cnt = 0;
int byte_irq_cnt=0;
/**
 * @brief		This function serves to handle the interrupt of MCU
 * @param[in] 	none
 * @return 		none
 */
_attribute_ram_code_ void irq_handler(void)
{
	irq_cnt ++;

	unsigned char  irq_status = reg_spi_slave_irq_status;
	//Byte Interrupt means that every byte will generate one interrupt
	if(irq_status & FLD_SPI_BYTE_IRQ)
	{
		reg_spi_slave_irq_status = irq_status;
		byte_irq_cnt ++;
		gpio_toggle(LED1);
	}

}
/**
 * @brief		This is main function
 * @param[in]	none
 * @return      none
 */
_attribute_ram_code_ int main (void)   //must on ramcode
{
	system_init(BSP_INTERNAL_CAP_ENABLE);

	clock_init(SYS_CLK);

	//rf_drv_init(RF_MODE_BLE_1M);

	gpio_init();

	user_init();

	irq_enable();

	while (1) {
		main_loop ();
	}

	return 0;
}

