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
int i2c_read_cnt = 0;
int i2c_write_cnt = 0;
_attribute_ram_code_ void irq_handler(void)
{
	irq_cnt ++;

	unsigned char  irq_status = reg_i2c_slave_irq_status;

	if(irq_status & FLD_HOST_CMD_IRQ)
	{
		reg_i2c_slave_irq_status = irq_status; //clear all irq status

		if(irq_status & FLD_HOST_READ_IRQ)
		{
			i2c_read_cnt ++;
			gpio_toggle(LED1);
		}
		else
		{
			i2c_write_cnt ++;
			gpio_toggle(LED2);
		}
	}

}

/**
 * @brief		This is main function
 * @param[in]	none
 * @return      none
 */
_attribute_ram_code_ int main (void)   //must on ramcode
{
	system_init();

	clock_init(SYS_CLK);

	//rf_drv_init(RF_MODE_BLE_1M);

	gpio_init();

	user_init();

	while (1) {
		main_loop ();
	}
}

