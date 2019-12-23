/********************************************************************************************************
 * @file     app.c 
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

unsigned long firmwareVersion;

#define SPI_MASTER_DEVICE		1
#define SPI_SLAVE_DEVICE		2
#define SPI_DEVICE		        2

/**************************************SPI MASTER MODE****************************************************/
#if (SPI_DEVICE==SPI_MASTER_DEVICE)

#define BUFF_DATA_LEN    		16
#define SPI_CS_PIN				GPIO_PC2//SPI CS pin
#define SLAVE_ADDR				0x8000
#define SLAVE_ADDR_LEN			2

volatile unsigned char spi_tx_buff[BUFF_DATA_LEN]={0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};

volatile unsigned char spi_rx_buff[BUFF_DATA_LEN]={0x00};

/*********************************************************************************************************/

void user_init()
{
	delay_ms(2000);
    //1.init the LED pin,for indication
	gpio_set_func(LED1 ,AS_GPIO);
	gpio_set_output_en(LED1, 1); 		//enable output
	gpio_set_input_en(LED1 ,0);			//disable input
	gpio_write(LED1, 0);              	//LED On

	spi_master_init((unsigned char)(CLOCK_SYS_CLOCK_HZ/(2*500000)-1),SPI_MODE0);//div_clock. spi_clk = sys_clk/((div_clk+1)*2),mode select
	spi_master_set_pin(SPI_GPIO_GROUP_C2C3C4C5);//master mode: spi pin set

}

void main_loop (void)
{
	delay_ms(1000);   //1S
	spi_tx_buff[0] ++;
	gpio_toggle(LED1);
	spi_write_buff(SLAVE_ADDR, SLAVE_ADDR_LEN,(unsigned char*)spi_tx_buff, BUFF_DATA_LEN,SPI_CS_PIN);
	spi_read_buff(SLAVE_ADDR, SLAVE_ADDR_LEN,(unsigned char*)spi_rx_buff, BUFF_DATA_LEN,SPI_CS_PIN);
}

/**************************************SPI SLAVE MODE******************************************************/

#elif (SPI_DEVICE==SPI_SLAVE_DEVICE)

void user_init()
{
	delay_ms(1000);  //leave enough time for SWS_reset when power on

    //1.init the LED pin,for indication
	gpio_set_func(LED1 ,AS_GPIO);
	gpio_set_output_en(LED1, 1); 		//enable output
	gpio_set_input_en(LED1 ,0);			//disable input
	gpio_write(LED1, 0);              	//LED On

	gpio_set_func(LED2 ,AS_GPIO);
	gpio_set_output_en(LED2, 1); 		//enable output
	gpio_set_input_en(LED2 ,0);			//disable input
	gpio_write(LED2, 0);              	//LED On

	spi_slave_init((unsigned char)(CLOCK_SYS_CLOCK_HZ/(2*500000)-1),SPI_MODE0);           //slave mode init
	spi_slave_set_pin(SPI_GPIO_GROUP_C2C3C4C5);      //slave mode spi pin set
	reg_irq_mask |= FLD_IRQ_MIX_CMD_EN;
	irq_enable();

}
void main_loop (void)
{

}

#endif





