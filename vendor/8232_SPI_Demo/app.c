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





