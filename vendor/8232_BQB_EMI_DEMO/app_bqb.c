/********************************************************************************************************
 * @file	app_bqb.c
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

#if(TEST_DEMO==BQB_DEMO)
	
#include "drivers.h"
#include "common.h"
#include "BQB/phytest.h"	

unsigned long firmwareVersion;


#if DEVELOPER_TEST
#define LED1					GPIO_PB0
#endif

#define rec_buff_Len    256
__attribute__((aligned(4))) unsigned char rec_buff[rec_buff_Len]={0};

#if BQB_SUPPORT_USR_CONFIG
extern User_Config_u user_cfg;
#endif

void user_init(void){
#if DEVELOPER_TEST
	//init the LED pin,for indication
	gpio_set_func(LED1 ,AS_GPIO);
	gpio_set_output_en(LED1, 1); 		//enable output
	gpio_set_input_en(LED1 ,0);			//disable input
	gpio_write(LED1, 1);              	//LED On
#endif

	uart_set_recbuff( (unsigned short *)&rec_buff, sizeof(rec_buff));
#if BQB_SUPPORT_USR_CONFIG
	//UART 	TX: UART_TX_PA3, UART_TX_PB4, UART_TX_PC4
	//		RX:	UART_RX_PA4, UART_RX_PB5, UART_RX_PC5
	switch(user_cfg.uart)
	{
	case UART_TX_PA3_RX_PA4:
		uart_set_pin(UART_TX_PA3, UART_RX_PA4);
		break;
	case UART_TX_PA3_RX_PB5:
		uart_set_pin(UART_TX_PA3, UART_RX_PB5);
		break;
	case UART_TX_PA3_RX_PC5:
		uart_set_pin(UART_TX_PA3, UART_RX_PC5);
		break;
	case UART_TX_PB4_RX_PA4:
		uart_set_pin(UART_TX_PB4, UART_RX_PA4);
		break;
	case UART_TX_PB4_RX_PB5:
		uart_set_pin(UART_TX_PB4, UART_RX_PB5);
		break;
	case UART_TX_PB4_RX_PC5:
		uart_set_pin(UART_TX_PB4, UART_RX_PC5);
		break;
	case UART_TX_PC4_RX_PA4:
		uart_set_pin(UART_TX_PC4, UART_RX_PA4);
		break;
	case UART_TX_PC4_RX_PB5:
		uart_set_pin(UART_TX_PC4, UART_RX_PB5);
		break;
	case UART_TX_PC4_RX_PC5:
		uart_set_pin(UART_TX_PC4, UART_RX_PC5);
		break;
	default:
		break;
	}
#else
	uart_set_pin(BQB_UART_TX, BQB_UART_RX);
#endif

	uart_reset();  //will reset uart digital registers from 0x90 ~ 0x9f, so uart setting must set after this reset
	uart_init_baudrate(115200,CLOCK_SYS_CLOCK_HZ,PARITY_NONE,STOP_BIT_ONE);
	uart_dma_en(0, 0);
	irq_clr_mask(FLD_IRQ_DMA_EN);
	dma_set_irq_en(FLD_DMA_CHN_UART_RX | FLD_DMA_CHN_UART_TX, 0);
	uart_irq_en(0,0);
	uart_ndma_set_triglevel(1,0);	//set the trig level. 1 indicate one byte will occur interrupt

	rf_mode_init(RF_MODE);

	// customize TP0/TP1
	unsigned long cust_tp_info_addr = 0, cust_cap_info_addr = 0;
#if BQB_SUPPORT_USR_CONFIG
	if(user_cfg.f_size == FLASH_512K)
	{
		cust_tp_info_addr = CUST_TP_INFO_ADDR_512K;
		cust_cap_info_addr = CUST_CAP_INFO_ADDR_512K;
	}
	else if(user_cfg.f_size == FLASH_128K)
	{
		cust_tp_info_addr = CUST_TP_INFO_ADDR_128K;
		cust_cap_info_addr = CUST_CAP_INFO_ADDR_128K;
	}
	else
	{
		cust_tp_info_addr = CUST_TP_INFO_ADDR_64K;
		cust_cap_info_addr = CUST_CAP_INFO_ADDR_64K;
	}
#else
	cust_tp_info_addr = CUST_TP_INFO_ADDR;
	cust_cap_info_addr = CUST_CAP_INFO_ADDR;
#endif

	unsigned char usr_tp0 = 0xff, usr_tp1 = 0xff, usr_cap = 0xff;
	flash_read_page(cust_tp_info_addr, 1, &usr_tp0);
	flash_read_page(cust_tp_info_addr + 1, 1, &usr_tp1);
	flash_read_page(cust_cap_info_addr, 1, &usr_cap);

	if( (usr_tp0 != 0xff) && (usr_tp1 != 0xff) ){
		rf_set_tp(usr_tp0, usr_tp1);
	}

	if( usr_cap != 0xff ){
		if(internal_cap_flag)
		{
			//ana_81<4:0> is cap value(0x00 - 0x1f)
			analog_write(0x81, (analog_read(0x81)&0xe0) | (usr_cap & 0x1f) );
		}
	}
#if SPECIAL_PROCESS
	WRITE_REG8(0x402, 0x29);// for frequency drift
	analog_write(0xaa, 0x86);//for Rx C/I
	analog_write(0xac, 0xac);//for Rx C/I
//	analog_write(0x81, 0xda);//
	analog_write(0x8b, 0x44);//optimization the fdev
	analog_write(0x8f, 0x2b);//optimization the fdev
#endif
	phy_test_init();
}

void main_loop(void){
	phytest ();
}



#endif
