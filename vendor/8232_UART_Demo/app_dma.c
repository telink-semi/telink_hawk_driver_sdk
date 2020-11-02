/********************************************************************************************************
 * @file     app.c
 *
 * @brief    This is the source file for TLSR8232
 *
 * @author	 peng.sun@telink-semi.com;yang.ye@telink-semi.com
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

#if (UART_MODE==UART_DMA)



#define rec_buff_Len    16
#define trans_buff_Len  16

__attribute__((aligned(4))) unsigned char rec_buff[rec_buff_Len]={0};

// dma len must be 4 byte
__attribute__((aligned(4))) unsigned char trans_buff[trans_buff_Len] = {0x0c, 0x00, 0x00, 0x00, 0x11, 0x22, 0x33, 0x44,

																0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc};
volatile unsigned char uart_dmairq_tx_cnt;
volatile unsigned char uart_dmairq_rx_cnt;


void user_init()
{
	delay_ms(2000);  //leave enough time for SWS_reset when power on

	//note: dma addr must be set first before any other uart initialization! (confirmed by sihui)
	uart_set_recbuff( (unsigned short *)&rec_buff, sizeof(rec_buff));

	uart_set_pin(UART_TX_PB4, UART_RX_PB5);// uart tx/rx pin set

	uart_reset();  //will reset uart digital registers from 0x90 ~ 0x9f, so uart setting must set after this reset

	//baud rate: 9600
	uart_init_baudrate(115200,CLOCK_SYS_CLOCK_HZ,PARITY_NONE, STOP_BIT_ONE);

	uart_dma_en(1, 1); 	//uart data in hardware buffer moved by dma, so we need enable them first

	irq_set_mask(FLD_IRQ_DMA_EN);

	dma_set_irq_en(FLD_DMA_CHN_UART_RX | FLD_DMA_CHN_UART_TX, 1);   	//uart Rx/Tx dma irq enable

	uart_irq_en(0, 0);  	//uart Rx/Tx irq no need, disable them


}

void main_loop (void)
{
	delay_ms(1000);
	uart_dma_send((unsigned short*)&rec_buff);
	delay_ms(300);
	uart_dma_send((unsigned short*)&trans_buff);
}

#endif














