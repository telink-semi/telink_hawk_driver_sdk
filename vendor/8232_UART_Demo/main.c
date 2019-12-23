/********************************************************************************************************
 * @file     main.c 
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

extern void user_init();
extern void main_loop (void);
#define rec_buff_Len    16
#define trans_buff_Len    16
volatile unsigned int cnt=0;


extern volatile unsigned char uart_dmairq_tx_cnt;
extern volatile unsigned char uart_dmairq_rx_cnt;
extern volatile unsigned char uart_rx_flag;
extern volatile unsigned int  uart_ndmairq_cnt;
extern volatile unsigned char uart_ndmairq_index;
extern __attribute__((aligned(4))) unsigned char rec_buff[rec_buff_Len];
extern __attribute__((aligned(4))) unsigned char trans_buff[trans_buff_Len];


_attribute_ram_code_ void irq_handler(void)
{

#if (UART_MODE==UART_DMA)

	unsigned char uart_dma_irqsrc;
	//1. UART irq
	uart_dma_irqsrc = dma_get_irq_status();///in function,interrupt flag have already been cleared,so need not to clear DMA interrupt flag here


	if(uart_dma_irqsrc & FLD_DMA_CHN_UART_RX){
		dma_clr_irq_status(FLD_DMA_CHN_UART_RX);

		uart_dmairq_rx_cnt++;

		//Received uart data in rec_buff, user can copy data here

	}
	if(uart_dma_irqsrc & FLD_DMA_CHN_UART_TX){
		dma_clr_irq_status(FLD_DMA_CHN_UART_TX);

		uart_dmairq_tx_cnt++;
	}

#elif(UART_MODE==UART_NDMA)
	#if( FLOW_CTR == NORMAL)
		static unsigned char uart_ndma_irqsrc;
		uart_ndma_irqsrc = uart_ndma_get_irq();  ///get the status of uart irq.
		if(uart_ndma_irqsrc){

	//cycle the four registers 0x90 0x91 0x92 0x93,in addition reading will clear the irq.
		if(uart_rx_flag==0){
			rec_buff[uart_ndmairq_cnt++] = reg_uart_data_buf(uart_ndmairq_index);
			uart_ndmairq_index++;
			uart_ndmairq_index &= 0x03;// cycle the four registers 0x90 0x91 0x92 0x93, it must be done like this for the design of SOC.
			if(uart_ndmairq_cnt%16==0&&uart_ndmairq_cnt!=0){
				uart_rx_flag=1;
			}
		}
		else{
			READ_REG8(0x90+ uart_ndmairq_index);
			uart_ndmairq_index++;
			uart_ndmairq_index &= 0x03;
		}
	}
#endif

#endif

}
/**
 * @brief		This is main function
 * @param[in]	none
 * @return      none
 */
int main (void)   //must on ramcode
{
	system_init();

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


