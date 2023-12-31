/********************************************************************************************************
 * @file	app_ble_1m.c
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

#if(RF_MODE==RF_BLE_1M)
#define TX_FIRST				1
#define RX_FIRST				2
#define RF_STRX_MODE			TX_FIRST

#define STX_WAITTIME_US         4000
#define SRX_WAITTIME_US         100000
#define TX_DELAY_US             10

#define RF_FREQ					35
#define RF_POWER				RF_POWER_7dBm
#define ACCESS_CODE				0x29417671




volatile unsigned int rx_cnt=0;
volatile unsigned int tx_cnt=0;
volatile unsigned int timeout_cnt=0;
volatile unsigned int tx_state=0;
volatile unsigned int rx_state=0;
volatile unsigned int timeout_state=0;
unsigned char  rx_packet[64]  __attribute__ ((aligned (4)));
unsigned char  ble_tx_packet[48] __attribute__ ((aligned (4))) = {0x23,0x00,0x00,0x00,0x00,0x21,0x00,0x00,0x00,0x00,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};



void user_init()
{
	//1.init the LED pin,for indication
	gpio_set_func(LED1,AS_GPIO);
	gpio_set_output_en(LED1, 1); //enable output
	gpio_set_input_en(LED1,0);	 //disable input
	gpio_write(LED1, 0);         //LED Off

	gpio_set_func(LED2,AS_GPIO);
	gpio_set_output_en(LED2, 1); //enable output
	gpio_set_input_en(LED2 ,0);	 //disable input
	gpio_write(LED2, 0);         //LED Off

	gpio_set_func(LED3,AS_GPIO);
	gpio_set_output_en(LED3, 1); //enable output
	gpio_set_input_en(LED3,0);	 //disable input
	gpio_write(LED3, 0);         //LED Off

	rf_set_power_level_index (RF_POWER);
	rf_set_trx_state(RF_MODE_AUTO,RF_FREQ);
	rf_set_rx_buff(rx_packet,64, 0);

#if(RF_STRX_MODE==TX_FIRST)
    irq_disable();
    irq_clr_all_src();
    irq_set_mask(FLD_IRQ_ZB_RT_EN); //enable RF irq
    rf_irq_clr_mask(FLD_RF_IRQ_ALL);
    rf_irq_set_mask(FLD_RF_IRQ_TX|FLD_RF_IRQ_RX|FLD_RF_IRQ_TX2RX_RX_TIMEOUT);//stx2rx irq interrupt
    irq_enable(); //enable general irq
#elif(RF_STRX_MODE==RX_FIRST)
    irq_disable();
    irq_clr_all_src();
    irq_set_mask(FLD_IRQ_ZB_RT_EN); //enable RF irq
    rf_irq_clr_mask(FLD_RF_IRQ_ALL);
    rf_irq_set_mask(FLD_RF_IRQ_TX|FLD_RF_IRQ_RX|FLD_RF_IRQ_RX2TX_RX_TIMEOUT);//srx2tx irq interrupt
    irq_enable(); //enable general irq
#endif

	rf_set_acc_code(ACCESS_CODE);
}

void main_loop (void)
{
#if(RF_STRX_MODE==TX_FIRST)
	while(1)
	{
		tx_state=0;
		rx_state=0;
		timeout_state=0;
		delay_ms(100);
		rf_start_stx2rx (ble_tx_packet, get_sys_tick()+16*TX_DELAY_US,STX_WAITTIME_US);
		while(1)
		{
			if(tx_state==1)//tx
			{
				tx_state = 0;
				tx_cnt++;
			}
			else if(rx_state==1)//rx
			{
				if(RF_BLE_PACKET_CRC_OK(rx_packet)&&RF_BLE_PACKET_LENGTH_OK(rx_packet))
				rx_cnt++;
				break;

			}
			else if(timeout_state==1)//time out
			{
				timeout_cnt++;
				break;
			}
		}
	}


#elif(RF_STRX_MODE==RX_FIRST)

	while(1)
	{
		tx_state=0;
		rx_state=0;
		timeout_state=0;
		rf_start_srx2tx (ble_tx_packet, get_sys_tick()+16*TX_DELAY_US ,SRX_WAITTIME_US);
		while(1)
		{
			if(tx_state==1)//tx
			{
				tx_state = 0;
				tx_cnt++;
				break;
			}
			else if(rx_state==1)//rx (check crc & length)
			{
				rx_state=0;
				if(RF_BLE_PACKET_CRC_OK(rx_packet)&&RF_BLE_PACKET_LENGTH_OK(rx_packet))
				rx_cnt++;
			}
			else if(timeout_state==1)//time out
			{
				timeout_state = 0;
				timeout_cnt++;
				break;
			}
		}
	}
#endif
}

#endif


