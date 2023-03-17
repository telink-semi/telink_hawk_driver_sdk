/********************************************************************************************************
 * @file	app_pri_2m.c
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
#if(RF_MODE==RF_PRIVATE_2M)

#define TX					1
#define RX					2
#define RF_TRX_MODE			TX

#define AUTO  				1
#define MANUAL				2
#define RF_AUTO_MODE 		MANUAL

#define TPLL_MODE  			1
#define SB_MODE   			2
#define PRIVATE_MODE			SB_MODE

#define RX_PAYLOAD_LEN		32

#define RF_FREQ				35
#define RF_POWER			RF_POWER_7dBm
#define ACCESS_CODE			0x29417671
volatile unsigned int rx_cnt=0;
volatile unsigned int tx_cnt=0;
unsigned char  rx_packet[64]  __attribute__ ((aligned (4)));
unsigned char  Private_SB_tx_packet[48] __attribute__ ((aligned (4))) = {0x20,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};
unsigned char  Private_TPLL_tx_packet[48] __attribute__ ((aligned (4))) = {0x21,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};

#if(RF_AUTO_MODE == AUTO)

#define TX_INTERVAL_MS    1

void user_init()
{
	//1.init the LED pin,for indication
	gpio_set_func(LED1 ,AS_GPIO);
	gpio_set_output_en(LED1, 1); //enable output
	gpio_set_input_en(LED1 ,0);	 //disable input
	gpio_write(LED1, 0);         //LED On

	rf_set_power_level_index (RF_POWER);
	rf_set_tx_rx_off_auto();
	rf_set_channel(RF_FREQ,0);

	rf_set_acc_code(ACCESS_CODE);

#if(PRIVATE_MODE == TPLL_MODE)

#elif(PRIVATE_MODE == SB_MODE)
	rf_fix_payload_len_set(RX_PAYLOAD_LEN);
#endif

}

void main_loop (void)
{
#if(RF_TRX_MODE==TX)
	while(1)
	{
		delay_ms(1);
	#if(PRIVATE_MODE == TPLL_MODE)
		rf_start_stx (Private_TPLL_tx_packet, get_sys_tick() + 16*1000*TX_INTERVAL_MS);
	#elif(PRIVATE_MODE == SB_MODE)
		rf_start_stx (Private_SB_tx_packet, get_sys_tick() + 16*1000*TX_INTERVAL_MS);
	#endif
		while(!rf_is_tx_finish());
		rf_clr_tx_finish();
		tx_cnt++;
	}


#elif(RF_TRX_MODE==RX)
	rf_set_rx_buff(rx_packet,64, 0);
	rf_start_srx(get_sys_tick() + 100*16);

	while(1)
	{
		if(rf_is_rx_finish())
		{
		#if(PRIVATE_MODE == TPLL_MODE)
			if(RF_TPLL_PACKET_CRC_OK(rx_packet)&&RF_TPLL_PACKET_LENGTH_OK(rx_packet))
		#elif(PRIVATE_MODE == SB_MODE)
			if(RF_SB_PACKET_CRC_OK(rx_packet))
		#endif
			{
				gpio_toggle(LED1);
				rx_cnt++;
			}
			rf_clr_rx_finish();
			rf_set_tx_rx_off_auto();
			rf_start_srx(get_sys_tick() + 100*16);
		}
	}
#endif
}


#elif(RF_AUTO_MODE == MANUAL)
void user_init()
{
	//1.init the LED pin,for indication
	gpio_set_func(LED1 ,AS_GPIO);
	gpio_set_output_en(LED1, 1); //enable output
	gpio_set_input_en(LED1 ,0);//disable input
	gpio_write(LED1, 0);              //LED On

	rf_set_power_level_index (RF_POWER);
	rf_set_tx_rx_off();
	rf_set_channel(RF_FREQ,0);

	rf_set_acc_code(ACCESS_CODE);

#if(PRIVATE_MODE == TPLL_MODE)

#elif(PRIVATE_MODE == SB_MODE)
	rf_fix_payload_len_set(RX_PAYLOAD_LEN);
#endif

}

void main_loop (void)
{
#if(RF_TRX_MODE==TX)
	rf_set_tx_on();
	while(1)
	{
		delay_ms(1);
	#if(PRIVATE_MODE == TPLL_MODE)
		rf_tx_pkt (Private_TPLL_tx_packet);
	#elif(PRIVATE_MODE == SB_MODE)
		rf_tx_pkt (Private_SB_tx_packet);
	#endif
		while(!rf_is_tx_finish());
		rf_clr_tx_finish();
		tx_cnt++;
	}


#elif(RF_TRX_MODE==RX)
	rf_set_rx_buff(rx_packet,64, 0);
	rf_set_rx_on ();

	while(1)
	{
		if(rf_is_rx_finish())
		{
		#if(PRIVATE_MODE == TPLL_MODE)
			if(RF_TPLL_PACKET_CRC_OK(rx_packet)&&RF_TPLL_PACKET_LENGTH_OK(rx_packet))
		#elif(PRIVATE_MODE == SB_MODE)
			if(RF_SB_PACKET_CRC_OK(rx_packet))
		#endif
			{
				gpio_toggle(LED1);
				rx_cnt++;
			}
			rf_clr_rx_finish();
			rf_set_tx_rx_off_auto();
		}
	}
#endif
}
#endif
#endif
