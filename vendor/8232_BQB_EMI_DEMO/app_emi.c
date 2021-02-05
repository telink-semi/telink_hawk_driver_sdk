/********************************************************************************************************
 * @file	app_emi.c
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

#if(TEST_DEMO==EMI_DEMO)
	
#include "drivers.h"
#include "common.h"

unsigned long firmwareVersion;

#define     RF_RSSI					0x8004
#define 	RF_TX_MODE				0X8005
#define		RUM_CMD					0x8006
#define		RIGIG_CMD				0X8007
#define 	RF_POWER_LEVEL			0X8008
#define 	RF_CHN					0x8009
#define 	RF_MODE					0x800a
#define 	RF_HOP_ENABLE			0x800b
#define     RF_REC_NUMB				0x800c

#define CAP_VALUE_512K					0x77000
#define TP_LOW_VALUE_512K				0x77040
#define TP_HIGH_VALUE_512K				0x77041

#define CAP_VALUE_128K					0x1E000
#define TP_LOW_VALUE_128K				0x1E040
#define TP_HIGH_VALUE_128K				0x1E041

#define CAP_VALUE_64K					0xE000
#define TP_LOW_VALUE_64K				0xE040
#define TP_HIGH_VALUE_64K				0xE041

#define EMI_CAP_VALUE					CAP_VALUE_512K
#define EMI_TP_LOW_VALUE				TP_LOW_VALUE_512K
#define EMI_TP_HIGH_VALUE				TP_HIGH_VALUE_512K

#if DEVELOPER_TEST
#define LED1					GPIO_PB0
#endif

struct  test_list_s {
	unsigned char  cmd_id;
	void	 (*func)(RF_ModeTypeDef rf_mode,RF_PowerTypeDef pwr,signed char rf_chn);
};

unsigned char  mode=1;//1
unsigned char  power_level = 0;
signed char  chn = 2;//2
unsigned char  cmd_now=1;//2
unsigned char  run=1;

unsigned char tx_mode = 0;

unsigned  char g_tp0;
unsigned  char g_tp1;

extern unsigned char internal_cap_flag;
void read_flash_para(void)
{
	if(internal_cap_flag)
	{
		unsigned char cap;
		flash_read_page(EMI_CAP_VALUE,1,&cap);

		if(cap != 0xff && cap > 0xbf && cap < 0xe0 )
		{
			analog_write(0x81,cap);
		}
	}

	flash_read_page(EMI_TP_LOW_VALUE,1,&g_tp0);
	flash_read_page(EMI_TP_HIGH_VALUE,1,&g_tp1);
}


void emi_init(void)
{
//	WRITE_REG32(0x408,	0x29417671 );//access code  0xf8118ac9

	WRITE_REG8(RUM_CMD,	run);//run
	WRITE_REG8(RIGIG_CMD,	cmd_now);//cmd
	WRITE_REG8(RF_POWER_LEVEL,	power_level);//power
	WRITE_REG8(RF_CHN,	chn);//chn
	WRITE_REG8(RF_MODE,	mode);//mode
	WRITE_REG8(RF_RSSI,	0);
	WRITE_REG8(RF_TX_MODE, tx_mode);
	WRITE_REG32(RF_REC_NUMB,	0);
}

void emicarrieronly(RF_ModeTypeDef rf_mode,RF_PowerTypeDef pwr,signed char rf_chn)
{
	rf_emi_single_tone(pwr,rf_chn);
	while( ((READ_REG8(RUM_CMD)) == run ) &&  ((READ_REG8(RIGIG_CMD)) == cmd_now )\
			&& ((READ_REG8(RF_POWER_LEVEL)) == power_level ) &&  ((READ_REG8(RF_CHN)) == chn )\
			&& ((READ_REG8(RF_MODE)) == mode ))
	{
	}
	rf_set_tx_rx_off();
}

void emi_con_prbs9(RF_ModeTypeDef rf_mode,RF_PowerTypeDef pwr,signed char rf_chn)
{
	rf_emi_tx_continue_setup(rf_mode,pwr,rf_chn,0);

	while( ((READ_REG8(RUM_CMD)) == run ) &&  ((READ_REG8(RIGIG_CMD)) == cmd_now )\
			&& ((READ_REG8(RF_POWER_LEVEL)) == power_level ) &&  ((READ_REG8(RF_CHN)) == chn )\
			&& ((READ_REG8(RF_MODE)) == mode ))
	{
		rf_continue_mode_run();
	}
	rf_set_tx_rx_off();
}

void emi_con_tx0f(RF_ModeTypeDef rf_mode,RF_PowerTypeDef pwr,signed char rf_chn)
{
	rf_emi_tx_continue_setup(rf_mode,pwr,rf_chn,1);

	while( ((READ_REG8(RUM_CMD)) == run ) &&  ((READ_REG8(RIGIG_CMD)) == cmd_now )\
			&& ((READ_REG8(RF_POWER_LEVEL)) == power_level ) &&  ((READ_REG8(RF_CHN)) == chn )\
			&& ((READ_REG8(RF_MODE)) == mode ))
	{
		rf_continue_mode_run();
	}
	rf_set_tx_rx_off();
}

void emi_con_tx55(RF_ModeTypeDef rf_mode,RF_PowerTypeDef pwr,signed char rf_chn)
{
	rf_emi_tx_continue_setup(rf_mode,pwr,rf_chn,2);

	while( ((READ_REG8(RUM_CMD)) == run ) &&  ((READ_REG8(RIGIG_CMD)) == cmd_now )\
			&& ((READ_REG8(RF_POWER_LEVEL)) == power_level ) &&  ((READ_REG8(RF_CHN)) == chn )\
			&& ((READ_REG8(RF_MODE)) == mode ))
	{
		rf_continue_mode_run();
	}
	rf_set_tx_rx_off();
}

void emitxprbs9(RF_ModeTypeDef rf_mode,RF_PowerTypeDef pwr,signed char rf_chn)
{
	unsigned short tx_cnt_temp;
	rf_emi_tx_brust_setup(rf_mode,pwr,rf_chn,0);
	while( ((READ_REG8(RUM_CMD)) == run ) &&  ((READ_REG8(RIGIG_CMD)) == cmd_now )\
			&& ((READ_REG8(RF_POWER_LEVEL)) == power_level ) &&  ((READ_REG8(RF_CHN)) == chn )\
			&& ((READ_REG8(RF_MODE)) == mode )&& (READ_REG8(RF_TX_MODE) == tx_mode))
	{
		if(tx_mode)
		{
			tx_cnt_temp = 1000;
			while(tx_cnt_temp--)
			{
				rf_emi_tx_brust_loop(rf_mode,0);
			}
			break;
		}
		else
		{
			rf_emi_tx_brust_loop(rf_mode,0);
		}
	}
	rf_set_tx_rx_off();
}

void emitx0f(RF_ModeTypeDef rf_mode,RF_PowerTypeDef pwr,signed char rf_chn)
{
	unsigned short tx_cnt_temp;
	rf_emi_tx_brust_setup(rf_mode,pwr,rf_chn,1);
	while( ((READ_REG8(RUM_CMD)) == run ) &&  ((READ_REG8(RIGIG_CMD)) == cmd_now )\
			&& ((READ_REG8(RF_POWER_LEVEL)) == power_level ) &&  ((READ_REG8(RF_CHN)) == chn )\
			&& ((READ_REG8(RF_MODE)) == mode )&& (READ_REG8(RF_TX_MODE) == tx_mode))
	{
		if(tx_mode)
		{
			tx_cnt_temp = 1000;
			while(tx_cnt_temp--)
			{
				rf_emi_tx_brust_loop(rf_mode,1);
			}
			break;
		}
		else
		{
			rf_emi_tx_brust_loop(rf_mode,1);
		}
	}
	rf_set_tx_rx_off();
}

void emitx55(RF_ModeTypeDef rf_mode,RF_PowerTypeDef pwr,signed char rf_chn)
{
	unsigned short tx_cnt_temp;
	rf_emi_tx_brust_setup(rf_mode,pwr,rf_chn,2);
	while( ((READ_REG8(RUM_CMD)) == run ) &&  ((READ_REG8(RIGIG_CMD)) == cmd_now )\
			&& ((READ_REG8(RF_POWER_LEVEL)) == power_level ) &&  ((READ_REG8(RF_CHN)) == chn )\
			&& ((READ_REG8(RF_MODE)) == mode )&& (READ_REG8(RF_TX_MODE) == tx_mode))
	{
		if(tx_mode)
		{
			tx_cnt_temp = 1000;
			while(tx_cnt_temp--)
			{
				rf_emi_tx_brust_loop(rf_mode,2);
			}
			break;
		}
		else
		{
			rf_emi_tx_brust_loop(rf_mode,2);
		}
	}
	rf_set_tx_rx_off();
}


void emirx(RF_ModeTypeDef rf_mode, RF_PowerTypeDef pwr,signed char rf_chn)
{
	rf_emi_rx(rf_mode,rf_chn);
	while( ((READ_REG8(RUM_CMD)) == run ) &&  ((READ_REG8(RIGIG_CMD)) == cmd_now )\
			&& ((READ_REG8(RF_POWER_LEVEL)) == power_level ) &&  ((READ_REG8(RF_CHN)) == chn )\
			&& ((READ_REG8(RF_MODE)) == mode ))
	{
		rf_emi_rx_loop();
		if(rf_emi_get_rxpkt_cnt()!=READ_REG32(0x84000c))
		{
			WRITE_REG8(RF_RSSI,rf_emi_get_rssi_avg());
			WRITE_REG32(RF_REC_NUMB,rf_emi_get_rxpkt_cnt());
		}
	}
	rf_set_tx_rx_off();
}

struct  test_list_s  ate_list[] = {
	{0x01,emicarrieronly},
	{0x02,emi_con_prbs9},
	{0x03,emirx},
	{0x04,emitxprbs9},
	{0x05,emitx55},
	{0x06,emitx0f},
	//{0x07,emi_con_tx55},
	//{0x08,emi_con_tx0f},
};

void emi_service_loop(void)
{
	unsigned char i=0;

	while(1)
	{
	   run = READ_REG8(RUM_CMD);  // get the run state!
	   if(run!=0)
	   {
		   irq_disable();
		   power_level = READ_REG8(RF_POWER_LEVEL);
		   chn = READ_REG8(RF_CHN);
		   mode = READ_REG8(RF_MODE);
		   tx_mode = READ_REG8(RF_TX_MODE);
		   cmd_now = READ_REG8(RIGIG_CMD);  // get the command!

			for (i=0; i<sizeof (ate_list)/sizeof (struct test_list_s); i++)
			{
				if(cmd_now == ate_list[i].cmd_id)
				{
					if(mode==0)//ble 2M mode
					{
						rf_set_acc_code(0x29417671);
						rf_mode_init(RF_MODE_BLE_2M);
						if( (g_tp0 != 0xff ) && (g_tp1 != 0xff))
						{
							rf_set_tp(g_tp0, g_tp1);
						}
						ate_list[i].func(RF_MODE_BLE_2M, power_level, chn);
					}
					else if(mode==1)//ble 1M mode
					{
						rf_set_acc_code(0x29417671);
						rf_mode_init(RF_MODE_BLE_1M_NO_PN);
						if( (g_tp0 != 0xff ) && (g_tp1 != 0xff))
						{
							rf_set_tp(g_tp0, g_tp1);
						}
						ate_list[i].func(RF_MODE_BLE_1M_NO_PN,power_level,chn);
					}
					else if(mode==2)//zigbee mode
					{
						rf_mode_init(RF_MODE_ZIGBEE_250K);
						if( (g_tp0 != 0xff ) && (g_tp1 != 0xff))
						{
							rf_set_tp(g_tp0, g_tp1);
						}
						ate_list[i].func(RF_MODE_ZIGBEE_250K,power_level,chn);
					}
					break;
				}
			}
			run = 0;
			WRITE_REG8(RUM_CMD, run);
	   }
	}

}

void user_init()
{
#if DEVELOPER_TEST
	gpio_set_func(LED1 ,AS_GPIO);
	gpio_set_output_en(LED1, 1); 		//enable output
	gpio_set_input_en(LED1 ,0);			//disable input
	gpio_write(LED1, 1);              	//LED On
#endif

	read_flash_para();
	emi_init();
	rf_set_power_level_index (power_level);
	rf_set_tx_rx_off_auto();
}

void main_loop (void)
{
	emi_service_loop();
}

#endif
