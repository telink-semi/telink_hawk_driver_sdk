/********************************************************************************************************
 * @file	phytest.c
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
#include "../app_config.h"

#if(TEST_DEMO == BQB_DEMO)
#include "drivers.h"
#include "common.h"
#include "phytest.h"

#define		CMD_RX							1
#define		CMD_TX							2
#define		CMD_END							3
#define		CMD_SETUP						0
#define		RF_BLE_PACKET_CRC_OK(p)			((p[p[0]+p[1]*256+3] & 0x51) == 0x40)
#define 	reg_rf_irq_status				REG_ADDR16(0xf20)


#define RF_RX_BUFFER_LENGTH 512
#define RF_TX_PACET_LENGTH 	264

volatile u32 t0;
u8 tx_start_flag = 0;
u8	buffer_phytest[RF_RX_BUFFER_LENGTH] __attribute__ ((aligned (4)));


u8 pkt_phytest [RF_TX_PACET_LENGTH]  __attribute__ ((aligned (4)))= {
		39, 0, 0, 0,
		0, 37,
		0, 1, 2, 3, 4, 5, 6, 7
};

static union pkt_length_u
{
	u8 len;
	struct len_t
	{
		u8 low:6;
		u8 upper:2;
	}l;
}pkt_length;

#if SPECIAL_PROCESS
/*
 * Special process: Change the process of sudden energy increase to slowly increase and then slowly decrease
 * Add by Pengcheng, suggest by Wenfeng, to pass In-band Emision
 */
void special_process_tx(u32* tick_tx)
{
	//set power to 7.9dBm
	analog_write (0xa2, 0x04);
	analog_write(0xa3,analog_read(0xa3)&0xef); //ana_0xa3<4> PA ramp enable
	analog_write(0xa3,analog_read(0xa3)|0x10);
	analog_write (0x04, 0xa2);
	analog_write (0xa7, 0xd9);
	analog_write (0x8d, 0x63);

	analog_write(0xa5,0x04);   // for carrier  mode
	WRITE_REG8 (0x8004e8, 0x04); // for  carrier mode//tx_cyc1

//	analog_write (0xa7, 0xe9);
	analog_write (0xa7, 0xd9);
	analog_write (0xa7, 0xb9);
	analog_write (0xa7, 0x79);
//	analog_write (0xa7, 0xf1);
	*tick_tx =  get_sys_tick();
	rf_tx_pkt(pkt_phytest);
	while(!rf_is_tx_finish());
	rf_clr_tx_finish();
	analog_write (0xa7, 0x79);
	analog_write (0xa7, 0xb9);
	analog_write (0xa7, 0xd9);
	analog_write (0xa7, 0xe9);

	//set power to -19.5dBm
	analog_write (0xa2, 0x00);
	analog_write(0xa3,analog_read(0xa3)&0xef); //ana_0xa3<4> PA ramp enable
	analog_write(0xa3,analog_read(0xa3)|0x10);
	analog_write (0x04, 0x92);
	analog_write (0xa7, 0xe9);
	analog_write (0x8d, 0x63);

	analog_write(0xa5,0x00);   // for carrier  mode
	WRITE_REG8 (0x8004e8, 0x00); // for  carrier mode//tx_cyc1
}
#endif

/**
 * @brief   This function serves to obtain the pkt interval in different payload lengths and different modes
 * @param   payload_len: payload length
 * @param   mode: tx mode
 * 				1:BLE1M
 * 				2:BLE2M
 * 				3:BLE125K
 * 				4:BLE500K
 * @return  the pkt interval
 */
u32 get_pkt_interval(u8 payload_len, u8 mode)
{
	u32 total_len,byte_time=8;
	u8 preamble_len;

	preamble_len = READ_REG8(0x402) & 0x1f ;
	total_len = preamble_len + 4 + 2 + payload_len +3; // preamble + access_code + header + payload + crc

	if(mode==1)//1m
	{
		byte_time = 8;
	}
	else if(mode==2)//2m
	{
		byte_time = 4;
	}
	else if(mode==3)//s=8
	{
		byte_time = 64;
	}
	else if(mode==4) // s=2
	{
		byte_time = 16;
	}
	return (((byte_time * total_len + 249  + 624)/625)*625);

}


u8	phyTest_Channel (u8 chn)
{
	if (chn == 0)
	{
		return 37;
	}
	else if (chn < 12)
	{
		return chn - 1;
	}
	else if (chn == 12)
	{
		return 38;
	}
	else if (chn < 39)
	{
		return chn - 2;
	}
	else
	{
		return 39;
	}
}
void phyTest_PRBS9 (u8 *p, int n)
{
	//PRBS9: (x >> 1) | (((x<<4) ^ (x<<8)) & 0x100)
	u16 x = 0x1ff;
	int i;
	int j;
	for ( i=0; i<n; i++)
	{
		u8 d = 0;
		for (j=0; j<8; j++)
		{
			if (x & 1)
			{
				d |= BIT(j);
			}
			x = (x >> 1) | (((x<<4) ^ (x<<8)) & 0x100);
		}
		*p++ = d;
	}
}
u8 Rf_GetBleFreChannel(u8 chn)
{
	WRITE_REG8 (0x80040d, chn);

	if (chn < 11)
		chn += 2;
	else if (chn < 37)
		chn += 3;
	else if (chn == 37)
		chn = 1;
	else if (chn == 38)
		chn = 13;
	else
		chn = 40;

	chn = chn << 1;
	return chn;
}
static u8 index=0;
static inline u8 Get_Uart_Byte(void)
{

	u8 ret;

	ret = REG_ADDR8 (0x90+index);
	index++;
	if(index>=4)
		index = 0;

	return ret;

}

u16 uart_phyTestGet(u16* cmd)
{
	static u32 tick_rx = 0;
	u16 l, h;

	if (!tick_rx && (REG_ADDR8(0x9c)&15) == 1)
	{
		tick_rx = get_sys_tick ();
	}
	else if ((REG_ADDR8(0x9c)&15) == 2)
	{
		h = Get_Uart_Byte()&0xff;
		l = Get_Uart_Byte()&0xff;

		*cmd = (l | (h<<8));

		tick_rx = 0;
		return 1;
	}
	else if (tick_rx && timeout_us (tick_rx, 5000))
	{
		tick_rx = 0;
		Get_Uart_Byte();
	}
	else if((REG_ADDR8(0x9c)&15)>2)
	{
		u8 i;

		u8 l = REG_ADDR8(0x9c)&0x0f;
		for(i=0; i<l; i++)
			Get_Uart_Byte();

	}
	return 0;
}
static inline void uart_phyTestSend (u16 st)
{
	static u8 index = 0;
	u8 l;
	l = st>>8;

	REG_ADDR16(0x90+index) = (st<<8 | l);
	index += 2;

	index = index%4;
}
/*
 * 80 94 PRBS9
 * 80 95 0x0f
 * 80 96 0x55
 */
/*
 * 0x81 0xd3, power table index 9
 */
void phytest (void)
{
	static u16 cmd;
	static u8 chn, ctrl, para, type;
	static u16 rsp = 0,st = 0, pkt_interval = 625;
	static u16 pkts = 0;

	u32 tick_tx=0; //initial
	do
	{
		if((st==1) || (st==2))
		{
	    	if(timeout_us(t0, 1000*300))
			{
	    		t0 = get_sys_tick();
#if DEVELOPER_TEST
	    		gpio_toggle(LED1);
#endif
			}
		}
		if (uart_phyTestGet(&cmd))
		{
			tick_tx = get_sys_tick();
			u8 ct = cmd >> 14;
			switch(ct)
			{
			case CMD_SETUP:				
				pkts = 0;
				rf_set_tx_rx_off();
				ctrl = (cmd >> 8)&0x3f;
				para = (cmd >> 2)&0x3f;
				rsp = 0;//add by pengcheng 07/08
				switch(ctrl)
				{
				case 0:
					if(para==0)
					{
						pkt_length.l.upper =0;
						rsp = 0;
					}
					else
					{
						rsp = BIT(0);//Error
					}
					break;
				case 1:
					if((para>=0) && (para<=3))
					{
						pkt_length.l.upper = para &0x03;
						rsp = 0;//add by pengcheng 07/08
					}
					else
					{
						rsp = BIT(0); //Error
					}
					break;
				case 2:
					if((para >= 1) && (para <= 4))
					{
						rsp = 0;
					}
					else
					{
						rsp = BIT(0);
					}
					break;
				case 3:
					rsp = 0;
					break;
				case 4:					
					rsp |= BIT(1) | BIT(2);
					rsp = (rsp <<1);
					break;
				case 5:
					if(para==0)
					{
						rsp = (251<<1)&0x7ffe;
					}
					else if(para==1)
					{
						rsp = (17040 << 1)&0x7ffe;
					}
					else if(para==2)
					{
						rsp = (251<<1)&0x7ffe;
					}
					else if(para==3)
					{
						rsp = (17040 << 1)&0x7ffe;
					}
					else
					{
						rsp = BIT(0);//status EVENT Error
					}
					break;
				default:
					rsp = BIT(0);//add by pengcheng 07/08 ***most***
					break;
				}
				uart_phyTestSend (rsp);
				break;
			case CMD_RX:
				pkts = 0;
				chn = (cmd >> 8) & 0x3f;	//frequency
				chn = Rf_GetBleFreChannel(phyTest_Channel(chn));
				rf_set_trx_state(RF_MODE_RX,chn);
				rsp = 0;
				uart_phyTestSend (rsp);
				break;
			case CMD_TX:
				pkts = 0;
				chn = (cmd >> 8) & 0x3f;	//frequency
				pkt_length.l.low = (cmd >> 2) & 0x3f;
				type = cmd & 0x03;
				rsp = 0;
				memset(pkt_phytest, 0, 261);
				if(pkt_length.len <= 253)
				{
					pkt_phytest[0] = pkt_length.len + 2;
					pkt_phytest[1] = 0;
				}
				else
				{
					pkt_phytest[0] = pkt_length.len + 2 - 256;
					pkt_phytest[1] = 1;
				}
				pkt_phytest[4] = type;
				pkt_phytest[5] = pkt_length.len;
				
				switch(type)
				{
				case 0:
					phyTest_PRBS9 (pkt_phytest + 6, pkt_length.len);
					break;
				case 1:
					memset(pkt_phytest + 6, 0x0f, pkt_length.len);
					break;
				case 2:
					memset(pkt_phytest + 6, 0x55, pkt_length.len);
					break;
				default:
					rsp = BIT(0);
					break;
				}
				pkt_interval = get_pkt_interval(pkt_length.len, 1);
				chn = Rf_GetBleFreChannel(phyTest_Channel(chn));
				tx_start_flag = 1;
#if !SPECIAL_PROCESS
				rf_set_power_level_index(RF_TX_POWER);
#endif
				rf_set_trx_state(RF_MODE_TX ,chn);
				uart_phyTestSend (rsp);
				break;
			case CMD_END:
				rf_set_tx_rx_off();
				uart_phyTestSend (BIT(15) | (pkts & 0x7fff));
#if DEVELOPER_TEST
				gpio_write(LED1, 0);
#endif
				break;
			default:
				WRITE_REG8(0x808004,0x99);//ERR
				break;
			}			
			st = ct;
		}


		if (st == CMD_TX)
		{
			if (timeout_us(tick_tx, pkt_interval) || (tx_start_flag == 1))
			{
				if(tx_start_flag)
					tx_start_flag = 0;
#if SPECIAL_PROCESS
				special_process_tx(&tick_tx);
#else
				tick_tx =  get_sys_tick();
				rf_tx_pkt(pkt_phytest);
				while(!rf_is_tx_finish());
				rf_clr_tx_finish();
#endif
			}
		}
		else if (st == CMD_RX)
		{
			if (READ_REG8(0xf20) & BIT(0))
			{
				BM_SET(REG_ADDR8(0xf20) , BIT(0));
				if	( RF_BLE_PACKET_LENGTH_OK(buffer_phytest) && RF_BLE_PACKET_CRC_OK(buffer_phytest) )
				{
					pkts++;
				}
			}
		}
	} while (st == 1 || st == 2);
}


u8 uart_phyTest_init(u16 uartCLKdiv, u8 bwpc){

	if(bwpc<3)
		return 0;
	WRITE_REG16(0x800094,(uartCLKdiv|0x8000));//set uart clk divider and enable clock divider
	WRITE_REG8(0x800096,bwpc);			//register mode
	WRITE_REG8(0x80009a,(bwpc+1)*10);
	WRITE_REG8(0x80009b,1);//For save

	WRITE_REG8(0x800097,0x00);//No clts and rts
	return 1;
}

void  phy_test_init(void)
{
	phyTest_PRBS9 (pkt_phytest + 6, 37);
	rf_set_rx_buff(buffer_phytest,RF_RX_BUFFER_LENGTH,0);
	REG_ADDR8(0x401) = 0;				//disable PN
	rf_set_acc_code(ACCESS_CODE);
}


#endif
