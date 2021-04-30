/********************************************************************************************************
 * @file	phytest.h
 *
 * @brief	This is the header file for TLSR8232
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
#ifndef PHYTEST_H_
#define PHYTEST_H_

#ifndef		CUST_CAP_INFO_ADDR_64K
#define		CUST_CAP_INFO_ADDR_64K				0xE000
#endif

#ifndef		CUST_TP_INFO_ADDR_64K
#define		CUST_TP_INFO_ADDR_64K				0xE040
#endif

#ifndef		CUST_CAP_INFO_ADDR_128K
#define		CUST_CAP_INFO_ADDR_128K				0x1E000
#endif

#ifndef		CUST_TP_INFO_ADDR_128K
#define		CUST_TP_INFO_ADDR_128K				0x1E040
#endif

#ifndef		CUST_CAP_INFO_ADDR_512K
#define		CUST_CAP_INFO_ADDR_512K				0x77000
#endif

#ifndef		CUST_TP_INFO_ADDR_512K
#define		CUST_TP_INFO_ADDR_512K				0x77040
#endif

#define RF_MODE					RF_MODE_BLE_1M
#define ACCESS_CODE				0x29417671
#define RF_TX_POWER				RF_POWER_7P9dBm

#if !BQB_SUPPORT_USR_CONFIG
//UART 	TX: UART_TX_PA3, UART_TX_PB4, UART_TX_PC4
//		RX:	UART_RX_PA4, UART_RX_PB5, UART_RX_PC5
#define BQB_UART_TX					UART_TX_PA3
#define BQB_UART_RX					UART_RX_PA4
#define CUST_CAP_INFO_ADDR		CUST_CAP_INFO_ADDR_512K
#define CUST_TP_INFO_ADDR		CUST_TP_INFO_ADDR_512K
#endif

#define SPECIAL_PROCESS 1

/**
 *   type of the flash
 */
typedef enum {
	FLASH_128K = 0,
	FLASH_512K = 1,
	FLASH_64K = 2
}Flash_type;
/**
 *   choice of uart port
 */
typedef enum {
	UART_TX_PA3_RX_PA4 = 0,
	UART_TX_PA3_RX_PB5 = 1,
	UART_TX_PA3_RX_PC5 = 2,

	UART_TX_PB4_RX_PA4 = 3,
	UART_TX_PB4_RX_PB5 = 4,
	UART_TX_PB4_RX_PC5 = 5,

	UART_TX_PC4_RX_PA4 = 6,
	UART_TX_PC4_RX_PB5 = 7,
	UART_TX_PC4_RX_PC5 = 8
}Uart_port_choice;

/**
 *  flag table for user's setting
 */
typedef union{
	unsigned char cfg_value;
	struct{
		unsigned char uart :5;
		unsigned char cap  :1;
		unsigned char f_size:2;
	};
}User_Config_u;



extern void phy_test_init(void);

extern void phytest(void);

#endif /* PHYTEST_H_ */
