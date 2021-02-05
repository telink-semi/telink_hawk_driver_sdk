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


#define FLASH_ADDR				0x010000
#define FLASH_BUFF_LEN			256

#define		MID1140C8		0        	// GD25D10C & GD25D10B
#define		MID134051		1  			// MD25D40DGIG
#define		MID12325E		2			// ZB25WD20A
#define		MID13325E		3			// ZB25WD40B

#define		FLASH_TYPE		MID13325E

volatile unsigned char Flash_Read_Buff1[FLASH_BUFF_LEN]={0};
volatile unsigned char Flash_Read_Buff2[FLASH_BUFF_LEN]={0};
volatile unsigned char Flash_Read_Buff3[FLASH_BUFF_LEN]={0};
volatile unsigned char Flash_Read_Buff4[FLASH_BUFF_LEN]={0};

volatile unsigned char Flash_Read_Buff5[FLASH_BUFF_LEN]={0};
volatile unsigned char Flash_Read_Buff6[FLASH_BUFF_LEN]={0};

volatile unsigned char Flash_Read_Buff7[FLASH_BUFF_LEN]={0};

volatile unsigned char OTP_Read_Buff1[FLASH_BUFF_LEN]={0};
volatile unsigned char OTP_Read_Buff2[FLASH_BUFF_LEN]={0};
volatile unsigned char OTP_Read_Buff3[FLASH_BUFF_LEN]={0};

volatile unsigned char Flash_Write_Buff[FLASH_BUFF_LEN]=
{		0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
		0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,
		0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,
		0xb0,0xb1,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,
		0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,
		0xd0,0xd1,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,
		0xe0,0xe1,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,
		0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,


		0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
		0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,
		0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,
		0xb0,0xb1,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,
		0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,
		0xd0,0xd1,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,
		0xe0,0xe1,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,
		0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,

		0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
		0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,
		0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,
		0xb0,0xb1,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,
		0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,
		0xd0,0xd1,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,
		0xe0,0xe1,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,
		0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,

		0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
		0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,
		0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,
		0xb0,0xb1,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,
		0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,
		0xd0,0xd1,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,
		0xe0,0xe1,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,
		0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,
};

volatile unsigned int check_umid;
volatile unsigned char status;
unsigned int mid_buf=0;
unsigned char uid_buf[16]={0};

void user_init()
{
	//1.init the LED pin,for indication
	gpio_set_func(LED1 ,AS_GPIO);
	gpio_set_output_en(LED1, 1); //enable output
	gpio_set_input_en(LED1 ,0);//disable input
	gpio_write(LED1, 0);              //LED On

	flash_write_page(FLASH_ADDR,FLASH_BUFF_LEN,(unsigned char *)Flash_Write_Buff);
	flash_write_page(FLASH_ADDR+0x0f00,FLASH_BUFF_LEN,(unsigned char *)Flash_Write_Buff);

	flash_read_page(FLASH_ADDR,FLASH_BUFF_LEN,(unsigned char *)Flash_Read_Buff1);
	flash_read_page(FLASH_ADDR+0x0f00,FLASH_BUFF_LEN,(unsigned char *)Flash_Read_Buff2);


	flash_erase_sector(FLASH_ADDR);

	flash_read_page(FLASH_ADDR,FLASH_BUFF_LEN,(unsigned char *)Flash_Read_Buff3);
	flash_read_page(FLASH_ADDR+0x0f00,FLASH_BUFF_LEN,(unsigned char *)Flash_Read_Buff4);

#if(FLASH_TYPE == MID12325E)
	flash_lock_mid12325e(FLASH_LOCK_LOW_128K_MID12325E);
	status = flash_read_status_mid12325e();

#elif(FLASH_TYPE == MID134051)
	flash_lock_mid134051(FLASH_LOCK_LOW_256K_MID134051);
	status = flash_read_status_mid134051();
#elif(FLASH_TYPE == MID1140C8)
	flash_lock_mid1140c8(FLASH_LOCK_LOW_120K_MID1140C8);
	status = flash_read_status_mid1140c8();

#elif(FLASH_TYPE == MID13325E)
	flash_lock_mid13325e(FLASH_LOCK_LOW_256K_MID13325E);
	status = flash_read_status_mid13325e();
#endif

#if(FLASH_TYPE == MID13325E) || (FLASH_TYPE == MID134051)
	flash_write_page(FLASH_ADDR,FLASH_BUFF_LEN,(unsigned char *)Flash_Write_Buff);
	flash_write_page(FLASH_ADDR+0x30000,FLASH_BUFF_LEN,(unsigned char *)Flash_Write_Buff);

	flash_read_page(FLASH_ADDR,FLASH_BUFF_LEN,(unsigned char *)Flash_Read_Buff5);
	flash_read_page(FLASH_ADDR+0x30000,FLASH_BUFF_LEN,(unsigned char *)Flash_Read_Buff6);

#elif(FLASH_TYPE == MID12325E)
	flash_write_page(FLASH_ADDR,FLASH_BUFF_LEN,(unsigned char *)Flash_Write_Buff);
	flash_write_page(FLASH_ADDR+0x10000,FLASH_BUFF_LEN,(unsigned char *)Flash_Write_Buff);

	flash_read_page(FLASH_ADDR,FLASH_BUFF_LEN,(unsigned char *)Flash_Read_Buff5);
	flash_read_page(FLASH_ADDR+0x10000,FLASH_BUFF_LEN,(unsigned char *)Flash_Read_Buff6);
#elif(FLASH_TYPE == MID1140C8)
	flash_write_page(FLASH_ADDR,FLASH_BUFF_LEN,(unsigned char *)Flash_Write_Buff);
	flash_write_page(FLASH_ADDR+0xe000,FLASH_BUFF_LEN,(unsigned char *)Flash_Write_Buff);

	flash_read_page(FLASH_ADDR,FLASH_BUFF_LEN,(unsigned char *)Flash_Read_Buff5);
	flash_read_page(FLASH_ADDR+0xe000,FLASH_BUFF_LEN,(unsigned char *)Flash_Read_Buff6);
#endif


#if(FLASH_TYPE == MID12325E)
	flash_unlock_mid12325e();
#elif(FLASH_TYPE == MID134051)
	flash_unlock_mid134051();
#elif(FLASH_TYPE == MID1140C8)
	flash_unlock_mid1140c8();
#elif(FLASH_TYPE == MID13325E)
	flash_unlock_mid13325e();
#endif

	flash_write_page(FLASH_ADDR,FLASH_BUFF_LEN,(unsigned char *)Flash_Write_Buff);
	flash_read_page(FLASH_ADDR,FLASH_BUFF_LEN,(unsigned char *)Flash_Read_Buff7);

	check_umid = flash_read_mid_uid_with_check((unsigned int *)&mid_buf, uid_buf);
}


void main_loop (void)
{
	gpio_toggle(LED1);
	delay_ms(100);
}



