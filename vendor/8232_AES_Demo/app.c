/********************************************************************************************************
 * @file     app.c 
 *
 * @brief    This is the source file for TLSR8258
 *
 * @author	 junwei.lu@telink-semi.com;
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

#define LED1     		        GPIO_PC1


#if(AES_MODE == RISC_MODE)

unsigned char sPlainText[16] 	= {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};
unsigned char sKey[16] 			= {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f};
unsigned char EncryptResult[16] = {};
unsigned char sCipherText[16] 	= {};
unsigned char DecryptResult[16] = {};


void user_init()
{
	delay_ms(2000);
	//1.init the LED pin,for indication
	gpio_set_func(LED1 ,AS_GPIO);
	gpio_set_output_en(LED1, 1); 		//enable output
	gpio_set_input_en(LED1 ,0);			//disable input
	gpio_write(LED1, 0);              	//LED On

	//step1: encrypt the plain text
	aes_encrypt(sKey, sPlainText, EncryptResult);
	//step2: save the result being encrypted to sCipherText[]
	for(unsigned char i=0;i<16;i++)
	{
		sCipherText[i] = EncryptResult[i];
	}
	//step3: decrypt the result being encrypted and save result to DecryptResult[]
	aes_decrypt(sKey, sCipherText, DecryptResult);
}

void main_loop (void)
{
	gpio_toggle(LED1);
	delay_ms(100);
}

#endif


