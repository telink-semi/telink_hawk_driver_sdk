/********************************************************************************************************
 * @file     aes.h 
 *
 * @brief    This is the header file for TLSR8232
 *
 * @author	 jian.zhang@telink-semi.com;junwei.lu@telink-semi.com;
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

#ifndef AES_H_
#define AES_H_

/**
 * @brief       This function servers to perform aes_128 encryption for 16-Byte input data
 *              with specific 16-Byte key
 * @param[in]   Key the pointer to the 16-Byte Key
 * @param[in]   Data the pointer to the 16-Byte plain text
 * @param[out]  Result the pointer to the encryption result cipher text
 * @return      0: success;
 *              not 0: failure
 */
extern int aes_encrypt(unsigned char *Key, unsigned char *Data, unsigned char *Result);

/**
 * @brief       This function servers to perform aes_128 decryption for 16-Byte input data
 *              with specific 16-Byte key
 * @param[in]   Key the pointer to the 16-Byte Key
 * @param[in]   Data the pointer to the 16-Byte cipher text
 * @param[out]  Result the pointer to the decryption result plain text
 * @return      0: success;
 *              not 0: failure
 */
extern int aes_decrypt(unsigned char *Key, unsigned char *Data, unsigned char *Result);

#endif /* AES_H_ */

/** \defgroup GP2  AES Examples
 *
 *  @{
 */

/*! \page aes Table of Contents
    - [API-AES-CASE1:AES RSIC MODE](#AES_RISC_MODE)


<h1 id=AES_RISC_MODE> API-AES-CASE1:AES RISC MODE </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | None ||| Interrupt handler function [**Mandatory**] | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | aes_encrypt() | aes_encrypt(sKey, sPlainText, EncryptResult) | generate encryption by key and plaintext | ^ |
| ^ | ^ | aes_decrypt() | aes_decrypt(sKey, EncryptResult, DecryptResult) | generate decryption by key and encryption | ^ |
| ^ | main_loop() | none ||| ^ |

\n
Variables above are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
unsigned char sPlainText[16] 	= {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};
unsigned char sKey[16] 			= {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f};
unsigned char EncryptResult[16] = {};
unsigned char DecryptResult[16] = {};
~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1> History Record </h1>

| Date | Description | Author |
| :--- | :---------- | :----- |
| 2019-1-10 | initial release | ZJY/LJW |


*/

 /** @}*/ //end of GP2


