/********************************************************************************************************
 * @file     main.c
 *
 * @brief    This is the RF file for TLSR8258
 *
 * @author	 liang.zhong@telink-semi.com;
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
 *			 2.add ant mode and rffe api(Jan. 2 2019)
 * @version  A001
 *
 *******************************************************************************************************/
#include "app_config.h"

extern void user_init();
extern void main_loop (void);



/**
 * @brief		This function serves to handle the interrupt of MCU
 * @param[in] 	none
 * @return 		none
 */
_attribute_ram_code_ void irq_handler(void)
{


}

/**
 * @brief		This is main function
 * @param[in]	none
 * @return      none
 */
void main (void) {

	system_init();

	clock_init(SYS_CLK);

	#if(RF_MODE==RF_BLE_2M)
	rf_mode_init(RF_MODE_BLE_2M);
	#elif(RF_MODE==RF_BLE_1M)
	rf_mode_init(RF_MODE_BLE_1M);
	#elif(RF_MODE==RF_BLE_2M_NO_PN)
	rf_mode_init(RF_MODE_BLE_2M_NO_PN);
	#elif(RF_MODE==RF_BLE_1M_NO_PN)
	rf_mode_init(RF_MODE_BLE_1M_NO_PN);
	#elif(RF_MODE==RF_ZIGBEE_250K)
	rf_mode_init(RF_MODE_ZIGBEE_250K);
	#elif(RF_MODE==RF_PRIVATE_1M)
	rf_mode_init(RF_MODE_PRIVATE_1M);
	#elif(RF_MODE==RF_PRIVATE_2M)
	rf_mode_init(RF_MODE_PRIVATE_2M);
	#endif

	gpio_init();

	user_init();

	irq_enable();

	while (1) {
		main_loop ();
	}
}
