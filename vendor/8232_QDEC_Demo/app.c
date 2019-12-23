/********************************************************************************************************
 * @file     app.c 
 *
 * @brief    This is the source file for TLSR8258
 *
 * @author	 yang,ye@telink-semi.com;
 * @date     December 5, 2018
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
 *         
 *******************************************************************************************************/
#include "app_config.h"

#define LED1     				GPIO_PD0

#define QDEC_CHA 	GPIO_PC1
#define QDEC_CHB 	GPIO_PC2
unsigned char qdec_count = 0;

void user_init()
{
	gpio_set_func(QDEC_CHA, AS_GPIO);
	gpio_set_output_en(QDEC_CHA,0);
	gpio_set_input_en(QDEC_CHA,1);

	gpio_set_func(QDEC_CHB, AS_GPIO);
	gpio_set_output_en(QDEC_CHB,0);
	gpio_set_input_en(QDEC_CHB,1);
	qdec_clk_en ();
	//qdec_32krc_clock_init();
	qdec_set_mode(DOUBLE_ACCURACY_MODE);
	qdec_set_pin(PC1A,PC2A);
	qdec_set_debouncing(1);   //set debouncing

}


/////////////////////////////////////////////////////////////////////
// main loop flow
/////////////////////////////////////////////////////////////////////
void main_loop (void)
{

	qdec_count =  qdec_get_count_value();;
	delay_ms(1000);

}



