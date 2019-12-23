/********************************************************************************************************
 * @file     analog.c 
 *
 * @brief    This is the source file for TLSR8232
 *
 * @author	 public@telink-semi.com;
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


#include "analog.h"

#include "compiler.h"
#include "register.h"
#include "irq.h"

/**
 * @brief      This function serves to wait for analog register ready.
 * @param[in]  none.
 * @return     none.
 */
static inline void analog_wait(){
	while(reg_ana_ctrl & FLD_ANA_BUSY){}
}

/**
 * @brief      This function serves to analog register read.
 * @param[in]  addr - address need to be read.
 * @return     the result of read.
 */
_attribute_ram_code_ unsigned char analog_read(unsigned char addr){
	unsigned char r = irq_disable();

	reg_ana_addr = addr;
	reg_ana_ctrl = (FLD_ANA_START);
	analog_wait();
	unsigned char data = reg_ana_data;
	reg_ana_ctrl = 0;
	irq_restore(r);
	return data;
}

/**
 * @brief      This function serves to analog register write.
 * @param[in]  addr - address need to be write.
 * @param[in]  v - the value need to be write.
 * @return     none.
 */
_attribute_ram_code_ void analog_write(unsigned char addr, unsigned char v){
	unsigned char r = irq_disable();

	reg_ana_addr = addr;
	reg_ana_data = v;
	reg_ana_ctrl = (FLD_ANA_START | FLD_ANA_RW);
	analog_wait();
	reg_ana_ctrl = 0;
	irq_restore(r);
}

