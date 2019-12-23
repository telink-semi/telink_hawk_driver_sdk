/********************************************************************************************************
 * @file     clock.c 
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
 *         
 *******************************************************************************************************/

#include "register.h"
#include "clock.h"
#include "irq.h"
#include "analog.h"
#include "timer.h"


/**
 * @brief       This function to select the system clock source.
 * @param[in]   SYS_CLK - the clock source of the system clock.
 * @return      none
 */
void clock_init(SYS_CLK_TYPEDEF SYS_CLK)
{
	WRITE_REG8(0x70,READ_REG8(0x70)&0xfe);
	WRITE_REG8(0x66,SYS_CLK);
}

/**
 * @brief   This function serves to set 32k clock source.
 * @param[in]   variable of 32k type.
 * @return  none.
 */
_attribute_ram_code_ void clock_32k_init (CLK_32K_TypeDef src)
{
	unsigned char sel_32k   = analog_read(0x2d)&0x7f;
	unsigned char power_32k = analog_read(0x05)&0xfc;
	analog_write(0x2d, sel_32k|(src<<7));
	if(src)
	{
		analog_write(0x05, power_32k|0x0);//power on 32K XTAL and 32K RC at the same time
#if 1
		//2.set pc3 as pwm output
		unsigned char sys_clk = READ_REG8(0x66);
		WRITE_REG8(0x66,0x43);
		unsigned char reg_58e = READ_REG8(0x58e);
		WRITE_REG8(0x58e,reg_58e&0x7f);
		unsigned short reg_798 = READ_REG16(0x798);
		WRITE_REG16(0x798,0x01);
		unsigned short reg_79a = READ_REG16(0x79a);
		WRITE_REG16(0x79a,0x02);
		unsigned char reg_780 = READ_REG8(0x780);
		WRITE_REG8(0x780,0x02);
		unsigned char reg_781 = READ_REG8(0x781);
		WRITE_REG8(0x781,0xf3);

		//3.wait for PWM wake up Xtal
		delay_ms(5);

		//4.Xtal 32k output
		analog_write(0x07,0x01); //<1:0>current select

		//5.Recover PC3 as Xtal pin
		WRITE_REG8(0x66,sys_clk);
		WRITE_REG8(0x58e,reg_58e);
		WRITE_REG16(0x798,reg_798);
		WRITE_REG16(0x79a,reg_79a);
		WRITE_REG8(0x780,reg_780);
		WRITE_REG8(0x781,reg_781);
#endif
	}
	else
	{
		analog_write(0x05, power_32k|0x2);//power on 32K RC
	}
}

/**
 * @brief     This function performs to select 24M as the system clock source.
 * @param[in] none.
 * @return    none.
 */
_attribute_ram_code_ void rc_24m_cal (void)
{
	sub_wr_ana(0x83, 3, 6, 4);	//wait len
	sub_wr_ana(0x83, 0, 1, 1);	//sel calbr 24m
	sub_wr_ana(0x02, 1, 4, 4);	//manual off
	sub_wr_ana(0x83, 1, 0, 0);	//calbr en on
	while((analog_read(0x84) & 0x01) == 0);	//wait done
	unsigned char cap = analog_read(0x85);	//read 24m cap result
	analog_write(0x30, cap);		//write 24m cap into manual register
	sub_wr_ana(0x83, 0, 0, 0);	//calbr en off
	sub_wr_ana(0x02, 0, 4, 4);	//manual on

}

/**
 * @brief     This function performs to select 32K as the system clock source.
 * @param[in] none.
 * @return    none.
 */
_attribute_ram_code_ void rc_32k_cal (void)
{
	sub_wr_ana(0x83, 3, 6, 4);	//wait len
	sub_wr_ana(0x83, 1, 1, 1);	//sel calbr 32k
	sub_wr_ana(0x02, 1, 2, 2);	//manual off
	sub_wr_ana(0x83, 1, 0, 0);	//calbr en on
	while((analog_read(0x84) & 0x01) == 0);	//wait done
	unsigned char cap = analog_read(0x85);	//read 32k cap result
	analog_write(0x2f, cap);		//write 32k cap into manual register
	sub_wr_ana(0x83, 0, 0, 0);	//calbr en off
	sub_wr_ana(0x02, 0, 2, 2);	//manual on
}









