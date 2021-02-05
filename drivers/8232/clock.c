/********************************************************************************************************
 * @file	clock.c
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
#include "register.h"
#include "clock.h"
#include "irq.h"
#include "analog.h"
#include "timer.h"


CLK_32K_TypeDef g_clk_32k_src;

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
 * @brief       This function is to accelerate the oscillation process by using PWM
 * @param[in]   none
 * @return      none
 */
void pwm_kick_32k_pad(void)
{
	unsigned char reg_66 = READ_REG8(0x66);
	WRITE_REG8(0x66,0x43);

	//1.set pb6, pb7 as pwm output
	unsigned char reg_58e = READ_REG8(0x58e);
	WRITE_REG8(0x58e,reg_58e&0x3f);
	unsigned char reg_5ab = READ_REG8(0x5ab);
	WRITE_REG8(0x5ab,reg_5ab&0x0f);
	WRITE_REG8(0x781,0xf3);//pwm clk div

	unsigned short reg_794 = READ_REG16(0x794);
	WRITE_REG16(0x794,0x01);//pwm0's high time or low time
	unsigned short reg_796 = READ_REG16(0x796);
	WRITE_REG16(0x796,0x02);//pwm0's cycle time
	WRITE_REG8(0x780,0x01);//enable pwm0
	delay_ms(25);
	unsigned short reg_798 = READ_REG16(0x798);
	WRITE_REG16(0x798,0x01);//pwm1's high time or low time
	unsigned short reg_79a = READ_REG16(0x79a);
	WRITE_REG16(0x79a,0x02);//pwm1's cycle time
	WRITE_REG8(0x780,0x03);//enable pwm1

	//2.wait for pwm wake up xtal
	delay_ms(25);
	//3.recover pb6, pb7 as xtal pin
	WRITE_REG8(0x780,0x02);
	WRITE_REG16(0x794,reg_794);
	WRITE_REG16(0x796,reg_796);
	WRITE_REG8(0x780,0x00);
	WRITE_REG16(0x798,reg_798);
	WRITE_REG16(0x79a,reg_79a);

	WRITE_REG8(0x781,0x00);
	WRITE_REG8(0x66,reg_66);
	WRITE_REG8(0x58a,READ_REG8(0x58a)|0xc0);
	WRITE_REG8(0x58e,reg_58e|0xc0);

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
		pwm_kick_32k_pad();//PWM kick external 32k pad
		g_clk_32k_src = CLK_32K_XTAL;
	}
	else
	{
		analog_write(0x05, power_32k|0x2);//power on 32K RC
		g_clk_32k_src = CLK_32K_RC;
	}
}

/**
 * @brief     This function performs to select 24M as the system clock source.
 * @param[in] none.
 * @return    none.
 */
_attribute_ram_code_ void rc_24m_cal (void)
{
	unsigned char temp = 0;

	/* Reset to default value */
	analog_write(0x83,0x34);

	/* cap from analog register */
	temp = analog_read(0x02);
	temp |= (1<<4);
	analog_write(0x02,temp);

	/*Disable 24M RC calibration.*/
	temp = analog_read(0x83);
	temp &= ~(1<<0);
	temp &= ~(1<<1);
	analog_write(0x83,temp);

	for(volatile int i=0; i<100; i++);

	/* Enable 24M RC calibration. */
	temp = analog_read(0x83);
	temp |= (1<<0);
	analog_write(0x83,temp);

	/* Wait Calibration completely. */
	for(volatile int i=0; i<10000; i++)
	{
	   if((analog_read(0x84) & 0x01))
	   {
			unsigned char CalValue = 0;
			CalValue = analog_read(0x85);
			analog_write(0x30,CalValue);

			break;
	   }
	}

	/* Disable 24M RC calibration. */
	temp = analog_read(0x83);
	temp &= ~(1<<0);
	analog_write(0x83,temp);

	/* cap from pm_top */
	temp = analog_read(0x02);
	temp &= ~(1<<4);
	analog_write(0x02,temp);
}

/**
 * @brief     This function performs to select 32K as the system clock source.
 * @param[in] none.
 * @return    none.
 */
_attribute_ram_code_ void rc_32k_cal (void)
{
	unsigned char temp = 0;

	/* Reset to default value */
	analog_write(0x83,0x34);

	/* cap from analog register */
	temp = analog_read(0x02);
	temp |= (1<<2);
	analog_write(0x02, temp);

	/* Disable 32K RC calibration. */
	temp = analog_read(0x83);
	temp &= ~(1<<0);//disable
	temp |= (1<<1);//Select calibrate 32k RC
	analog_write(0x83,temp);

	for(volatile int i=0; i<100; i++);

	/* Enable 32K RC calibration. */
	temp = analog_read(0x83);
	temp |= (1<<0);//Enable
	analog_write(0x83,temp);

	/* Wait Calibration completely. */
	for(volatile int i=0; i<10000; i++)
	{
		if((analog_read(0x84) & 0x01))
		{
			unsigned char CalValue = 0;
			CalValue = analog_read(0x85);
			analog_write(0x2f,CalValue);

			break;
		}
	}

	/* Disable 32K RC calibration. */
	temp = analog_read(0x83);
	temp &= ~(1<<0);
	analog_write(0x83,temp);

	/* cap from pm_top */
	temp = analog_read(0x02);
	temp &= ~(1<<2);
	analog_write(0x02, temp);
}









