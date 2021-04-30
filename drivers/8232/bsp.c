/********************************************************************************************************
 * @file	bsp.c
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
#include "bsp.h"
#include "clock.h"
#include "timer.h"
#include "analog.h"

const Cmd_TblDef  tbl_sys_init[] = {
	{0x60, 0x00,   TCMD_UNDER_BOTH | TCMD_WRITE},//open all the clk,disable all the rst
	{0x61, 0x00,   TCMD_UNDER_BOTH | TCMD_WRITE},//open all the clk,disable all the rst
	{0x62, 0x00,   TCMD_UNDER_BOTH | TCMD_WRITE},//open all the clk,disable all the rst
	{0x63, 0xff,   TCMD_UNDER_BOTH | TCMD_WRITE},//open all the clk,disable all the rst
	{0x64, 0xff,   TCMD_UNDER_BOTH | TCMD_WRITE},//open all the clk,disable all the rst
	{0x65, 0xff,   TCMD_UNDER_BOTH | TCMD_WRITE},
	{0x66, 0x00,   TCMD_UNDER_BOTH | TCMD_WRITE},

	{0x5b5, 0x0c,  TCMD_UNDER_BOTH | TCMD_WRITE},//Enable gpio(core) irq and wakeup for keyboard

	{0x03, 0x4b,   TCMD_UNDER_BOTH | TCMD_WAREG},//Increase Flash current
	{0x06, 0x00,   TCMD_UNDER_BOTH | TCMD_WAREG},

	{0x20, 0x00,   TCMD_UNDER_BOTH | TCMD_WAREG},//wakeup reset time: (0xff - 0xc1)*32 = 2000 us
	{0x2d, 0x48,   TCMD_UNDER_BOTH | TCMD_WAREG},//quick settle: 200 us
};

/**
 * @brief      This function performs a series of operations of writing digital or analog registers
 *             according to a command table
 * @param[in]  pt - pointer to a command table containing several writing commands
 * @param[in]  size  - number of commands in the table
 * @return     number of commands are carried out
 */
int load_tbl_cmd(const Cmd_TblDef * pt, int size){
	int l=0;

	while (l<size) {
		unsigned int  cadr = ((unsigned int)0x800000) | pt[l].ADR;
		unsigned char cdat = pt[l].DAT;
		unsigned char ccmd = pt[l].CMD;
		unsigned char cvld =(ccmd & TCMD_UNDER_WR);
		ccmd &= TCMD_MASK;
		if (cvld) {
			if (ccmd == TCMD_WRITE) {
				WRITE_REG8 (cadr, cdat);
			}
			else if (ccmd == TCMD_WAREG) {
				analog_write (cadr, cdat);
			}
			else if (ccmd == TCMD_WAIT) {
				delay_us (pt[l].ADR*256 + cdat);
			}
		}
		l++;
	}
	return size;

}

/**
 * @brief      This function writes a byte data to analog register
 * @param[in]  addr - the address of the analog register needs to write
 * @param[in]  value  - the data will be written to the analog register
 * @param[in]  e - the end address of value
 * @param[in]  s - the start address of the value
 * @return     none
 */
void sub_wr_ana(unsigned int addr, unsigned char value, unsigned char e, unsigned char s)
{
	unsigned char v, mask, tmp1, target, tmp2;

	v = analog_read(addr);
	mask = BIT_MASK_LEN(e - s + 1);
	tmp1 = value & mask;

	tmp2 = v & (~BIT_RNG(s,e));

	target = (tmp1 << s) | tmp2;
	analog_write(addr, target);
}

/**
 * @brief      This function writes a byte data to a specified analog register
 * @param[in]  addr - the address of the analog register needs to write
 * @param[in]  value  - the data will be written to the analog register
 * @param[in]  e - the end address of value
 * @param[in]  s - the start address of the value
 * @return     none
 */
void sub_wr(unsigned int addr, unsigned char value, unsigned char e, unsigned char s)
{
	unsigned char v, mask, tmp1, target, tmp2;

	v = READ_REG8(addr);
	mask = BIT_MASK_LEN(e - s + 1);
	tmp1 = value & mask;

	tmp2 = v & (~BIT_RNG(s,e));

	target = (tmp1 << s) | tmp2;
	WRITE_REG8(addr, target);
}


/**
 * @brief   This function serves to initialize the related analog registers
 *          to default values after MCU is waked up from deep sleep mode.
 * @param   none
 * @return  none
 */
unsigned char internal_cap_flag;
#if(BLE_SDK_EN)
_attribute_ram_code_sec_noinline_  void cpu_wakeup_init(void)
#else
_attribute_ram_code_sec_noinline_ void system_init(Bsp_InternalCapDef cap_flag)    //must on ramcode
#endif
{
	internal_cap_flag = cap_flag;
	load_tbl_cmd (tbl_sys_init, sizeof (tbl_sys_init)/sizeof (Cmd_TblDef));

	analog_write(0x00, 0xe8);//increase XTAL DCDC
	if(!internal_cap_flag)
	{
		analog_write(0x81, 0xe0); // set external cap value
	}
	else
	{
		analog_write(0x81, 0xe8); // set internal cap value
	}

    /* Open 24M XTAL. */
    analog_write(0x05, 0xca);
    for(volatile unsigned int i =0; i<10*24; i++);
	analog_write(0x05, 0xc2);
	for(volatile unsigned int i =0; i<210*24; i++);
	reg_dma_chn_en = 0;
	reg_dma_chn_irq_msk = 0;
	/* Set 24M XTAL buffer and doubler. */
	if(!internal_cap_flag)
	{
		analog_write(0x80, 0x21); //Enable 24M clk buf
		analog_write(0x81, 0xc0); //Enable 24M clk buf -> 0x4f
	}
	else
	{
		analog_write(0x80, 0x61); //Enable 24M clk buf
		analog_write(0x81, 0xd4); //Enable 24M clk buf -> 0x4f
	}

	analog_write(0x82, 0x5f); //Enable 48M doubler
	/* 24M RC calibrate. */
	rc_24m_cal();
	rc_32k_cal();
	/* initiate the value of 32k count */
	WRITE_REG16(0x748, 8000); //set 32k 16cyle avoid err use in a very quick suspend/deepsleep
	/* System Timer enable. */
	reg_sys_timer_ctrl |= FLD_SYSTEM_TICK_START;

	/* Must */
	WRITE_REG8(0x74a,0x29);//Enable calibration and close system timer 0x29
	WRITE_REG8(0x74a,0x28);//End calibration,calibration value is to be written to register 0x749|0x748.0x28
	WRITE_REG8(0x74a,0xa8);//Enable system timer and disable calibration
}
