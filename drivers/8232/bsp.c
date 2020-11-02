/********************************************************************************************************
 * @file     bsp.c
 *
 * @brief    This file provides set of common functions for driver
 *
 * @author   kaixin.chen@telink-semi.com; qiuwei.chen@telink-semi.com
 * @date     Oct. 8, 2016
 *
 * @par      Copyright (c) 2016, Telink Semiconductor (Shanghai) Co., Ltd.
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
_attribute_ram_code_  void cpu_wakeup_init(void)
#else
_attribute_ram_code_ void system_init(Bsp_InternalCapDef cap_flag)    //must on ramcode
#endif
{
	internal_cap_flag = cap_flag;
	load_tbl_cmd (tbl_sys_init, sizeof (tbl_sys_init)/sizeof (Cmd_TblDef));

	analog_write(0x00, 0xe8);//increase XTAL DCDC
	if(!internal_cap_flag)
	{
		analog_write(0x81, 0xe0); // set internal cap value
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
