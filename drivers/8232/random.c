/********************************************************************************************************
 * @file     random.c 
 *
 * @brief    This is the source file for TLSR8258
 *
 * @author	 junyuna.zhang@telink-semi.com;
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
#include "adc.h"

#include "analog.h"
#include "timer.h"

unsigned int rnd_m_w = 0;
unsigned int rnd_m_z = 0;

typedef union
{
	unsigned int rng32;

	struct {
		unsigned int bit0:1;
		unsigned int bit1:1;
		unsigned int bit2:1;
		unsigned int bit3:1;
		unsigned int bit4:1;
		unsigned int bit5:1;
		unsigned int bit6:1;
		unsigned int bit7:1;
		unsigned int bit8:1;
		unsigned int bit9:1;
		unsigned int bit10:1;
		unsigned int bit11:1;
		unsigned int bit12:1;
		unsigned int bit13:1;
		unsigned int bit14:1;
		unsigned int bit15:1;
		unsigned int bit16:1;

	}rng_bits;

}acd_rng;

volatile static acd_rng rng = {0};

static unsigned short rng_made(void)
{

	rng.rng_bits.bit16 = rng.rng_bits.bit16 ^ rng.rng_bits.bit15 ^ rng.rng_bits.bit13 ^ rng.rng_bits.bit4 ^ rng.rng_bits.bit0;
	if(rng.rng_bits.bit16)
	{
		rng.rng32 = (rng.rng32<<1)+ 1;
	}
	else
	{
		rng.rng32 = (rng.rng32<<1);
	}

	return ((unsigned short)rng.rng32);
}

/**
 * @brief This function serves to set adc sampling and get results.
 * @param[in]  none.
 * @return the result of sampling.
 */
volatile signed short adc_dat_buf_rd[16];
_attribute_ram_code_ unsigned short adc_rng_result(void)
{

	volatile unsigned short rng_result;

	unsigned char i;

	unsigned int t0 =  get_sys_tick();


	//dfifo setting will lose in suspend/deep, so we need config it every time
	adc_aif_set_misc_buf((unsigned short *)adc_dat_buf_rd,16);  //size: ADC_SAMPLE_NUM*4
	adc_aif_set_m_chn_en(1);
	adc_aif_set_use_raw_data_en();


	while(! timeout_us(t0, 25));  //wait at least 2 sample cycle(f = 96K, T = 10.4us)

	for(i=0;i<16;i++)
	{
		while((!adc_dat_buf_rd[i])||(! timeout_us(t0,16)));  //wait for new adc sample data,

		t0 =  get_sys_tick();

		rng.rng32 &= 0x0000ffff;
		if(adc_dat_buf_rd[i] & BIT(0))
		{
			rng.rng_bits.bit16 = 1;
		}

		rng_result = rng_made();

	}
	return rng_result;

}


/**
 * @brief     This function performs to preparatory initials random generator.
 * @param[in] none.
 * @return    none.
 */

void rng_init(void)
{
	rng.rng32 = 0x0000ffff;

	//ADC modle init
	adc_init();

	adc_vbat_init(GPIO_PA7);

	//After setting the ADC parameters, turn on the ADC power supply control bit
	adc_power_on(1);

	rnd_m_w = adc_rng_result()<<16 | adc_rng_result();
	rnd_m_z = adc_rng_result()<<16 | adc_rng_result();

	adc_power_on(0);

}

/**
 * @brief     This function performs to initialize the rand time in flash/sram.
 *            (example: system clock:16M, code in flash 23us, code in sram 4us)
 * @param[in] none.
 * @return    the value of time.
 */
_attribute_ram_code_ unsigned int rand(void)
{
	rnd_m_w = 18000 * (rnd_m_w & 0xffff) + (rnd_m_w >> 16);
	rnd_m_z = 36969 * (rnd_m_z & 0xffff) + (rnd_m_z >> 16);
	unsigned int result = (rnd_m_z << 16) + rnd_m_w;

	return (unsigned int)( result ^  get_sys_tick() );
}

/**
 * @brief       generate random number
 * @param[in]   len  - length of buffer
 * @param[out]  data - buffer
 * @return      none
 */
void rng_get_result_buff(int len, unsigned char *data)
{
	int i;
	unsigned int randNums = 0;
    /* if len is odd */
	for (i=0; i<len; i++ ) {
		if( (i & 3) == 0 ){
			randNums = rand();
		}

		data[i] = randNums & 0xff;
		randNums >>=8;
	}
}
