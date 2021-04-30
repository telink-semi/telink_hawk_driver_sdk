/********************************************************************************************************
 * @file	pwm.h
 *
 * @brief	This is the header file for TLSR8232
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
#ifndef PWM_H_
#define PWM_H_

#include "register.h"
#include "clock.h"
#include "timer.h"
/**
 * @brief  enum variable, the number of PWM channels supported
 */
typedef enum {
	PWM0 = 0,
	PWM1,
	PWM2,
}PWM_TypeDef;

/**
 * @brief  enum variable used for PWM work mode setting
 */
typedef enum{
	PWM_NORMAL_MODE   		= 0x00,
	PWM_COUNT_MODE    		= 0x01,
	PWM_IR_MODE       		= 0x03,
	PWM_IR_FIFO_MODE  		= 0x07,
	PWM_IR_DMA_FIFO_MODE  	= 0x0F,
}PWM_ModeDef;

/**
 * @brief  pwm interrupt source
 */
typedef enum{
	PWM0_PNUM_IRQ =					BIT(0),
	PWM0_IR_DMA_FIFO_DONE_IRQ =		BIT(1),
	PWM0_FRAME_IRQ =				BIT(2),
	PWM1_FRAME_IRQ =				BIT(3),
	PWM2_FRAME_IRQ =				BIT(4),
	PWM0_IR_FIFO_DONE_IRQ =         BIT(16),

}PWM_Irq_TypeDef;

/**
 * pwm interrupt source
 */

typedef enum{
	PWM0_PULSE_NORMAL =		0,       // duty cycle and period from TCMP0/TMAX0 					 0x794~0x797
	PWM0_PULSE_SHADOW =		BIT(14), // duty cycle and period from TCMP0_SHADOW / TMAX0_SHADOW   0x7c4~0x7c7
}PWM0_Pulse_TypeDef;


/**
 * @brief     This fuction servers to set pwm mode.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @param[in] mode - variable of enum to indicates the pwm mode.
 * @return	  none.
 */
static inline void pwm_set_mode(PWM_TypeDef id, PWM_ModeDef mode){
	if(PWM0 == id){
		reg_pwm0_mode = mode;  //only PWM0 has count/IR/fifo IR mode
	}
}

/**
 * @brief     This fuction servers to set pwm clock frequency
 * @param[in] system_clock_hz - variable to set system clock hz.
 * @param[in] pwm_clk - variable of the pwm clock.
 * @return	  none.
 */
static inline void pwm_set_clk(int system_clock_hz, int pwm_clk){
	reg_pwm_clk = (int)system_clock_hz /pwm_clk - 1;
}

/**
 * @brief     This fuction servers to set pwm count status(CMP) time.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @param[in] cmp_tick - variable of the CMP.
 * @return	  none.
 */
static inline void pwm_set_cmp(PWM_TypeDef id, unsigned short cmp_tick){
	reg_pwm_cmp(id) = cmp_tick;
}

/**
 * @brief     This fuction servers to set pwm cycle time.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @param[in] cycle_tick - variable of the cycle time.
 * @return	  none.
 */
static inline void pwm_set_max(PWM_TypeDef id, unsigned short max_tick){
	reg_pwm_max(id) = max_tick;
}

/**
 * @brief     This fuction servers to set pwm cycle time & count status.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @param[in] cycle_tick - variable of the cycle time.
 * @param[in] cmp_tick - variable of the CMP.
 * @return	  none.
 */
static inline void pwm_set_max_and_cmp(PWM_TypeDef id, unsigned short max_tick, unsigned short cmp_tick){
	reg_pwm_cycle(id) = MASK_VAL(FLD_PWM_CMP, cmp_tick, FLD_PWM_MAX, max_tick);
}

/**
 * @brief     This fuction servers to set pwm cycle time & count status.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @param[in] cycle_tick - variable of the cycle time.
 * @param[in] cmp_tick - variable of the CMP.
 * @return	  none.
 */
static inline void pwm_set_shadow_max_and_cmp(PWM_TypeDef id, unsigned short max_tick, unsigned short cmp_tick)
{
	if(PWM0==id)
	{
		reg_pwm_tcmp0_shadow = cmp_tick;
		reg_pwm_tmax0_shadow = max_tick;
	}
}

/**
 * @brief     This fuction servers to set the pwm pulse number.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @param[in] pulse_num - variable of the pwm pulse number.
 * @return	  none.
 */
static inline void pwm_set_pulse_num(PWM_TypeDef id, unsigned short pulse_num){
	if(PWM0 == id){
		reg_pwm0_pulse_num = pulse_num;
	}

}

/**
 * @brief     This fuction servers to start the pwm.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @return	  none.
 */
static inline void pwm_start(PWM_TypeDef id){

	BM_SET(reg_pwm_enable, BIT(id));
}

/**
 * @brief     This fuction servers to stop the pwm.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @return	  none.
 */
static inline void pwm_stop(PWM_TypeDef id){

	BM_CLR(reg_pwm_enable, BIT(id));
}

/**
 * @brief     This fuction servers to revert the PWMx.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @return	  none.
 */
static inline void pwm_revert(PWM_TypeDef id){
	reg_pwm_invert |= BIT(id);
}

/**
 * @brief     This fuction servers to revert the PWMx_N.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @return	  none.
 */
static inline void pwm_n_revert(PWM_TypeDef id){
	reg_pwm_n_invert |= BIT(id);
}

/**
 * @brief     This fuction servers to enable the pwm polarity.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @param[in] en: 1 enable. 0 disable.
 * @return	  none.
 */
static inline void pwm_set_pol(PWM_TypeDef id, int en){
	if(en){
		BM_SET(reg_pwm_pol, BIT(id));
	}else{
		BM_CLR(reg_pwm_pol, BIT(id));
	}
}

/**
 * @brief     This fuction servers to enable the pwm interrupt.
 * @param[in] PWM_Irq_TypeDef - variable of enum to select the pwm interrupt source.
 * @return	  none.
 */
static inline void pwm_set_irq_en(PWM_Irq_TypeDef irq, unsigned char en){
	if(en){
		if(irq==PWM0_IR_FIFO_DONE_IRQ)
		{
		BM_SET(reg_pwm_irq_mask(1), BIT(0));
		}
		else
		{
		BM_SET(reg_pwm_irq_mask(0), irq);
		}
    }else{
    	if(irq==PWM0_IR_FIFO_DONE_IRQ)
    	{
		   BM_CLR(reg_pwm_irq_mask(1),  BIT(0));
    	}
    	else
    	{
    	   BM_CLR(reg_pwm_irq_mask(0),  irq);
    	}
	}
}


/**
 * @brief     This fuction servers to get  the pwm interrupt.
 * @param[in] PWM_Irq_TypeDef - variable of enum to select the pwm interrupt source.
 * @return	  none.
 */
static inline char pwm_get_irq_status(PWM_Irq_TypeDef irq){
	if(irq==PWM0_IR_FIFO_DONE_IRQ)
	{
        return (reg_pwm_irq_sta(1)&BIT(0));
	}
	else
	{
		return (reg_pwm_irq_sta(0)&irq);
	}
}


/**
 * @brief     This fuction servers to clear the pwm interrupt.
 * @param[in] PWM_Irq_TypeDef - variable of enum to select the pwm interrupt source.
 * @return	  none.
 */
static inline void pwm_clr_irq_status(PWM_Irq_TypeDef irq){
	if(irq==PWM0_IR_FIFO_DONE_IRQ)
		{
		reg_pwm_irq_sta(1) = BIT(0);
		}
		else
		{
			reg_pwm_irq_sta(0) = irq;
		}

}

/**
 * @brief     This fuction serves to set trigger level of interrupt for IR FiFo mode
 * @param[in] none
 * @return	  none
 */
static inline void pwm_ir_fifo_set_irq_trig_level(unsigned char trig_level)
{
	reg_pwm_ir_fifo_irq_trig_level = trig_level;
}
/**
 * @brief     This fuction serves to clear data in fifo. Only when pwm is in not active mode,
 * 			  it is possible to clear data in fifo.
 * @param[in] none
 * @return	  none
 */
static inline void pwm_ir_fifo_clr_data(void)
{
	reg_pwm_ir_clr_fifo_data |= FLD_PWM0_IR_FIFO_CLR_DATA;
}
/**
 * @brief     This fuction serves to get the number of data in fifo.
 * @param[in] none
 * @return	  the number of data in fifo
 */
static inline unsigned char pwm_ir_fifo_get_data_num(void)
{
	return (reg_pwm_ir_fifo_data_status&FLD_PWM0_IR_FIFO_DATA_NUM);
}
/**
 * @brief     This fuction serves to determine whether data in fifo is empty.
 * @param[in] none
 * @return	  yes: 1 ,no: 0;
 */
static inline unsigned char pwm_ir_fifo_is_empty(void)
{
	return (reg_pwm_ir_fifo_data_status&FLD_PWM0_IR_FIFO_EMPTY);
}

/**
 * @brief     This fuction serves to determine whether data in fifo is full.
 * @param[in] none
 * @return	  yes: 1 ,no: 0;
 */

static inline unsigned char pwm_ir_fifo_is_full(void)
{
	return (reg_pwm_ir_fifo_data_status&FLD_PWM0_IR_FIFO_FULL);
}

/**
 * @brief     This fuction serves to write data into FiFo
 * @param[in] pulse_num  - the number of pulse
 * @param[in] use_shadow - determine whether the configuration of shadow cmp and shadow max is used
 * 						   1: use shadow, 0: not use
 * @param[in] carrier_en - enable sending carrier, 1: enable, 0: disable
 * @return	  none
 */
static inline void pwm_ir_fifo_set_data_entry(unsigned short pulse_num, unsigned char use_shadow, unsigned char carrier_en)
{
	static unsigned char index=0;
	unsigned short cfg_data = pulse_num + ((use_shadow&BIT(0))<<14) + ((carrier_en&BIT(0))<<15);
	while(pwm_ir_fifo_is_full());
	reg_pwm_ir_fifo_dat(index) = cfg_data;
	index++;
	index&=0x01;
}

/**
 * @brief     This fuction serves to config the pwm's dma wave form.
 * @param[in] carrier_en - must 1 or 0.
 * @param[in] PWM0_Pulse_TypeDef - type of pwm0's pulse.
 * @param[in] pulse_num - the number of pulse.
 * @return	  none.
 */
static inline unsigned short pwm_ir_dma_fifo_set_waveform(int carrier_en, PWM0_Pulse_TypeDef pulse_type,  unsigned short pulse_num)
{
	return  ( carrier_en<<15 | pulse_type | (pulse_num & 0x3fff) );
}

/**
 * @brief     This fuction servers to set the pwm's dma address.
 * @param[in] pdat - variable of pointer to indicate the address.
 * @return	  none.
 */
static inline void pwm_set_dma_addr(void * pdat)
{
	reg_dma_pwm_addr = (unsigned short)((unsigned int)pdat);
	reg_dma_pwm_mode  &= ~FLD_DMA_WR_MEM;
}

/**
 * @brief     This fuction servers to start the pwm's IRQ sending.
 * @param[in] none.
 * @return	  none.
 */
static inline void pwm_ir_dma_fifo_start_tx(void)
{
	reg_dma_chn_en |= FLD_DMA_CHN_PWM;
	reg_dma_tx_rdy0 |= FLD_DMA_CHN_PWM;
}

/**
 * @brief     This fuction servers to stop the pwm's IRQ sending.
 * @param[in] none.
 * @return	  none.
 */
static inline void pwm_ir_dma_fifo_stop_tx(void)
{
//	reg_dma_tx_rdy0 &= ~FLD_DMA_PWM;

	reg_rst0 = FLD_RST1_PWM;
	delay_us(20);  //1us <-> 4 byte
	reg_rst0 = 0;
}

#endif /* PWM_H_ */


/** \defgroup GP8  PWM Examples
 *
 * 	@{
 */

/*! \page pwm Table of Contents
	- [API-PWM-CASE1:PWM NORMAL](#PWM_NORMAL)
	- [API-PWM-CASE2:PWM COUNT](#PWM_COUNT)
	- [API-PWM-CASE3:PWM IR](#PWM_IR)
	- [API-PWM-CASE4:PWM IR FIFO](#PWM_IR_FIFO)
	- [API-PWM-CASE5:PWM IR DMA FIFO](#PWM_IR_DMA_FIFO)

<h1 id=PWM_NORMAL> API-PWM-CASE1:PWM NORMAL </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | pwm_set_clk() | pwm_set_clk(CLOCK_SYS_CLOCK_HZ, CLOCK_SYS_CLOCK_HZ) | set the clock frequency of PWM | ^ |
| ^ | ^ | gpio_set_func() | gpio_set_func(PWM_PIN, AS_PWMx) | set pin as PWM | ^ |
| ^ | ^ | pwm_set_mode() | pwm_set_mode(PWM_ID, PWM_NORMAL_MODE) | set mode of PWM | ^ |
| ^ | ^ | pwm_set_phase() | pwm_set_phase(PWM_ID, 0) | set the phase of PWM | ^ |
| ^ | ^ | pwm_set_max_and_cmp() | pwm_set_max_and_cmp(PWM_ID, 2, 1) | set the max and cmp for PWM | ^ |
| ^ | ^ | pwm_start() | pwm_start(PWM_ID) | start PWM | ^ |
| ^ | main_loop() | none || Main program loop | ^ |

\n
Variables above are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
#define PWM_PIN		GPIO_PA2
#define AS_PWMx		AS_PWM0
#define PWM_ID		PWM0
~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=PWM_COUNT> API-PWM-CASE2:PWM COUNT </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | pwm_set_clk() | pwm_set_clk(CLOCK_SYS_CLOCK_HZ, CLOCK_SYS_CLOCK_HZ) | set the clock frequency of PWM | ^ |
| ^ | ^ | gpio_set_func() | gpio_set_func(PWM_PIN, AS_PWMx) | set pin as PWM | ^ |
| ^ | ^ | pwm_set_mode() | pwm_set_mode(PWM_ID, PWM_COUNT_MODE) | set mode of PWM | ^ |
| ^ | ^ | pwm_set_pulse_num() | pwm_set_pulse_num(PWM_ID,PWM_PULSE_NUM) | set the bumber of pulse | ^ |
| ^ | ^ | pwm_set_phase() | pwm_set_phase(PWM_ID, 0) | set the phase of PWM | ^ |
| ^ | ^ | pwm_set_max_and_cmp() | pwm_set_max_and_cmp(PWM_ID, 2, 1) | set the max and cmp for PWM | ^ |
| ^ | ^ | pwm_start() | pwm_start(PWM_ID) | start PWM | ^ |
| ^ | main_loop() | none || Main program loop | ^ |

\n
Variables above are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
#define PWM_PIN			GPIO_PA2
#define AS_PWMx			AS_PWM0
#define PWM_ID			PWM0
#define PWM_PULSE_NUM	3
~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=PWM_IR> API-PWM-CASE3:PWM IR </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | pwm_set_clk() | pwm_set_clk(CLOCK_SYS_CLOCK_HZ, CLOCK_SYS_CLOCK_HZ) | set the clock frequency of PWM | ^ |
| ^ | ^ | gpio_set_func() | gpio_set_func(PWM_PIN, AS_PWMx) | set pin as PWM | ^ |
| ^ | ^ | pwm_set_mode() | pwm_set_mode(PWM_ID, PWM_IR_MODE) | set mode of PWM | ^ |
| ^ | ^ | pwm_set_pulse_num() | pwm_set_pulse_num(PWM_ID,PWM_PULSE_NUM) | set the bumber of pulse | ^ |
| ^ | ^ | pwm_set_phase() | pwm_set_phase(PWM_ID, 0) | set the phase of PWM | ^ |
| ^ | ^ | pwm_set_max_and_cmp() | pwm_set_max_and_cmp(PWM_ID, 2, 1) | set the max and cmp for PWM | ^ |
| ^ | ^ | pwm_start() | pwm_start(PWM_ID) | start PWM | ^ |
| ^ | ^ | pwm_set_mode() | pwm_set_mode(PWM_ID, PWM_COUNT_MODE) | switch to count mode to stop IR mode  | ^ |
| ^ | main_loop() | none || Main program loop | ^ |

\n
Variables above are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
#define PWM_PIN					GPIO_PA2
#define AS_PWMx					AS_PWM0
#define PWM_ID					PWM0
#define PWM_PULSE_NUM			2
~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=PWM_IR_FIFO> API-PWM-CASE4:PWM IR FIFO </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | pwm_set_clk() | pwm_set_clk(CLOCK_SYS_CLOCK_HZ, CLOCK_SYS_CLOCK_HZ) | set the clock frequency of PWM | ^ |
| ^ | ^ | gpio_set_func() | gpio_set_func(PWM_PIN, AS_PWMx) | set pin as PWM | ^ |
| ^ | ^ | pwm_set_mode() | pwm_set_mode(PWM_ID, PWM_IR_FIFO_MODE) | set mode of PWM | ^ |
| ^ | ^ | pwm_set_pulse_num() | pwm_set_pulse_num(PWM_ID,PWM_PULSE_NUM) | set the bumber of pulse | ^ |
| ^ | ^ | pwm_set_phase() | pwm_set_phase(PWM_ID, 0) | set the phase of PWM | ^ |
| ^ | ^ | pwm_set_max_and_cmp() | pwm_set_max_and_cmp(PWM_ID, 2, 1) | set the max and cmp for PWM | ^ |
| ^ | ^ | pwm_set_shadow_max_and_cmp() | pwm_set_shadow_max_and_cmp(PWM_ID,4, 2) | set the max and cmp of shadow for PWM | ^ |
| ^ | ^ | pwm_ir_fifo_set_data_entry() | pwm_ir_fifo_set_data_entry(PWM_PULSE_NUM1,0,1) | set data entry of IR fifo for PWM | ^ |
| ^ | ^ | ^ | pwm_ir_fifo_set_data_entry(PWM_PULSE_NUM2,0,0) | ^ | ^ |
| ^ | ^ | ^ | pwm_ir_fifo_set_data_entry(PWM_PULSE_NUM3,1,1) | ^ | ^ |
| ^ | ^ | pwm_start() | pwm_start(PWM_ID) | start PWM | ^ |
| ^ | main_loop() | none || Main program loop | ^ |

\n
Variables above are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
#define PWM_PIN					GPIO_PA2
#define AS_PWMx					AS_PWM0
#define PWM_ID					PWM0
#define PWM_PULSE_NUM1			5
#define PWM_PULSE_NUM2			5
#define PWM_PULSE_NUM3			10
~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=PWM_IR_DMA_FIFO> API-PWM-CASE5:PWM IR DMA FIFO </h1>


| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | if(reg_pwm_irq_sta & FLD_IRQ_PWM0_IR_DMA_FIFO_DONE) ||| determine whether interrupt flag of IR DMA fifo done is right | 2019-1-10 |
| ^ | reg_pwm_irq_sta &Iota;= FLD_IRQ_PWM0_IR_DMA_FIFO_DONE ||| clear interrupt flag | 2019-1-10 |
| ^ | ir_dma_fifo_cnt++ ||| Interrupt processing | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | pwm_set_clk() | pwm_set_clk(CLOCK_SYS_CLOCK_HZ, CLOCK_SYS_CLOCK_HZ) | set the clock frequency of PWM | ^ |
| ^ | ^ | gpio_set_func() | gpio_set_func(PWM_PIN, AS_PWMx) | set pin as PWM | ^ |
| ^ | ^ | pwm_set_mode() | pwm_set_mode(PWM_ID, PWM_IR_DMA_FIFO_MODE) | set mode of PWM | ^ |
| ^ | ^ | pwm_set_pulse_num() | pwm_set_pulse_num(PWM_ID,PWM_PULSE_NUM) | set the bumber of pulse | ^ |
| ^ | ^ | pwm_set_phase() | pwm_set_phase(PWM_ID, 0) | set the phase of PWM | ^ |
| ^ | ^ | pwm_set_max_and_cmp() | pwm_set_max_and_cmp(PWM_ID, IR_DMA_MAX_TICK, IR_DMA_CMP_TICK) | set the max and cmp for PWM | ^ |
| ^ | ^ | pwm_set_shadow_max_and_cmp() | pwm_set_shadow_max_and_cmp(PWM_ID,IR_DMA_SHADOW_MAX_TICK, IR_DMA_SHADOW_CMP_TICK) | set the max and cmp of shadow for PWM | ^ |
| ^ | ^ | pwm_ir_dma_fifo_set_waveform() | IR_DMA_Buff[index++]= pwm_ir_dma_fifo_set_waveform(1, PWM0_PULSE_NORMAL, 9000 * CLOCK_SYS_CLOCK_1US/IR_DMA_MAX_TICK) | set waveform of IR DMA FIFO mode for PWM | ^ |
| ^ | ^ | ^ | ... for details, refer to [Driver Demo](#) || ^ |
| ^ | ^ | pwm_set_dma_addr() | pwm_set_dma_addr(&IR_DMA_Buff) | set the address of DMA | ^ |
| ^ | ^ | reg_irq_mask &Iota;= FLD_IRQ_SW_PWM_EN || enable pwm interrrupt | ^ |
| ^ | ^ | reg_pwm_irq_mask &Iota;= FLD_IRQ_PWM0_IR_DMA_FIFO_DONE || enable IR DMA FIFO done interrupt of pwm | ^ |
| ^ | ^ | irq_enable() || enable global interrupt | ^ |
| ^ | ^ | pwm_ir_dma_fifo_start_tx() || start IR DMA fifo mode of PWM | ^ |
| ^ | main_loop() | none || Main program loop | ^ |

\n
Variables above are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
#define PWM_PIN					GPIO_PA2
#define AS_PWMx					AS_PWM0
#define PWM_ID					PWM0

#define IR_DMA_CARRIER_FREQ				38000
#define IR_DMA_MAX_TICK					(CLOCK_SYS_CLOCK_HZ/IR_DMA_CARRIER_FREQ)
#define IR_DMA_CMP_TICK					(IR_DMA_MAX_TICK/2)

#define IR_DMA_SHADOW_CARRIER_FREQ		56000
#define IR_DMA_SHADOW_MAX_TICK			(CLOCK_SYS_CLOCK_HZ/IR_DMA_SHADOW_CARRIER_FREQ)
#define IR_DMA_SHADOW_CMP_TICK			(IR_DMA_SHADOW_MAX_TICK/2)

unsigned short IR_DMA_Buff[64]={0};

unsigned int ir_dma_fifo_cnt=0;

unsigned char index = 2;
~~~~~~~~~~~~~~~~~~~~~~~~~~~


<h1> History Record </h1>

| Date | Description | Author |
| :--- | :---------- | :----- |
| 2019-1-10 | initial release | LJW |


*/

 /** @}*/ //end of GP8
