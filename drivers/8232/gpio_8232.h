/********************************************************************************************************
 * @file     gpio.h
 *
 * @brief    This file provides set of functions to manage GPIOs
 *
 * @author   jian.zhang@telink-semi.com;
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
#ifndef __GPIO_H
#define __GPIO_H

#include "bsp.h"
#include "gpio_default_8232.h"
#include "register_8232.h"

/**
 *  @brief  Define GPIO types
 */

typedef enum {
	GPIO_GROUPA  = 0x000,
	GPIO_GROUPB  = 0x100,
	GPIO_GROUPC  = 0x200,
	GPIO_GROUPD  = 0x300,
	GPIO_GROUPE  = 0x400,

    GPIO_PA0  = GPIO_GROUPA | BIT(0),	GPIO_PWM0A0  = GPIO_PA0,
    GPIO_PA1  = GPIO_GROUPA | BIT(1),	GPIO_UCTSA1   = GPIO_PA1,
    GPIO_PA2  = GPIO_GROUPA | BIT(2), 	GPIO_PWM1NA2 = GPIO_PA2,	GPIO_URTSA2   = GPIO_PA2,
    GPIO_PA3  = GPIO_GROUPA | BIT(3), 	GPIO_UTXA3    = GPIO_PA3,	GPIO_I2C_MCKA3 = GPIO_PA3,	GPIO_SPI_CKA3 = GPIO_PA3,
    GPIO_PA4  = GPIO_GROUPA | BIT(4), 	GPIO_PWM2A4  = GPIO_PA4,  	GPIO_URXA4    = GPIO_PA4,	GPIO_I2C_MSDA4 = GPIO_PA4,	GPIO_SPI_CKA4 = GPIO_PA4,
    GPIO_PA5  = GPIO_GROUPA | BIT(5), 	GPIO_I2C_CKA5 = GPIO_PA5,	GPIO_I2C_MCKA5 = GPIO_PA5,
    GPIO_PA6  = GPIO_GROUPA | BIT(6), 	GPIO_I2C_SDA6 = GPIO_PA6,	GPIO_I2C_MSDA6 = GPIO_PA6,	GPIO_CE_LNAA6 = GPIO_PA6,
    GPIO_PA7  = GPIO_GROUPA | BIT(7), 	GPIO_CE_PAA7  = GPIO_PA7,
    GPIOA_ALL = GPIO_GROUPA | 0x00ff,

    GPIO_PB0  = GPIO_GROUPB | BIT(0), 	GPIO_SPI_MCNB0 = GPIO_PB0,	GPIO_CE_LNAB0  = GPIO_PB0,
    GPIO_PB1  = GPIO_GROUPB | BIT(1), 	GPIO_PWM1B1  = GPIO_PB1,	GPIO_SPI_MDOB1 = GPIO_PB1,	GPIO_CE_PAB1   = GPIO_PB1,	GPIO_PGAB1 = GPIO_PB1,
    GPIO_PB2  = GPIO_GROUPB | BIT(2), 	GPIO_PWM2B2  = GPIO_PB2,   	GPIO_SPI_MDIB2 = GPIO_PB2,  GPIO_UCTSB2    = GPIO_PB2,	GPIO_I2C_MCKB2 = GPIO_PB2, GPIO_PGAB2 = GPIO_PB2,
    GPIO_PB3  = GPIO_GROUPB | BIT(3), 	GPIO_PWM0B3  = GPIO_PB3,  	GPIO_SPI_MCKB3 = GPIO_PB3,	GPIO_URTSB3    = GPIO_PB3,	GPIO_I2C_MSDB3 = GPIO_PB3,
    GPIO_PB4  = GPIO_GROUPB | BIT(4), 	GPIO_PWM1NB4 = GPIO_PB4,    GPIO_UTXB4     = GPIO_PB4,	GPIO_PGAB4     = GPIO_PB4,
    GPIO_PB5  = GPIO_GROUPB | BIT(5),   GPIO_URXB5     = GPIO_PB5,	GPIO_PGAB5     = GPIO_PB5,
    GPIO_PB6  = GPIO_GROUPB | BIT(6), 	GPIO_PWM0NB6 = GPIO_PB6,    GPIO_I2C_MCKB6 = GPIO_PB6,	GPIO_URTSB6    = GPIO_PB6,
    GPIO_PB7  = GPIO_GROUPB | BIT(7), 	GPIO_PWM1B7  = GPIO_PB7,  	GPIO_I2C_MSDB7 = GPIO_PB7,  GPIO_UCTSB7    = GPIO_PB7,
    GPIOB_ALL = GPIO_GROUPB | 0x00ff,


    GPIO_PC1  = GPIO_GROUPC | BIT(1),	GPIO_PWM2NC1 = GPIO_PC1,
    GPIO_PC2  = GPIO_GROUPC | BIT(2),	GPIO_PWM0NC2 = GPIO_PC2,	GPIO_SPI_CNC2 = GPIO_PC2,	GPIO_SPI_MCNC2 = GPIO_PC2,	GPIO_UCTSC2 = GPIO_PC2,
    GPIO_PC3  = GPIO_GROUPC | BIT(3),	GPIO_SPI_DOC3 = GPIO_PC3,	GPIO_SPI_MDOC3 = GPIO_PC3,	GPIO_URTSC3 = GPIO_PC3,
    GPIO_PC4  = GPIO_GROUPC | BIT(4),	GPIO_SPI_DIC4 = GPIO_PC4,	GPIO_I2C_MSDC4 = GPIO_PC4,	GPIO_SPI_MDIC4 = GPIO_PC4,	GPIO_UTXC4 = GPIO_PC4,
    GPIO_PC5  = GPIO_GROUPC | BIT(5),	GPIO_SPI_CKC5 = GPIO_PC5,  GPIO_I2C_MCKC5 = GPIO_PC5,	GPIO_SPI_MCKC5 = GPIO_PC5,	GPIO_URXC5 = GPIO_PC5,
    GPIO_PC6  = GPIO_GROUPC | BIT(6),
    GPIO_PC7  = GPIO_GROUPC | BIT(7),	GPIO_SWS = GPIO_PC7,
    GPIOC_ALL = GPIO_GROUPC | 0x00ff,

    GPIO_PE0  = GPIO_GROUPE | BIT(0),
    GPIO_PE1  = GPIO_GROUPE | BIT(1),
    GPIO_PE2  = GPIO_GROUPE | BIT(2),
    GPIO_PE3  = GPIO_GROUPE | BIT(3),
    GPIOE_ALL  = GPIO_GROUPE | 0x00ff,

    GPIO_ALL = 0x500,
} GPIO_PinTypeDef;

/**
 *  @brief  Define GPIO Function types
 */

typedef enum{
    AS_GPIO	   =  0,

    AS_MSPI	   =  1,
    AS_SWIRE   =  2,
    AS_UART	   =  3,
    AS_PWM	   =  4,
    AS_I2C	   =  5,
    AS_SPI	   =  6,
    AS_ETH_MAC =  7,
    AS_I2S	   =  8,
    AS_SDM	   =  9,
    AS_DMIC	   =  10,
    AS_USB	   =  11,
    AS_SWS	   =  12,
    AS_SWM	   =  13,
    AS_TEST	   =  14,
    AS_ADC	   =  15,
    AS_KS      =  16,
    AS_DEBUG   =  17,

    AS_PWM0 	= 20,
    AS_PWM1		= 21,
    AS_PWM2 	= 22,

    AS_PWM0_N	= 26,
    AS_PWM1_N	= 27,
    AS_PWM2_N	= 28,


    AS_32K_CLK_OUTPUT   = 32,
    AS_RESERVE_0   = 33,
    AS_RESERVE_1   = 34,
    AS_RESERVE_2   = 35,
    AS_RESERVE_3   = 36,
    AS_UART_CTS    = 37,
    AS_UART_RTS    = 38,
    AS_UART_TX     = 39,
    AS_UART_RX     = 40,

    AS_I2C_CK      = 41,
    AS_I2C_MCK     = 42,
    AS_I2C_MSD     = 43,
    AS_I2C_SD_OR_SPI_DI  = 44,
    AS_I2C_CK_OR_SPI_CK  = 45,
    AS_I2C_SD            = 46,

    AS_RX_CYC2LNA        = 50,
    AS_SYS_CLK_OUTPUT    = 52,
    AS_TX_CYC2PA         = 53,
    AS_SPI_MCN           = 54,
    AS_SPI_MDO           = 55,
    AS_SPI_MDI           = 56,
    AS_SPI_MCK           = 57,

    AS_SPI_CN            = 58,
    AS_SPI_DO            = 59,
    AS_SPI_DI            = 60,
    AS_SPI_CK            = 61,

    AS_RX_CYC2LNA_OR_SPI_CN = 63,
    AS_TX_CYC2LNA_OR_SPI_DO = 64,
    AS_UART_CTS_OR_SPI_DI   = 65,
    AS_UART_RTS_OR_SPI_CK   = 66,

}GPIO_FuncTypeDef;

typedef enum{
	LEVEL_LOW = 0,
	LEVEL_HIGH,
}GPIO_LevelTypeDef;

/**
 *  @brief  Define rising/falling types
 */
typedef enum{
	POL_RISING = 0,
	POL_FALLING,
}GPIO_PolTypeDef;

/**
 *  @brief  Define pull-up/down types
 */

typedef enum {
	PM_PIN_UP_DOWN_FLOAT    = 0,
	PM_PIN_PULLUP_1M     	= 1,
	PM_PIN_PULLUP_10K 		= 2,
	PM_PIN_PULLDOWN_100K  	= 3,

}GPIO_PullTypeDef;

/**
 * @brief      This function servers to initialization all gpio.
 * @param[in]  none.
 * @return     none.
 */
/**Processing methods of unused GPIO
 * Set it to high resistance state and set it to open pull-up or pull-down resistance to
 * let it be in the determined state.When GPIO uses internal pull-up or pull-down resistance,
 * do not use pull-up or pull-down resistance on the board in the process of practical
 * application because it may have the risk of electric leakage .
 */
void gpio_init(void);
/**
 * @brief      This function servers to set the GPIO's function.
 * @param[in]  pin - the special pin.
 * @param[in]  func - the function of GPIO.
 * @return     none.
 */
/**Steps to set GPIO as a multiplexing function is as follows.
 * Step 1: Set GPIO as a multiplexing function.
 * Step 2: Disable GPIO function.
 * NOTE: Failure to follow the above steps may result in risks.
 */
void gpio_set_func(GPIO_PinTypeDef pin, GPIO_FuncTypeDef func);

/**
 * @brief      This function set the output function of a pin.
 * @param[in]  pin - the pin needs to set the output function
 * @param[in]  value - enable or disable the pin's output function(0: enable, 1: disable)
 * @return     none
 */
static inline void gpio_set_output_en(GPIO_PinTypeDef pin, unsigned int value)
{
	unsigned char bit = pin & 0xff;
	if(!value){
		BM_SET(reg_gpio_oen(pin), bit);
	}else{
		BM_CLR(reg_gpio_oen(pin), bit);
	}
}

/**
 * @brief      This function set the input function of a pin.
 * @param[in]  pin - the pin needs to set the input function
 * @param[in]  value - enable or disable the pin's input function(0: disable, 1: enable)
 * @return     none
 */
void gpio_set_input_en(GPIO_PinTypeDef pin, unsigned int value);

/**
 * @brief      This function determines whether the output function of a pin is enabled.
 * @param[in]  pin - the pin needs to determine whether its output function is enabled.
 * @return     1: the pin's output function is enabled ;
 *             0: the pin's output function is disabled
 */
static inline int gpio_is_output_en(GPIO_PinTypeDef pin)
{

	return !BM_IS_SET(reg_gpio_oen(pin), pin & 0xff);
}
/**
 * @brief     This function to judge whether a pin's input is enable.
 * @param[in] pin - the pin needs to enable its input.
 * @return    1:enable the pin's input function.
 *            0:disable the pin's input function.
 */
int gpio_is_input_en(GPIO_PinTypeDef pin);


/**
 * @brief     This function set the pin's output level.
 * @param[in] pin - the pin needs to set its output level
 * @param[in] value - value of the output level(1: high 0: low)
 * @return    none
 */
static inline void gpio_write(GPIO_PinTypeDef pin, unsigned int value)
{
	unsigned char bit = pin & 0xff;
	if(value){
		BM_SET(reg_gpio_out(pin), bit);
	}else{
		BM_CLR(reg_gpio_out(pin), bit);
	}
}

/**
 * @brief     This function read the pin's input/output level.
 * @param[in] pin - the pin needs to read its level
 * @return    the pin's level(1: high 0: low)
 */
static inline unsigned int gpio_read(GPIO_PinTypeDef pin)
{
	return BM_IS_SET(reg_gpio_in(pin), pin & 0xff);
}

/**
 * @brief     This function set the pin toggle.
 * @param[in] pin - the pin needs to toggle
 * @return    none
 */
static inline void gpio_toggle(GPIO_PinTypeDef pin)
{
	reg_gpio_out(pin) ^= (pin & 0xFF);
}

/**
 * @brief      This function set the pin's driving strength.
 * @param[in]  pin - the pin needs to set the driving strength
 * @param[in]  value - the level of driving strength(1: strong 0: poor)
 * @return     none
 */
void gpio_set_data_strength(GPIO_PinTypeDef pin, unsigned int value);

/**
 * @brief     This function set a pin's pull-up/down resistor.
 * @param[in] gpio - the pin needs to set its pull-up/down resistor
 * @param[in] up_down - the type of the pull-up/down resistor
 * @return    none
 */
void gpio_set_up_down_resistor(GPIO_PinTypeDef gpio, GPIO_PullTypeDef up_down);

/**
 * @brief      This function servers to set the specified GPIO as high resistor.
 * @param[in]  pin  - select the specified GPIO
 * @return     none.
 */
void gpio_shutdown(GPIO_PinTypeDef pin);

/**
 * @Brief: 		This function serves to disable or enable 50k pull-up resistor. only PA6,PA7 and PB0~PB7 have
 *        		50K pull-up resistor.
 * @Param[in]: 	en - 0: disable; 1: enable. (disable by default)
 * @Return:		none.
 */
void gpio_set_50k_pull_up_en(unsigned int en);

/**
 * @brief     This function set a pin's IRQ.
 * @param[in] pin - the pin needs to enable its IRQ
 * @param[in] falling - value of the edge polarity(1: falling edge 0: rising edge)
 * @return    none
 */
static inline void gpio_set_interrupt(GPIO_PinTypeDef pin, GPIO_PolTypeDef falling)
{
	unsigned char bit = pin & 0xff;
	BM_SET(reg_gpio_irq_wakeup_en(pin), bit);
	reg_gpio_wakeup_irq |= FLD_GPIO_CORE_INTERRUPT_EN;
	if(falling){
		BM_SET(reg_gpio_pol(pin), bit);
	}else{
		BM_CLR(reg_gpio_pol(pin), bit);
	}
/*clear gpio interrupt source (after setting gpio polarity,before enable interrupt)to avoid unexpected interrupt. confirm by minghai*/
	reg_irq_src |= FLD_IRQ_GPIO_EN|FLD_IRQ_GPIO_RISC0_EN|FLD_IRQ_GPIO_RISC1_EN|FLD_IRQ_GPIO_RISC2_EN;
	reg_irq_mask |= FLD_IRQ_GPIO_EN ;
}

/**
 * @brief     This function set a pin's IRQ.
 * @param[in] pin - the pin needs to enable its IRQ
 * @param[in] falling - value of the edge polarity(1: falling edge 0: rising edge)
 * @return    none
 */
static inline void gpio_set_interrupt_risc0(GPIO_PinTypeDef pin, GPIO_PolTypeDef falling)
{
	unsigned char bit = pin & 0xff;
	BM_SET(reg_gpio_irq_risc0_en(pin), bit);
	if(falling){
		BM_SET(reg_gpio_pol(pin), bit);
	}else{
		BM_CLR(reg_gpio_pol(pin), bit);
	}
/*clear gpio interrupt sorce (after setting gpio polarity,before enable interrupt)to avoid unexpected interrupt. confirm by minghai*/
	reg_irq_src |= FLD_IRQ_GPIO_EN|FLD_IRQ_GPIO_RISC0_EN|FLD_IRQ_GPIO_RISC1_EN|FLD_IRQ_GPIO_RISC2_EN;
	reg_irq_mask |= FLD_IRQ_GPIO_RISC0_EN;
}

/**
 * @brief     This function set a pin's IRQ.
 * @param[in] pin - the pin needs to enable its IRQ
 * @param[in] falling - value of the edge polarity(1: falling edge 0: rising edge)
 * @return    none
 */
static inline void gpio_set_interrupt_risc1(GPIO_PinTypeDef pin, GPIO_PolTypeDef falling)
{
	unsigned char bit = pin & 0xff;
	BM_SET(reg_gpio_irq_risc1_en(pin), bit);
	if(falling){
		BM_SET(reg_gpio_pol(pin), bit);
	}else{
		BM_CLR(reg_gpio_pol(pin), bit);
	}
/*clear gpio interrupt sorce (after setting gpio polarity,before enable interrupt)to avoid unexpected interrupt. confirm by minghai*/
	reg_irq_src |= FLD_IRQ_GPIO_EN|FLD_IRQ_GPIO_RISC0_EN|FLD_IRQ_GPIO_RISC1_EN|FLD_IRQ_GPIO_RISC2_EN;
	reg_irq_mask |= FLD_IRQ_GPIO_RISC1_EN;
}

/**
 * @brief     This function set a pin's IRQ.
 * @param[in] pin - the pin needs to enable its IRQ
 * @param[in] falling - value of the edge polarity(1: falling edge 0: rising edge)
 * @return    none
 */
static inline void gpio_set_interrupt_risc2(GPIO_PinTypeDef pin, GPIO_PolTypeDef falling)
{
	unsigned char bit = pin & 0xff;
	BM_SET(reg_gpio_irq_risc2_en(pin), bit);
	if(falling){
		BM_SET(reg_gpio_pol(pin), bit);
	}else{
		BM_CLR(reg_gpio_pol(pin), bit);
	}
/*clear gpio interrupt sorce (after setting gpio polarity,before enable interrupt)to avoid unexpected interrupt. confirm by minghai*/
	reg_irq_src |= FLD_IRQ_GPIO_EN|FLD_IRQ_GPIO_RISC0_EN|FLD_IRQ_GPIO_RISC1_EN|FLD_IRQ_GPIO_RISC2_EN;
	reg_irq_mask |= FLD_IRQ_GPIO_RISC1_EN;
}

#endif //GPIO_H


/** \defgroup GP5  GPIO Examples
 *
 * 	@{
 */

/*! \page gpio Table of Contents
	- [API-GPIO-CASE1:GPIO IRQ](#GPIO_IRQ)
	- [API-GPIO-CASE2:GPIO IRQ RSIC0](#GPIO_IRQ_RSIC0)
	- [API-GPIO-CASE3:GPIO IRQ RSIC1](#GPIO_IRQ_RSIC1)
	- [API-GPIO-CASE4:GPIO READ/WRITE/TOGGLE](#GPIO_TOGGLE)
	- [API-GPIO-CASE5:GPIO HIGH RESISTOR](#GPIO_HIGH_RESISTOR)

\n
Variables used in the following cases are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}

#define LED1     		        GPIO_PD0
#define LED2     		        GPIO_PD3
#define LED3     		        GPIO_PD4
#define LED4     		        GPIO_PD5

#define SW1      		        GPIO_PD1
#define SW2      		        GPIO_PD2

volatile unsigned int gpio_irq_cnt;

~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=GPIO_IRQ> API-GPIO-CASE1:GPIO IRQ </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | if((reg_irq_src & FLD_IRQ_GPIO_EN)==FLD_IRQ_GPIO_EN) ||| determine whether interrupt flag is right | 2019-1-10 |
| ^ | reg_irq_src &Iota;= FLD_IRQ_GPIO_EN ||| clear interrrupt flag | ^ |
| ^ | gpio_irq_cnt++  ||| Interrupt processing | ^ |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | gpio_set_func() | gpio_set_func(SW1 ,AS_GPIO) | set pin as GPIO | ^ |
| ^ | ^ | gpio_set_output_en() | gpio_set_output_en(SW1, 0) | disable GPIO output  | ^ |
| ^ | ^ | gpio_set_input_en() | gpio_set_input_en(SW1 ,1) | enable GPIO input | ^ |
| ^ | ^ | gpio_set_up_down_resistor() | gpio_set_up_down_resistor(SW1, PM_PIN_PULLUP_10K) | enable GPIO 10k resistor pull-up | ^ |
| ^ | ^ | gpio_set_interrupt() | gpio_set_interrupt(SW1, POL_FALLING) | set pin as GPIO interrupt  | ^ |
| ^ | main_loop() | None || Main program loop | ^ |

<h1 id=GPIO_IRQ_RSIC0> API-GPIO-CASE2:GPIO IRQ RSIC0 </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | if((reg_irq_src & FLD_IRQ_GPIO_RISC0_EN)==FLD_IRQ_GPIO_RISC0_EN) ||| determine whether interrupt flag is right | 2019-1-10 |
| ^ | reg_irq_src &Iota;= FLD_IRQ_GPIO_RISC0_EN ||| clear interrrupt flag | ^ |
| ^ | gpio_irq_cnt++  ||| Interrupt processing | ^ |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | gpio_set_func() | gpio_set_func(SW1 ,AS_GPIO) | set pin as GPIO | ^ |
| ^ | ^ | gpio_set_output_en() | gpio_set_output_en(SW1, 0) | disable GPIO output  | ^ |
| ^ | ^ | gpio_set_input_en() | gpio_set_input_en(SW1 ,1) | enable GPIO input | ^ |
| ^ | ^ | gpio_set_up_down_resistor() | gpio_set_up_down_resistor(SW1, PM_PIN_PULLUP_10K) | enable GPIO 10k resistor pull-up | ^ |
| ^ | ^ | gpio_set_interrupt_risc0() | gpio_set_interrupt_risc0(SW1, POL_FALLING) | set pin as GPIO interrupt risc0  | ^ |
| ^ | main_loop() | None || Main program loop | ^ |


<h1 id=GPIO_IRQ_RSIC1> API-GPIO-CASE3:GPIO IRQ RSIC1 </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | if((reg_irq_src & FLD_IRQ_GPIO_RISC1_EN)==FLD_IRQ_GPIO_RISC1_EN) ||| determine whether interrupt flag is right | 2019-1-10 |
| ^ | reg_irq_src &Iota;= FLD_IRQ_GPIO_RISC1_EN ||| clear interrrupt flag | ^ |
| ^ | gpio_irq_cnt++  ||| Interrupt processing | ^ |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | gpio_set_func() | gpio_set_func(SW1 ,AS_GPIO) | set pin as GPIO | ^ |
| ^ | ^ | gpio_set_output_en() | gpio_set_output_en(SW1, 0) | disable GPIO output  | ^ |
| ^ | ^ | gpio_set_input_en() | gpio_set_input_en(SW1 ,1) | enable GPIO input | ^ |
| ^ | ^ | gpio_set_up_down_resistor() | gpio_set_up_down_resistor(SW1, PM_PIN_PULLUP_10K) | enable GPIO 10k resistor pull-up | ^ |
| ^ | ^ | gpio_set_interrupt_risc1() | gpio_set_interrupt_risc1(SW1, POL_FALLING) | set pin as GPIO interrupt risc1 | ^ |
| ^ | main_loop() | None || Main program loop | ^ |


<h1 id=GPIO_TOGGLE> API-GPIO-CASE4:GPIO TOGGLE </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | gpio_set_func() |gpio_set_func(LED1 ,AS_GPIO) | set pin as GPIO | ^ |
| ^ | ^ | gpio_set_output_en() | gpio_set_output_en(LED1, 1) | disable GPIO output  | ^ |
| ^ | ^ | gpio_set_input_en() | gpio_set_input_en(LED1 ,0) | enable GPIO input | ^ |
| ^ | ^ | gpio_write(), gpio_read() | gpio_write(LED1, !gpio_read(LED1)) | toggle GPIO | ^ |
| ^ | ^ | gpio_toggle() | gpio_toggle(LED1) | toggle GPIO | ^ |
| ^ | main_loop() | None || Main program loop | ^ |

<h1 id=GPIO_HIGH_RESISTOR> API-GPIO-CASE5:GPIO HIGH RESISTOR </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | gpio_shutdown() | gpio_shutdown(GPIO_ALL) | set all GPIOs as high resistor | ^ |
| ^ | main_loop() | None || Main program loop | ^ |

<h1> History Record </h1>

| Date | Description | Author |
| :--- | :---------- | :----- |
| 2019-1-10 | initial release | LJW |


*/

 /** @}*/ //end of GP5




