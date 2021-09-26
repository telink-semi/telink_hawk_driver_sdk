/********************************************************************************************************
 * @file	register_8232.h
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
#pragma once


#include "bsp.h"

/********************************************************************************************
 *****|~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|*****
 *****|								Digital  Register Table  						   |*****
 *****|~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|*****
 ********************************************************************************************/

/*******************************      i2c registers: 0x00      ******************************/

#define reg_i2c_speed			REG_ADDR8(0x00)
#define reg_i2c_id		        REG_ADDR8(0x01)//I2C Master device Id

enum{
	FLD_I2C_WRITE_READ_BIT  =  BIT(0),
	FLD_I2C_ID              =  BIT_RNG(1,7),
};

#define reg_i2c_status			REG_ADDR8(0x02)
enum{
	FLD_I2C_CMD_BUSY		= 	BIT(0),
	FLD_I2C_BUS_BUSY		= 	BIT(1),
	FLD_I2C_NAK				= 	BIT(2),
};

#define reg_i2c_mode			REG_ADDR8(0x03)
enum
{
	FLD_I2C_ADDR_AUTO_ADD   =	BIT(0),
	FLD_I2C_MASTER_EN		= 	BIT(1),//1:Enable master.
	FLD_I2C_SLAVE_MAPPING   =	BIT(2),//1:Mapping mode;0:DMA mode.
	FLD_I2C_HOLD_MASTER     =   BIT(3),
	FLD_I2C_SLAVE_EN        =   BIT(4),//1:Enable slave.
};

#define reg_i2c_adr_dat			REG_ADDR16(0x04)
#define reg_i2c_dat_ctrl		REG_ADDR32(0x04)
#define reg_i2c_di_ctrl			REG_ADDR16(0x06)
#define reg_i2c_adr				REG_ADDR8(0x04)
#define reg_i2c_do				REG_ADDR8(0x05)
#define reg_i2c_di				REG_ADDR8(0x06)
#define reg_i2c_ctrl			REG_ADDR8(0x07)
enum{
	FLD_I2C_CMD_ID			= 		BIT(0),
	FLD_I2C_CMD_ADDR		= 		BIT(1),
	FLD_I2C_CMD_DO			= 		BIT(2),
	FLD_I2C_CMD_DI			= 		BIT(3),
	FLD_I2C_CMD_START		= 		BIT(4),
	FLD_I2C_CMD_STOP		= 		BIT(5),
	FLD_I2C_CMD_READ_ID		= 		BIT(6),
	FLD_I2C_CMD_ACK			= 		BIT(7),
};

#define reg_i2c_slave_irq_status 	REG_ADDR8(0x21)
#define reg_spi_slave_irq_status 	REG_ADDR8(0x21)
enum{
	FLD_HOST_CMD_IRQ 	= 	BIT(0),
	FLD_HOST_READ_IRQ	= 	BIT(1),
	FLD_SPI_BYTE_IRQ 	= 	FLD_HOST_CMD_IRQ,
};

#define reg_i2c_slave_map_addr		REG_ADDR16(0x22)
#define reg_i2c_slave_id        	REG_ADDR8(0x28)//I2C Slave device Id

/*******************************      spi registers: 0x08      ******************************/

#define reg_spi_data			REG_ADDR8(0x08)
#define reg_spi_ctrl			REG_ADDR8(0x09)
enum
{
	FLD_SPI_CS           		= BIT(0),
	FLD_SPI_MASTER_MODE_EN    	= BIT(1),
	FLD_SPI_DATA_OUT_DIS 		= BIT(2),
	FLD_SPI_RD           		= BIT(3),
	FLD_SPI_ADDR_AUTO_EN 		= BIT(4),
	FLD_SPI_SHARE_MODE 			= BIT(5),
	FLD_SPI_SLAVE_EN     		= BIT(6),
	FLD_SPI_BUSY         		= BIT(7),
};
#define reg_spi_sp				REG_ADDR8(0x0a)
enum{
	FLD_MASTER_SPI_CLK = 		BIT_RNG(0,6),
	FLD_SPI_ENABLE =			BIT(7),
};

#define reg_spi_inv_clk			REG_ADDR8(0x0b)//spi supports four modes
enum {
	FLD_SPI_MODE_WORK_MODE = BIT_RNG(0,1),
};

/*******************************      mspi registers: 0x0c      ******************************/

#define reg_mspi_data		REG_ADDR8(0x0c)
#define reg_mspi_ctrl		REG_ADDR8(0x0d)

enum{
	FLD_MSPI_CS 	= 		BIT(0),
	FLD_MSPI_SDO 	= 		BIT(1),
	FLD_MSPI_CONT 	= 		BIT(2),
	FLD_MSPI_RD 	= 		BIT(3),
	FLD_MSPI_BUSY 	= 		BIT(4),
};

#define reg_mspi_mode      		REG_ADDR8(0x0f)
enum
{
	FLD_MSPI_DUAL_DATA_MODE_EN  = BIT(0),
	FLD_MSPI_DUAL_ADDR_MODE_EN  = BIT(1),
	FLD_MSPI_CLK_DIV     	 	= BIT_RNG(2,7),
};

/*******************************      aif registers: 0x30      ******************************/
#define reg_aif_l_chn_addr                REG_ADDR16(0x30)
#define reg_aif_l_chn_size                REG_ADDR8(0x32)
#define reg_aif_l_chn_ctrl                REG_ADDR8(0x33)
enum{
	FLD_L_CHANNEL_FIFO_EN   = BIT(0),
	FLD_L_CHANNEL_WPTR_EN   = BIT(1),
	FLD_L_CHANNEL_WPTR_CLR  = BIT(2),
};

#define reg_aif_m_chn_addr                REG_ADDR16(0x34)
#define reg_aif_m_chn_size                REG_ADDR8(0x36)
#define reg_aif_m_chn_ctrl                REG_ADDR8(0x37)
enum{
	FLD_M_CHANNEL_FIFO_EN   = BIT(0),
	FLD_M_CHANNEL_WPTR_EN   = BIT(1),
	FLD_M_CHANNEL_WPTR_CLR  = BIT(2),
	FLD_M_CHANNEL_MONO      = BIT(3),
};

#define reg_aif_l_chn_wptr                  REG_ADDR16(0x38)
#define reg_aif_m_chn_wptr                  REG_ADDR16(0x3a)

#define reg_aif_adc_ctrl                    REG_ADDR8(0x3d)
enum{
	FLD_USE_RAW_DATA        = BIT(2),
};






/*******************************      reset registers: 0x60      ******************************/

#define reg_rst0				REG_ADDR8(0x60)
enum{
	FLD_RST0_SPI = 				BIT(0),
	FLD_RST0_I2C = 				BIT(1),
//	FLD_RST0_RSVD,
//	FLD_RST0_RSVD,
	FLD_RST0_MCU = 				BIT(4),
//	FLD_RST0_RSVD,
	FLD_RST0_AIF = 				BIT(6),
	FLD_RST0_ZB = 				BIT(7),
};
#define reg_rst1				REG_ADDR8(0x61)
enum{
	FLD_RST1_SYS_TIMER = 		BIT(0),
	FLD_RST1_ALGM = 			BIT(1),
	FLD_RST1_DMA =				BIT(2),
	FLD_RST1_RS232 = 			BIT(3),
	FLD_RST1_PWM = 				BIT(4),
	FLD_RST1_AES = 				BIT(5),
//	FLD_RST0_RSVD,
	FLD_RST1_SWIRE = 			BIT(7),
};

#define reg_rst2				REG_ADDR8(0x62)
enum{
//	FLD_RST0_RSVD,
//	FLD_RST0_RSVD,
//	FLD_RST0_RSVD,
	FLD_RST2_ADC  =				BIT(3),
	FLD_RST2_MCIC = 			BIT(4),
	FLD_RST2_RESET_MCIC_EN =	BIT(5),
	FLD_RST2_MSPI =             BIT(6),
	FLD_RST2_ALG  =             BIT(7),
};


#define reg_clk_en0             REG_ADDR8(0x63)
enum
{
	FLD_CLK0_SPI_EN     = BIT(0),
	FLD_CLK0_I2C_EN     = BIT(1),
	FLD_CLK0_HOSTIRQ_EN = BIT(2),

	FLD_CLK0_MCU_EN     = BIT(4),
	FLD_CLK0_FPU_EN     = BIT(5),
	FLD_CLK0_AIF_EN     = BIT(6),
	FLD_CLK0_ZB_EN      = BIT(7),
};

#define reg_clk_en1				REG_ADDR8(0x64)
enum
{
	FLD_CLK1_SYS_TIMER_EN 		= BIT(0),
	FLD_CLK1_ALGM_EN          	= BIT(1),
	FLD_CLK1_DMA_EN           	= BIT(2),
	FLD_CLK1_RS232_EN          	= BIT(3),
	FLD_CLK1_PWM_EN           	= BIT(4),
	FLD_CLK1_AES_EN           	= BIT(5),
	FLD_CLK1_CAL_32K_EN 		= BIT(6),  // for system timer count 16 cycle of 32k, close it to save power
	FLD_CLK1_SWS_EN           	= BIT(7),
};

#define reg_clk_en2				REG_ADDR8(0x65)
enum{
	FLD_CLK2_32K_FOR_QDEC_EN    = BIT(0),
	FLD_CLK2_MCIC_EN         	= BIT(4),
	FLD_CLK2_QDEC_EN 			= BIT(5),
};

#define reg_clk_sel				REG_ADDR8(0x66)

enum{
	FLD_SCLK_DIV = 				BIT_RNG(0,4),
	FLD_SCLK_SEL =				BIT_RNG(5,6),
	FLD_SCLK_HS_SEL =			BIT(7),
};

#define reg_fhs_sel				REG_ADDR8(0x70)
enum{
	FLD_FHS_SEL = BIT(0),
};

#define reg_wakeup_en			REG_ADDR8(0x6e)
enum{
	FLD_WAKEUP_SRC_I2C 		= BIT(0),
	FLD_WAKEUP_SRC_SPI 		= BIT(1),
	FLD_WAKEUP_SRC_GPIO 	= BIT(3),
	FLD_WAKEUP_SRC_QDEC 	= BIT(4),
	FLD_WAKEUP_SRC_RST_SYS 	= BIT(7),
};

#define reg_pwdn_ctrl			REG_ADDR8(0x6f)
enum
{
	FLD_PWDN_CTRL_REBOOT = BIT(5),
	FLD_PWDN_CTRL_SLEEP  = BIT(7),
};

#define reg_mcu_wakeup_mask		REG_ADDR32(0x78)

/*******************************      uart registers: 0x90      ******************************/

#define reg_uart_data_buf0		REG_ADDR8(0x90)
#define reg_uart_data_buf1		REG_ADDR8(0x91)
#define reg_uart_data_buf2		REG_ADDR8(0x92)
#define reg_uart_data_buf3		REG_ADDR8(0x93)

#define reg_uart_data_buf(i)    REG_ADDR8(0x90 + (i))
#define reg_uart_clk_div		REG_ADDR16(0x94)
enum{
	FLD_UART_CLK_DIV = 			BIT_RNG(0,14),
	FLD_UART_CLK_DIV_EN = 		BIT(15)
};
#define reg_uart_ctrl0			REG_ADDR8(0x96)
enum{
	FLD_UART_BPWC = 			BIT_RNG(0,3),
	FLD_UART_RX_DMA_EN = 		BIT(4),
	FLD_UART_TX_DMA_EN =		BIT(5),
	FLD_UART_RX_IRQ_EN = 		BIT(6),
	FLD_UART_TX_IRQ_EN =		BIT(7),
};

#define reg_uart_ctrl1         		REG_ADDR8(0x97)
enum {
    FLD_UART_CTRL1_CTS_SELECT	   = BIT(0),
    FLD_UART_CTRL1_CTS_EN 		   = BIT(1),
    FLD_UART_CTRL1_PARITY_EN 	   = BIT(2),
    FLD_UART_CTRL1_PARITY_POLARITY = BIT(3),   //1:odd parity   0:even parity
    FLD_UART_CTRL1_STOP_BIT 	   = BIT_RNG(4,5),
    FLD_UART_CTRL1_TTL 			   = BIT(6),
    FLD_UART_CTRL1_LOOPBACK 	   = BIT(7),
};

#define reg_uart_ctrl2			REG_ADDR16(0x98)
enum {
    FLD_UART_CTRL2_RTS_TRIG_LVL   	 = BIT_RNG(0,3),
    FLD_UART_CTRL2_RTS_PARITY 		 = BIT(4),
    FLD_UART_CTRL2_RTS_MANUAL_VAL 	 = BIT(5),
    FLD_UART_CTRL2_RTS_MANUAL_EN 	 = BIT(6),
    FLD_UART_CTRL2_RTS_EN 			 = BIT(7),
	FLD_UART_CTRL3_RX_IRQ_TRIG_LEVEL = BIT_RNG(8,11),
	FLD_UART_CTRL3_TX_IRQ_TRIG_LEVEL = BIT_RNG(12,15),
};


#define reg_uart_ctrl3        	REG_ADDR8(0x99)
enum {
	FLD_UART_RX_IRQ_TRIG_LEV = BIT_RNG(0,3),
	FLD_UART_TX_IRQ_TRIG_LEV = BIT_RNG(4,7),
};

#define reg_uart_rx_timeout0	REG_ADDR8(0x9a)

enum{
	FLD_UART_TIMEOUT_BW = 		BIT_RNG(0,7),		//  timeout bit width
};

#define reg_uart_rx_timeout1    REG_ADDR8(0x9b)

enum{
	FLD_UART_TIMEOUT_MUL	 = 	BIT_RNG(0,1),
};

#define reg_uart_buf_cnt       REG_ADDR8(0x9c)
enum{
	FLD_UART_RX_BUF_CNT		=  BIT_RNG(0,3),
	FLD_UART_TX_BUF_CNT		=  BIT_RNG(4,7),
};

#define reg_uart_status0       REG_ADDR8(0x9d)
enum{
	FLD_UART_RBCNT 	     =  BIT_RNG(0,2),
	FLD_UART_IRQ_FLAG    =  BIT(3),
	FLD_UART_WBCNT 	     =  BIT_RNG(4,6),
	FLD_UART_CLEAR_RX_FLAG 	=  BIT(6),
	FLD_UART_RX_ERR_FLAG =  BIT(7),
};

#define reg_uart_status1       REG_ADDR8(0x9e)
enum{
	FLD_UART_TX_DONE   	  =  BIT(0),
	FLD_UART_TX_BUF_IRQ   =  BIT(1),
	FLD_UART_RX_DONE   	  =  BIT(2),
	FLD_UART_RX_BUF_IRQ   =  BIT(3),
};

#define reg_uart_state       REG_ADDR8(0x9f)
enum{
	FLD_UART_TSTATE_I 	     =  BIT_RNG(0,2),
	FLD_UART_RSTATE_I	     =  BIT_RNG(4,7),
};

/*******************************      swire registers: 0xb0      ******************************/

#define reg_swire_data			REG_ADDR8(0xb0)
#define reg_swire_ctrl1			REG_ADDR8(0xb1)
enum{
	FLD_SWIRE_WR  = 			BIT(0),
	FLD_SWIRE_RD  = 			BIT(1),
	FLD_SWIRE_CMD =				BIT(2),
	FLD_SWIRE_ERR_FLAG			= BIT(3),
	FLD_SWIRE_EOP     			= BIT(4),
	FLD_SWIRE_USB_DET =			BIT(6),
	FLD_SWIRE_USB_EN =			BIT(7),
};

#define reg_swire_clk_div		REG_ADDR8(0xb2)
enum
{
	FLD_SWIRE_CLK_DIV = BIT_RNG(0,6),
};

#define reg_swire_id      		REG_ADDR8(0xb3)
enum
{
	FLD_SWIRE_ID_SLAVE_ID      = BIT_RNG(0,6),
	FLD_SWIRE_ID_SLAVE_FIFO_EN = BIT(7),
};

/*******************************      analog control registers: 0xb8      ******************************/

#define reg_ana_ctrl32			REG_ADDR32(0xb8)
#define reg_ana_addr_data		REG_ADDR16(0xb8)
#define reg_ana_addr			REG_ADDR8(0xb8)
#define reg_ana_data			REG_ADDR8(0xb9)
#define reg_ana_ctrl			REG_ADDR8(0xba)

enum{
	FLD_ANA_BUSY  = 			BIT(0),
	FLD_ANA_RSV	=				BIT(4),
	FLD_ANA_RW  = 				BIT(5),
	FLD_ANA_START  = 			BIT(6),
	FLD_ANA_CYC  = 				BIT(7),
};


/*******************************      Baseband registers: 0x400      ******************************/

#define reg_rf_tx_mode1			REG_ADDR8(0x400)
#define reg_rf_tx_mode			REG_ADDR16(0x400)
enum{
	FLD_RF_TX_DMA_EN =			BIT(0),
	FLD_RF_TX_CRC_EN =			BIT(1),
	FLD_RF_TX_BANDWIDTH =		BIT_RNG(2,3),
	FLD_RF_TX_OUTPUT = 			BIT(4),
	FLD_RF_TX_TST_OUT =			BIT(5),
	FLD_RF_TX_TST_EN =			BIT(6),
	FLD_RF_TX_TST_MODE =		BIT(7),
	FLD_RF_TX_ZB_PN_EN =		BIT(8),
	FLD_RF_TX_ZB_FEC_EN =		BIT(9),
	FLD_RF_TX_ZB_INTL_EN =		BIT(10),	// interleaving
	FLD_RF_TX_1M2M_PN_EN =		BIT(11),
	FLD_RF_TX_1M2M_FEC_EN =		BIT(12),
	FLD_RF_TX_1M2M_INTL_EN =	BIT(13), 	// interleaving
};
#define reg_rf_tx_buf_sta		REG_ADDR32(0x41c)

#define reg_rf_rx_sense_thr		REG_ADDR8(0x422)
#define reg_rf_rx_auto			REG_ADDR8(0x426)
enum{
	FLD_RF_RX_IRR_GAIN =		BIT(0),
	FLD_RF_RX_IRR_PHASE =		BIT(1),
	FLD_RF_RX_DAC_I =			BIT(2),
	FLD_RF_RX_DAC_Q =			BIT(3),
	FLD_RF_RX_LNA_GAIN =		BIT(4),
	FLD_RF_RX_MIX2_GAIN =		BIT(5),
	FLD_RF_RX_PGA_GAIN =		BIT(6),
	FLD_RF_RX_CAL_EN =			BIT(7),
};
#define reg_rf_rx_sync			REG_ADDR8(0x427)
enum{
	FLD_RF_FREQ_COMP_EN =		BIT(0),
	FLD_RF_ADC_SYNC =			BIT(1),
	FLD_RF_ADC_INP_SIGNED =		BIT(2),
	FLD_RF_SWAP_ADC_IQ =		BIT(3),
	FLD_RF_NOTCH_FREQ_SEL =		BIT(4),
	FLD_RF_NOTCH_BAND_SEL = 	BIT(5),
	FLD_RF_NOTCH_EN = 			BIT(6),
	FLD_RF_DN_CONV_FREQ_SEL =	BIT(7),
};

#define reg_rf_rx_mode			REG_ADDR8(0x428)
enum{
	FLD_RF_RX_EN =				BIT(0),
	FLD_RF_RX_MODE_1M =			BIT(1),
	FLD_RF_RX_MODE_2M =			BIT(2),
	FLD_RF_RX_LOW_IF =			BIT(3),
	FLD_RF_RX_BYPASS_DCOC =		BIT(4),
	FLD_RF_RX_MAN_FINE_TUNE = 	BIT(5),
	FLD_RF_RX_SINGLE_CAL =		BIT(6),
	FLD_RF_RX_LOW_PASS_FILTER =	BIT(7),
};

#define reg_rf_rx_pilot			REG_ADDR8(0x42b)
enum{
	FLD_RF_PILOT_LEN =			BIT_RNG(0,3),
	FLD_RF_ZB_SFD_CHK =			BIT(4),
	FLD_RF_1M_SFD_CHK =			BIT(5),
	FLD_RF_2M_SFD_CHK = 		BIT(6),
	FLD_RF_ZB_OR_AUTO = 		BIT(7),
};

#define reg_rf_rx_chn_dc		REG_ADDR32(0x42c)
#define reg_rf_rx_q_chn_cal		REG_ADDR8(0x42f)
enum{
	FLD_RF_RX_DCQ_HIGH =		BIT_RNG(0,6),
	FLD_RF_RX_DCQ_CAL_START =	BIT(7),
};
#define reg_rf_rx_pel			REG_ADDR16(0x434)
#define reg_rf_rx_pel_gain		REG_ADDR32(0x434)
#define reg_rf_rx_rssi_offset	REG_ADDR8(0x439)

#define reg_rf_rx_hdx			REG_ADDR8(0x43b)
enum{
	FLD_RX_HEADER_LEN =			BIT_RNG(0,3),
	FLD_RT_TICK_LO_SEL = 		BIT(4),
	FLD_RT_TICK_HI_SEL = 		BIT(5),
	FLD_RT_TICK_FRAME = 		BIT(6),
	FLD_PKT_LEN_OUTP_EN = 		BIT(7),
};

#define reg_rf_rx_gctl			REG_ADDR8(0x43c)
enum{
	FLD_RX_GCTL_CIC_SAT_LO_EN =	BIT(0),
	FLD_RX_GCTL_CIC_SAT_HI_EN = BIT(1),
	FLD_RX_GCTL_AUTO_PWR =		BIT(2),
	FLD_RX_GCTL_ADC_RST_VAL =	BIT(4),
	FLD_RX_GCTL_ADC_RST_EN =	BIT(5),
	FLD_RX_GCTL_PWR_CHG_DET_S =	BIT(6),
	FLD_RX_GCTL_PWR_CHG_DET_N = BIT(7),
};
#define reg_rf_rx_peak			REG_ADDR8(0x43d)
enum{
	FLD_RX_PEAK_DET_SRC_EN =	BIT_RNG(0,2),
	FLD_TX_PEAK_DET_EN =		BIT(3),
	FLD_PEAK_DET_NUM =			BIT_RNG(4,5),
	FLD_PEAK_MAX_CNT_PRD =		BIT_RNG(6,7),
};

#define reg_rf_rx_status		REG_ADDR8(0x443)
enum{
	FLD_RF_RX_STATE =			BIT_RNG(0,3),
	FLD_RF_RX_STA_RSV = 		BIT_RNG(4,5),
	FLD_RF_RX_INTR = 			BIT(6),
	FLD_RF_TX_INTR =			BIT(7),
};

enum{
	FLD_RF_IRX_RX_TIMEOUT =		BIT(2),
	FLD_RF_IRX_CMD_DONE  =		BIT(5),
	FLD_RF_IRX_RETRY_HIT =		BIT(7),
};

enum{
	RF_RX_STA_IDLE = 0,
	RF_RX_STA_SET_GAIN = 1,
	RF_RX_STA_CIC_SETTLE = 2,
	RF_RX_STA_LPF_SETTLE = 3,
	RF_RX_STA_PE = 4,
	RF_RX_STA_SYN_START = 5,
	RF_RX_STA_GLOB_SYN = 6,
	RF_RX_STA_GLOB_LOCK = 7,
	RF_RX_STA_LOCAL_SYN = 8,
	RF_RX_STA_LOCAL_LOCK = 9,
	RF_RX_STA_ALIGN = 10,
	RF_RX_STA_ADJUST = 11,
	RF_RX_STA_DEMOD = 12,		// de modulation
	RF_RX_STA_FOOTER = 13,
};

#define reg_rx_rnd_mode			REG_ADDR8(0x447)
enum{
	FLD_RX_RND_SRC =			BIT(0),
	FLD_RX_RND_MANU_MODE =		BIT(1),
	FLD_RX_RND_AUTO_RD =		BIT(2),
	FLD_RX_RND_FREE_MODE =		BIT(3),
	FLD_RX_RND_CLK_DIV =		BIT_RNG(4,7),
};
#define reg_rnd_number			REG_ADDR16(0x448)

#define reg_bb_max_tick			REG_ADDR16(0x44c)
#define reg_rf_rtt				REG_ADDR32(0x454)
enum{
	FLD_RTT_CAL =				BIT_RNG(0,7),
	FLD_RTT_CYC1 =				BIT_RNG(8,15),
	FLD_RTT_LOCK =				BIT_RNG(16,23),
	FLD_RT_SD_DLY_40M =			BIT_RNG(24,27),
	FLD_RT_SD_DLY_BYPASS = 		BIT(28),
};

#define reg_rf_chn_rssi			REG_ADDR8(0x458)

#define reg_rf_rx_gain_agc(i)	REG_ADDR32(0x480+((i)<<2))

#define reg_rf_rx_dci			REG_ADDR8(0x4cb)	//  different from the document, why
#define reg_rf_rx_dcq			REG_ADDR8(0x4cf)	//  different from the document, why

#define reg_pll_rx_coarse_tune	REG_ADDR16(0x4d0)
#define reg_pll_rx_coarse_div	REG_ADDR8(0x4d2)
#define reg_pll_rx_fine_tune	REG_ADDR16(0x4d4)
#define reg_pll_rx_fine_div		REG_ADDR8(0x4d6)
#define reg_pll_tx_coarse_tune	REG_ADDR16(0x4d8)
#define reg_pll_tx_coarse_div	REG_ADDR8(0x4da)
#define reg_pll_tx_fine_tune	REG_ADDR16(0x4dc)
#define reg_pll_tx_fine_div		REG_ADDR8(0x4de)

#define reg_pll_rx_frac			REG_ADDR32(0x4e0)
#define reg_pll_tx_frac			REG_ADDR32(0x4e4)

#define reg_pll_tx_ctrl			REG_ADDR8(0x4e8)
#define reg_pll_ctrl16			REG_ADDR16(0x4e8)
#define reg_pll_ctrl			REG_ADDR32(0x4e8)
enum{
	FLD_PLL_TX_CYC0 =			BIT(0),
	FLD_PLL_TX_SOF =			BIT(1),
	FLD_PLL_TX_CYC1 =			BIT(2),
	FLD_PLL_TX_PRE_EN =			BIT(3),
	FLD_PLL_TX_VCO_EN =			BIT(4),
	FLD_PLL_TX_PWDN_DIV =		BIT(5),
	FLD_PLL_TX_MOD_EN =			BIT(6),
	FLD_PLL_TX_MOD_TRAN_EN =	BIT(7),
	FLD_PLL_RX_CYC0 =			BIT(8),
	FLD_PLL_RX_SOF = 			BIT(9),
	FLD_PLL_RX_CYC1 =			BIT(10),
	FLD_PLL_RX_PRES_EN = 		BIT(11),
	FLD_PLL_RX_VCO_EN =			BIT(12),
	FLD_PLL_RX_PWDN_DIV =		BIT(13),
	FLD_PLL_RX_PEAK_EN =		BIT(14),
	FLD_PLL_RX_TP_CYC = 		BIT(15),
	FLD_PLL_SD_RSTB =			BIT(16),
	FLD_PLL_SD_INTG_EN =		BIT(17),
	FLD_PLL_CP_TRI = 			BIT(18),
	FLD_PLL_PWDN_INTG1 = 		BIT(19),
	FLD_PLL_PWDN_INTG2 =		BIT(20),
	FLD_PLL_PWDN_INTG_DIV =		BIT(21),
	FLD_PLL_PEAK_DET_EN =		BIT(22),
	FLD_PLL_OPEN_LOOP_EN =		BIT(23),
	FLD_PLL_RX_TICK_EN =		BIT(24),
	FLD_PLL_TX_TICK_EN =		BIT(25),
	FLD_PLL_RX_ALWAYS_ON =		BIT(26),
	FLD_PLL_TX_ALWAYS_ON =		BIT(27),
	FLD_PLL_MANUAL_MODE_EN =	BIT(28),
	FLD_PLL_CAL_DONE_EN =		BIT(29),
	FLD_PLL_LOCK_EN =			BIT(30),
};
#define reg_pll_rx_ctrl			REG_ADDR8(0x4e9)
enum{
	FLD_PLL_RX2_CYC0 =			BIT(0),
	FLD_PLL_RX2_SOF = 			BIT(1),
	FLD_PLL_RX2_CYC1 =			BIT(2),
	FLD_PLL_RX2_PRES_EN = 		BIT(3),
	FLD_PLL_RX2_VCO_EN =		BIT(4),
	FLD_PLL_RX2_PD_DIV =		BIT(5),
	FLD_PLL_RX2_PEAK_EN =		BIT(6),
	FLD_PLL_RX2_TP_CYC = 		BIT(7),
};

#define reg_pll_ctrl_a			REG_ADDR8(0x4eb)
enum{
	FLD_PLL_A_RX_TICK_EN =		BIT(0),
	FLD_PLL_A_TX_TICK_EN =		BIT(1),
	FLD_PLL_A_RX_ALWAYS_ON =	BIT(2),
	FLD_PLL_A_TX_ALWAYS_ON =	BIT(3),
	FLD_PLL_A_MANUAL_MODE_EN =	BIT(4),
	FLD_PLL_A_CAL_DONE_EN =		BIT(5),
	FLD_PLL_A_LOCK_EN =			BIT(6),
};
// pll polarity
#define reg_pll_pol_ctrl		REG_ADDR16(0x4ec)
enum{
	FLD_PLL_POL_TX_PRE_EN =		BIT(0),
	FLD_PLL_POL_TX_VCO_EN =		BIT(1),
	FLD_PLL_POL_TX_PD_DIV =		BIT(2),
	FLD_PLL_POL_MOD_EN =		BIT(3),
	FLD_PLL_POL_MOD_TRAN_EN =	BIT(4),
	FLD_PLL_POL_RX_PRE_EN =		BIT(5),
	FLD_PLL_POL_RX_VCO_EN =		BIT(6),
	FLD_PLL_POL_RX_PD_DIV =		BIT(7),
	FLD_PLL_POL_SD_RSTB =		BIT(8),
	FLD_PLL_POL_SD_INTG_EN =	BIT(9),
	FLD_PLL_POL_CP_TRI =		BIT(10),
	FLD_PLL_POL_TX_SOF =		BIT(11),
	FLD_PLL_POL_RX_SOF =		BIT(12),
};

#define reg_rf_rx_cap			REG_ADDR16(0x4f0)
#define reg_rf_tx_cap			REG_ADDR16(0x4f0)


/*******************************      dma registers: 0x500      ******************************/

#define reg_dma0_addr			REG_ADDR16(0x500)
#define reg_dma0_size			REG_ADDR8 (0x502)
#define reg_dma0_mode			REG_ADDR8 (0x503)
enum{
	FLD_DMA_WR_MEM =			BIT(0),
	FLD_DMA_PINGPONG_EN =		BIT(1),
	FLD_DMA_FIFO_EN =			BIT(2),
	FLD_DMA_AUTO_MODE =			BIT(3),
	FLD_DMA_RSVD   =			BIT(4),
	FLD_DMA_BYTE_MODE =			BIT(5)
};


#define reg_dma1_addr			REG_ADDR16(0x504)
#define reg_dma1_size			REG_ADDR8(0x506)
#define reg_dma1_mode			REG_ADDR8(0x507)

#define reg_dma2_addr			REG_ADDR16(0x508)
#define reg_dma2_size			REG_ADDR8(0x50a)
#define reg_dma2_mode			REG_ADDR8(0x50b)

#define reg_dma3_addr			REG_ADDR16(0x50c)
#define reg_dma3_size			REG_ADDR8(0x50e)
#define reg_dma3_mode			REG_ADDR8(0x50f)

#define reg_dma5_addr			REG_ADDR16(0x514)
#define reg_dma5_size			REG_ADDR8(0x516)
#define reg_dma5_mode			REG_ADDR8(0x517)

#define reg_dma_size(v)			REG_ADDR8(0x502+4*v)

#define reg_dma_uart_rx_addr	reg_dma0_addr
#define reg_dma_uart_rx_size	reg_dma0_size
#define reg_dma_uart_rx_mode	reg_dma0_mode

#define reg_dma_uart_tx_addr	reg_dma1_addr
#define reg_dma_uart_tx_size	reg_dma1_size
#define reg_dma_uart_tx_mode	reg_dma1_mode

#define reg_dma_rf_rx_addr		reg_dma2_addr
#define reg_dma_rf_rx_size		reg_dma2_size
#define reg_dma_rf_rx_mode		reg_dma2_mode

#define reg_dma_rf_tx_addr		reg_dma3_addr
#define reg_dma_rf_tx_size		reg_dma3_size
#define reg_dma_rf_tx_mode		reg_dma3_mode

#define reg_dma_pwm_addr		reg_dma5_addr
#define reg_dma_pwm_size		reg_dma5_size
#define reg_dma_pwm_mode		reg_dma5_mode

#define reg_dma_chn_en			REG_ADDR8(0x520)
#define reg_dma_chn_irq_msk		REG_ADDR8(0x521)
#define reg_dma_tx_rdy0			REG_ADDR8(0x524)
#define reg_dma_tx_rdy1			REG_ADDR8(0x525)
#define reg_dma_rx_rdy0			REG_ADDR8(0x526)
#define reg_dma_rx_rdy1			REG_ADDR8(0x527)

#define reg_dma_irq_status      reg_dma_rx_rdy0

enum{
    FLD_DMA_CHN0 =	BIT(0),		FLD_DMA_CHN_UART_RX = BIT(0),
    FLD_DMA_CHN1 =	BIT(1),		FLD_DMA_CHN_UART_TX = BIT(1),
	FLD_DMA_CHN2 =	BIT(2),		FLD_DMA_CHN_RF_RX 	= BIT(2),
	FLD_DMA_CHN3 =	BIT(3),		FLD_DMA_CHN_RF_TX 	= BIT(3),
	FLD_DMA_CHN4 =	BIT(4),
	FLD_DMA_CHN5 =	BIT(5),		FLD_DMA_CHN_PWM 	= BIT(5),
	FLD_DMA_CHN6 =	BIT(6),
	FLD_DMA_CHN7 =	BIT(7),
};

typedef enum {
    FLD_DMA_IRQ_UART_RX  = BIT(0),
    FLD_DMA_IRQ_UART_TX  = BIT(1),
    FLD_DMA_IRQ_RF_RX    = BIT(2),
    FLD_DMA_IRQ_RF_TX    = BIT(3),
    FLD_DMA_IRQ_PWM		 = BIT(5),
    FLD_DMA_IRQ_ALL      = 0xff,
} IRQ_DMAIrqTypeDef;

#define reg_dma_rx_rptr			REG_ADDR8(0x528)
#define reg_dma_rx_wptr			REG_ADDR8(0x529)

#define reg_dma_tx_rptr			REG_ADDR8(0x52a)
#define reg_dma_tx_wptr			REG_ADDR8(0x52b)
#define reg_dma_tx_fifo			REG_ADDR16(0xc2c)

enum{
	FLD_DMA_RPTR_CLR =			BIT(4),
	FLD_DMA_RPTR_NEXT =			BIT(5),
	FLD_DMA_RPTR_SET =			BIT(6),
};

/*******************************      aes registers: 0x540      ******************************/

#define reg_aes_ctrl            REG_ADDR8(0x540)

enum {
    FLD_AES_CTRL_CODEC_TRIG = BIT(0),
    FLD_AES_CTRL_DATA_FEED = BIT(1),
    FLD_AES_CTRL_CODEC_FINISHED = BIT(2),
};

#define reg_aes_data            REG_ADDR32(0x548)
#define reg_aes_key(v)     		REG_ADDR8(0x550+v)

/*******************************      gpio registers: 0x580      ******************************/

#define reg_gpio_pa_in			REG_ADDR8(0x580)
#define reg_gpio_pa_ie			REG_ADDR8(0x581)
#define areg_gpio_pa5_6_7_ie	0xb6
#define reg_gpio_pa_oen			REG_ADDR8(0x582)
#define reg_gpio_pa_out			REG_ADDR8(0x583)
#define reg_gpio_pa_pol			REG_ADDR8(0x584)
#define reg_gpio_pa_ds			REG_ADDR8(0x585)
#define reg_gpio_pa5_6_7_ds		0xb8
#define reg_gpio_pa_gpio		REG_ADDR8(0x586)
#define reg_gpio_pa_irq_en		REG_ADDR8(0x587)

#define reg_gpio_pb_in			REG_ADDR8(0x588)
#define areg_gpio_pb_ie			0xb9
#define reg_gpio_pb_oen			REG_ADDR8(0x58a)
#define reg_gpio_pb_out			REG_ADDR8(0x58b)
#define reg_gpio_pb_pol			REG_ADDR8(0x58c)
#define areg_gpio_pb_ds			0xbb
#define reg_gpio_pb_gpio		REG_ADDR8(0x58e)
#define reg_gpio_pb_irq_en		REG_ADDR8(0x58f)

#define reg_gpio_pc_in			REG_ADDR8(0x590)
#define reg_gpio_pc_ie			REG_ADDR8(0x591)
#define reg_gpio_pc_oen			REG_ADDR8(0x592)
#define reg_gpio_pc_out			REG_ADDR8(0x593)
#define reg_gpio_pc_pol			REG_ADDR8(0x594)
#define reg_gpio_pc_ds			REG_ADDR8(0x595)
#define reg_gpio_pc_gpio		REG_ADDR8(0x596)
#define reg_gpio_pc_irq_en		REG_ADDR8(0x597)

#define reg_gpio_pe_in			REG_ADDR8(0x5a0)
#define reg_gpio_pe_ie			REG_ADDR8(0x5a1)
#define reg_gpio_pe_oen			REG_ADDR8(0x5a2)
#define reg_gpio_pe_out			REG_ADDR8(0x5a3)
#define reg_gpio_pe_pol			REG_ADDR8(0x5a4)
#define reg_gpio_pe_ds			REG_ADDR8(0x5a5)
#define reg_gpio_pe_gpio		REG_ADDR8(0x5a6)
#define reg_gpio_pe_irq_en		REG_ADDR8(0x5a7)

#define reg_gpio_pc_setting1	REG_ADDR32(0x590)
#define reg_gpio_pc_setting2	REG_ADDR32(0x594)

#define reg_gpio_in(i)			REG_ADDR8(0x580+((i>>8)<<3))
#define reg_gpio_ie(i)			REG_ADDR8(0x581+((i>>8)<<3))
#define reg_gpio_oen(i)			REG_ADDR8(0x582+((i>>8)<<3))
#define reg_gpio_out(i)			REG_ADDR8(0x583+((i>>8)<<3))
#define reg_gpio_pol(i)			REG_ADDR8(0x584+((i>>8)<<3))
#define reg_gpio_ds(i)			REG_ADDR8(0x585+((i>>8)<<3))

#define reg_gpio_gpio_func(i)		REG_ADDR8(0x586+((i>>8)<<3))
#define reg_gpio_irq_wakeup_en(i)	REG_ADDR8(0x587+((i>>8)<<3))  // reg_irq_mask: FLD_IRQ_GPIO_EN
#define reg_gpio_irq_risc0_en(i)    REG_ADDR8(0x5b8 + (i >> 8))	  // reg_irq_mask: FLD_IRQ_GPIO_RISC0_EN
#define reg_gpio_irq_risc1_en(i)    REG_ADDR8(0x5c0 + (i >> 8))	  // reg_irq_mask: FLD_IRQ_GPIO_RISC1_EN
#define reg_gpio_irq_risc2_en(i)    REG_ADDR8(0x5c8 + (i >> 8))   // reg_irq_mask: FLD_IRQ_GPIO_RISC2_EN

#define reg_mux_func_a1      		REG_ADDR8(0x5a8)
#define reg_mux_func_a2      		REG_ADDR8(0x5a9)
#define reg_mux_func_b1     		REG_ADDR8(0x5aa)
#define reg_mux_func_b2     		REG_ADDR8(0x5ab)
#define reg_mux_func_c1     		REG_ADDR8(0x5ac)
#define reg_mux_func_c2     		REG_ADDR8(0x5ad)

#define reg_gpio_wakeup_irq			REG_ADDR8(0x5b5)
enum{
    FLD_GPIO_CORE_WAKEUP_EN  	= BIT(2),
    FLD_GPIO_CORE_INTERRUPT_EN 	= BIT(3),
};

/*******************************      timer registers: 0x620      ******************************/

#define reg_tmr_ctrl			REG_ADDR32(0x620)
#define reg_tmr_ctrl16			REG_ADDR16(0x620)
#define reg_tmr_ctrl8			REG_ADDR8(0x620)
enum{
	FLD_TMR0_EN =				BIT(0),
	FLD_TMR0_MODE =				BIT_RNG(1,2),
	FLD_TMR1_EN = 				BIT(3),
	FLD_TMR1_MODE =				BIT_RNG(4,5),
	FLD_TMR2_EN =				BIT(6),
	FLD_TMR2_MODE = 			BIT_RNG(7,8),
	FLD_TMR_WD_CAPT = 			BIT_RNG(9,22),
	FLD_TMR_WD_EN =				BIT(23),
	FLD_TMR0_STA =				BIT(24),
	FLD_TMR1_STA =				BIT(25),
	FLD_TMR2_STA =				BIT(26),
	FLD_CLR_WD =				BIT(27),
};

#define reg_tmr_sta				REG_ADDR8(0x623)
enum{
	FLD_TMR_STA_TMR0 =			BIT(0),
	FLD_TMR_STA_TMR1 =			BIT(1),
	FLD_TMR_STA_TMR2 =			BIT(2),
	FLD_TMR_STA_WD =			BIT(3),
};

#define reg_tmr0_capt			REG_ADDR32(0x624)
#define reg_tmr1_capt			REG_ADDR32(0x628)
#define reg_tmr2_capt			REG_ADDR32(0x62c)
#define reg_tmr_capt(i)			REG_ADDR32(0x624 + ((i) << 2))
#define reg_tmr0_tick			REG_ADDR32(0x630)
#define reg_tmr1_tick			REG_ADDR32(0x634)
#define reg_tmr2_tick			REG_ADDR32(0x638)
#define reg_tmr_tick(i)			REG_ADDR32(0x630 + ((i) << 2))

/*******************************      irq registers: 0x640      ******************************/

#define reg_irq_mask			REG_ADDR32(0x640)
#define reg_irq_pri				REG_ADDR32(0x644)
#define reg_irq_src				REG_ADDR32(0x648)
#define reg_irq_src3			REG_ADDR8(0x64a)

enum
{
	FLD_IRQ_TMR0_EN       =	BIT(0),
	FLD_IRQ_TMR1_EN       =	BIT(1),
	FLD_IRQ_TMR2_EN       =	BIT(2),
//  FLD_IRQ_RSVD
	FLD_IRQ_DMA_EN        =	BIT(4),
//  FLD_IRQ_RSVD
	FLD_IRQ_UART_EN 	  =	BIT(6),
	FLD_IRQ_MIX_CMD_EN   =	BIT(7),

//  FLD_IRQ_RSVD
//  FLD_IRQ_RSVD
//  FLD_IRQ_RSVD
//  FLD_IRQ_RSVD
	FLD_IRQ_SOFT_IRQ_EN   =	BIT(12),
	FLD_IRQ_ZB_RT_EN      =	BIT(13),
	FLD_IRQ_SW_PWM_EN     =	BIT(14),
	FLD_IRQ_AN_EN         =	BIT(15),

//  FLD_IRQ_RSVD
//  FLD_IRQ_RSVD
	FLD_IRQ_GPIO_EN       =	BIT(18),
	FLD_IRQ_PM_EN         =	BIT(19),
//  FLD_IRQ_RSVD
	FLD_IRQ_GPIO_RISC0_EN =	BIT(21),
	FLD_IRQ_GPIO_RISC1_EN =	BIT(22),
	FLD_IRQ_GPIO_RISC2_EN = BIT(23),

};

#define reg_irq_en				REG_ADDR8(0x643)


/*******************************      system timer registers: 0x740      ******************************/

#define reg_system_tick			          	REG_ADDR32(0x740)

#define reg_system_tick_irq       			REG_ADDR32(0x744)
#define reg_32k_timer_cal_tick   			REG_ADDR16(0x748)
#define reg_sys_timer_ctrl                	REG_ADDR8(0x74a)

enum
{
	FLD_SYSTEM_WRITE_32K_EN	=		BIT(0),
	FLD_SYSTEM_AUTO_EN		=		BIT(2),
	FLD_SYSTEM_32K_CAL_EN	=		BIT(3),
	FLD_SYSTEM_32K_CAL_MODE	=		BIT_RNG(4,5),
	FLD_SYSTEM_TICK_IRQ_EN	=		BIT(6),
	FLD_SYSTEM_TICK_START	=		BIT(7),
};

#define reg_sys_timer_cmd_state   			REG_ADDR8(0x74b)
#define reg_32k_timer_tick        			REG_ADDR32(0x74c)

/*******************************      pwm registers: 0x780      ******************************/

#define reg_pwm_enable			REG_ADDR8(0x780)
#define reg_pwm_clk				REG_ADDR8(0x781)
#define reg_pwm0_mode			REG_ADDR8(0x782)
enum
{
	FLD_PWM0_MODE = BIT_RNG(0,3),
};

typedef enum{
	PWM_MODE_NORMAL      = 0x00,
	PWM_MODE_COUNT       = 0x01,
	PWM_MODE_IR          = 0x03,
	PWM_MODE_IR_FIFO     = 0x07,
	PWM_MODE_IR_DMA_FIFO = 0x0f,
}PWM_ModeTypeDef;

#define reg_pwm_invert			REG_ADDR8(0x783)
#define reg_pwm_n_invert		REG_ADDR8(0x784)
#define reg_pwm_pol				REG_ADDR8(0x785)

#define reg_pwm_cycle(i)		REG_ADDR32(0x794 + (i << 2))
#define reg_pwm_cmp(i)			REG_ADDR16(0x794 + (i << 2))
#define reg_pwm_max(i)			REG_ADDR16(0x796 + (i << 2))
enum{
	FLD_PWM_CMP  = BIT_RNG(0,15),
	FLD_PWM_MAX  = BIT_RNG(16,31),
};

#define reg_pwm0_pulse_num				REG_ADDR16(0x7ac)
#define reg_pwm_irq_mask(i)				REG_ADDR8(0x7b0+i*2)
#define reg_pwm_irq_sta(i)				REG_ADDR8(0x7b1+i*2)
enum{
	FLD_IRQ_PWM0_PNUM =					BIT(0),
	FLD_IRQ_PWM0_IR_DMA_FIFO_DONE  =	BIT(1),
	FLD_IRQ_PWM0_FRAME =				BIT(2),
	FLD_IRQ_PWM1_FRAME =				BIT(3),
	FLD_IRQ_PWM2_FRAME =				BIT(4),
	FLD_IRQ_PWM0_IR_FIFO =              BIT(16),
};


#define reg_pwm_tcmp0_shadow			REG_ADDR16(0x7c4)
#define reg_pwm_tmax0_shadow			REG_ADDR16(0x7c6)
#define reg_pwm_ir_fifo_dat(i)			REG_ADDR16(0x7c8+i*2)
#define reg_pwm_ir_fifo_irq_trig_level	REG_ADDR8(0x7cc)

#define reg_pwm_ir_fifo_data_status		REG_ADDR8(0x7cd)

enum{
	FLD_PWM0_IR_FIFO_DATA_NUM 	=		BIT_RNG(0,3),
	FLD_PWM0_IR_FIFO_EMPTY 		=		BIT(4),
	FLD_PWM0_IR_FIFO_FULL 		=		BIT(5),
};

#define reg_pwm_ir_clr_fifo_data		REG_ADDR8(0x7ce)

enum{
	FLD_PWM0_IR_FIFO_CLR_DATA 	=		BIT(0),
};


/*******************************      linklayer registers: 0xf00      ******************************/

#define reg_rf_ll_ctrl_0		REG_ADDR8(0xf02)
#define reg_rf_ll_ctrl_1		REG_ADDR8(0xf03)
#define reg_rf_ll_ctrl_2		REG_ADDR8(0xf15)

#define reg_rf_ll_ctrl_3		REG_ADDR8(0xf16)
enum{
	FLD_RF_TX_EN_DLY_EN = 		BIT(0),
	FLD_RF_PLL_RST_EN	=       BIT(1),
	FLD_RF_CMD_SCHEDULE_EN	=   BIT(2),
	FLD_RF_R_TX_EN_DLY =		BIT_RNG(4,7),
};

#define reg_rf_irq_mask			REG_ADDR16(0xf1c)
#define reg_rf_irq_status		REG_ADDR16(0xf20)
#define reg_rf_fsm_timeout		REG_ADDR32(0xf2c)

enum{
	FLD_RF_IRQ_RX = 			BIT(0),
	FLD_RF_IRQ_TX =				BIT(1),
	FLD_RF_IRQ_TX2RX_RX_TIMEOUT =		BIT(2),
	FLD_RF_IRQ_CRC =			BIT(4),
	FLD_RF_IRQ_CMD_DONE  =		BIT(5),
	FLD_RF_IRQ_FSM_TIMEOUT  =	BIT(6),
	FLD_RF_IRQ_RETRY_HIT =		BIT(7),
	FLD_RF_IRQ_TX_DS =          BIT(8),
    FLD_RF_IRQ_RX_DR =          BIT(9),
	FLD_RF_IRQ_RX2TX_RX_TIMEOUT =	BIT(10),
	FLD_RF_IRQ_INVALID_PID =    BIT(11),
	FLD_RF_IRQ_STX_TIMEOUT =    BIT(12),
	FLD_RF_IRQ_ALL =            0x1FFF,
};


/********************************************************************************************
 *****|~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|*****
 *****|								Aanlog  Register Table  						   |*****
 *****|~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|*****
 ********************************************************************************************/

/*******************************      analog registers(1v8): 0x80      ***************************/
/*******************************      analog registers(1v8): 0x80      ***************************/
#define anareg_80					0x80
enum{

	FLD_RESERVED                  = BIT_RNG(0,1),
	FLD_16M_CRYSTAL_POWERDOWN     = BIT(2),
	FLD_16M2RFPLL_POWERDOWN       = BIT(3),
	FLD_16M2DIGITAL_POWERDOWN     = BIT(4),
	FLD_XTL16M_BUFFER_CURRENT_CTL = BIT(5),
	FLD_16M_CRYSTAL_CAPACITOR_SEL = BIT(6),
	FLD_CLK_24M_TO_SAR_EN 		  = BIT(7),
};
#define anareg_e7					0xe7
enum{
	FLD_ADC_VREF_CHN_L = 		BIT_RNG(0,1),
	FLD_ADC_VREF_CHN_R = 		BIT_RNG(2,3),
	FLD_ADC_VREF_CHN_M = 		BIT_RNG(4,5),
};
#define anareg_e8					0xe8
#define anareg_e9					0xe9
#define anareg_ea					0xea

enum{
	FLD_ADC_AIN_NEGATIVE = 		BIT_RNG(0,3),
	FLD_ADC_AIN_POSITIVE = 		BIT_RNG(4,7),
};
#define anareg_eb					0xeb
enum{
	FLD_ADC_RES_L = 			BIT_RNG(0,1),
	FLD_ADC_RES_R = 			BIT_RNG(4,5),
};
#define anareg_ec					0xec
enum{
	FLD_ADC_RES_M = 			BIT_RNG(0,1),
	FLD_ADC_EN_DIFF_CHN_L =  	BIT(4),
	FLD_ADC_EN_DIFF_CHN_R =  	BIT(5),
	FLD_ADC_EN_DIFF_CHN_M =  	BIT(6),
};
#define anareg_ed					0xed
enum{
	FLD_ADC_TSAMPLE_CYCLE_CHN_L = 		BIT_RNG(0,3),
	FLD_ADC_TSAMPLE_CYCLE_CHN_R = 		BIT_RNG(4,7),
};
#define anareg_ee					0xee
enum{
	FLD_ADC_TSAMPLE_CYCLE_CHN_M = 		BIT_RNG(0,3),
};

#define anareg_ef	        		0xef
enum{
	FLD_R_MAX_MC0	= 			BIT_RNG(0,7),
};

#define anareg_f0       			0xf0
enum{
	FLD_R_MAX_C0	= 			BIT_RNG(0,7),
};

#define	anareg_f1			        0xf1
enum{
	FLD_R_MAX_S		=			BIT_RNG(0,3),
	FLD_R_MAX_C1	= 			BIT_RNG(4,5),
	FLD_R_MAX_MC1	= 			BIT_RNG(6,7),
};
#define anareg_f2					0xf2
enum{
	FLD_ADC_CHN_EN_L	= 		BIT(0),
	FLD_ADC_CHN_EN_R	= 		BIT(1),
	FLD_ADC_CHN_EN_M	= 		BIT(2),
	FLD_ADC_CHN_EN_RNS	= 		BIT(3),
	FLD_ADC_MAX_SCNT	=		BIT_RNG(4,6),
};

#define anareg_f4					0xf4
enum{
	FLD_ADC_SAMPLE_CLK_DIV  = 	BIT_RNG(0,2),
};
#define    anareg_f5        		0xf5
#define    anareg_f6        		0xf6
#define    anareg_f7        		0xf7
#define    anareg_f8        		0xf8

#define anareg_f9					0xf9
enum{
	FLD_ADC_VREF_VBAT_DIV   = 	BIT_RNG(2,3),
};
#define    anareg_fa        		0xfa
enum{
	FLD_ADC_ITRIM_PREAMP 	= 	BIT_RNG(0,1),
	FLD_ADC_ITRIM_VREFBUF	= 	BIT_RNG(2,3),
	FLD_ADC_ITRIM_VCMBUF	= 	BIT_RNG(4,5),
	FLD_SEL_AIN_SCALE 		= 	BIT_RNG(6, 7),
};

#define anareg_fb					0xfb
enum{
	FLD_PGA_CAP_TRIM_EN_L	  = BIT(0),
	//FLD_PGA_CAP_TRIM_EN_R	  = BIT(1),
	FLD_PGA_ITRIM_BOOST_L	  = BIT_RNG(4,5),
	//FLD_PGA_ITRIM_BOOST_R	  = BIT_RNG(6,7),
};

#define anareg_fc					0xfc
enum{
	FLD_PGA_ITRIM_GAIN_L		= BIT_RNG(0,1),
//	FLD_PGA_ITRIM_GAIN_R		= BIT_RNG(2,3),
	FLD_ADC_MODE				= BIT(4),
	FLD_SAR_ADC_POWER_DOWN 		= BIT(5),
	FLD_POWER_DOWN_PGA_CHN_L 	= BIT(6),
	//	FLD_POWER_DOWN_PGA_CHN_R 	= BIT(7),
};
#define anareg_fe					0xfe

#if defined(__cplusplus)
}
#endif




