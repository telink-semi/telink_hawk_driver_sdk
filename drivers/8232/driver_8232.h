/********************************************************************************************************
 * @file	driver_8232.h
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



#include "drivers/8232/bsp.h"
#include "drivers/8232/aes.h"
#include "drivers/8232/analog.h"
#include "drivers/8232/compiler.h"
#include "drivers/8232/register.h"
#include "drivers/8232/gpio.h"
#include "drivers/8232/pwm.h"
#include "drivers/8232/irq.h"
#include "drivers/8232/clock.h"
#include "drivers/8232/random.h"
#include "drivers/8232/flash.h"
#include "drivers/8232/rf_drv.h"
#include "drivers/8232/pm.h"
#include "drivers/8232/adc.h"
#include "drivers/8232/i2c.h"
#include "drivers/8232/spi.h"
#include "drivers/8232/uart.h"
#include "drivers/8232/register.h"
#include "drivers/8232/watchdog.h"
#include "drivers/8232/register.h"
#include "drivers/8232/qdec.h"
#include "drivers/8232/dma.h"
#include "drivers/8232/timer.h"
#include "drivers/8232/emi.h"

#include "drivers/8232/flash/flash_type.h"

/*! \mainpage API User guide for TLSR8258F512
\n
__Keyword:__ \n
Bluetooth Low Energy;BLE Mesh;6LoWPAN;Thread;Zigbee;RF4CE;HomeKit;2.4GHz;Features;Package;Pin layout;Memory;MCU;Working modes; Wakeup sources;RF Transceiver;Clock;Timers; Interrupt;PWM;Audio;QDEC;ADC;PGA;Temperature sensor;Low power comparator; AES; Electrical specification; \n
\n
__Brief:__\n
This manual is dedicated for Telink BLE + IEEE802.15.4 multi-standard SoC TLSR8258F512. In this manual,key features, working mode,main modules, electrical specification and application of the TLSR8258F512 are introduced. \n
\n
__Published by__ \n
__Telink Semiconductor__ \n
\n
__Bldg 3, 1500 Zuchongzhi Rd,__ \n
__Zhangjiang Hi-Tech Park, Shanghai, China__ \n
\n
__Telink Semiconductor__ \n
__All Right Reserved__ \n
\n
__Legal Disclaimer:__ \n
    Telink Semiconductor reserves the right to make changes without further notice to any products herein to improve reliability, function or design. Telink Semiconductor disclaims any and all liability for any errors, inaccuracies or incompleteness contained herein or in any other disclosure relating to any product.\n
    Telink Semiconductor does not assume any liability arising out of the application or use of any product or circuit described herein; neither does it convey any license under its patent rights, nor the rights of others \n
    This document is provided as-is. Telink Semiconductor reserves the right to make improvements without further notice to this document or any products herein. This document may contain technical inaccuracies or typographical errors. Telink Semiconductor disclaims any and all liability for any errors, inaccuracies or incompleteness contained herein.
	\n
	Copyright (c) 2018 Telink Semicondcutor (Shanghai) Ltd, Co.\n
	\n
__Information:__ \n
For further information on the technology, product and business term, please contact [Telink Semiconductor Company](http://www.telink-semi.com "Telink") \n
For sales or technical support, please send email to the address of:\n
<telinkcnsales@telink-semi.com> \n
<telinkcnsupport@telink-semi.com> \n
For more details about this SoC, please look through [Datasheet for Telink BLE+IEEE802.15.4 Multi-Standard Wireless SoC TLSR8258](http://wiki.telink-semi.cn/doc/ds/DS_TLSR8258-E_Datasheet%20for%20Telink%20BLE%20IEEE802.15.4%20Multi-Standard%20Wireless%20SoC%20TLSR8258.pdf) \n
\n
__Revision History:__ \n

| Version | Major Changes   | Date         | Author |
| :-----: | :-------------: | :----------: | :----: |
| 1.0.0   | initial release | Dec. 11 2018 | LJW/ZJY/ZL/LR/SP/YY |

\n

*/
