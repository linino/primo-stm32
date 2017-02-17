/**
 * @file    IO_Config.h
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __IO_CONFIG_H__
#define __IO_CONFIG_H__

#include "stm32f10x.h"
#include "compiler.h"
#include "daplink.h"

COMPILER_ASSERT(DAPLINK_HIC_ID == DAPLINK_HIC_ID_STM32F103XB);

//USB control pin
#define USB_CONNECT_PORT_ENABLE()    (RCC->APB2ENR |= RCC_APB2Periph_GPIOA)
#define USB_CONNECT_PORT_DISABLE()   (RCC->APB2ENR &= ~RCC_APB2Periph_GPIOA)
#define USB_CONNECT_PORT             GPIOA
#define USB_CONNECT_PIN              GPIO_Pin_15
#define USB_CONNECT_ON()             (USB_CONNECT_PORT->BRR  = USB_CONNECT_PIN)
#define USB_CONNECT_OFF()            (USB_CONNECT_PORT->BSRR = USB_CONNECT_PIN)


//When bootloader, disable the target port(not used)
#define POWER_EN_PIN_PORT            GPIOC
#define POWER_EN_PIN                 GPIO_Pin_6
#define POWER_EN_Bit                 6

#define nRESET_PIN_PORT              GPIOA
#define nRESET_PIN                   GPIO_Pin_6
#define nRESET_PIN_Bit               6

//SWDCLK
#define SWCLK_TCK_PIN_PORT           GPIOA
#define SWCLK_TCK_PIN                GPIO_Pin_4
#define SWCLK_TCK_PIN_Bit            4

//SWDIO/TMS
#define SWDIO_TMS_PIN_PORT					 GPIOA
#define SWDIO_TMS_PIN			  			 	 GPIO_Pin_5
#define SWDIO_TMS_PIN_Bit						 5

//LEDs
//Connected LED
#define CONNECTED_LED_PORT           GPIOC
#define CONNECTED_LED_PIN            GPIO_Pin_7
#define CONNECTED_LED_PIN_Bit        7

//USB status LED
#define RUNNING_LED_PORT             GPIOC
#define RUNNING_LED_PIN              GPIO_Pin_8
#define RUNNING_LED_Bit              8

#define PIN_HID_LED_PORT             GPIOC
#define PIN_HID_LED                  GPIO_Pin_6
#define PIN_HID_LED_Bit              6

#define PIN_CDC_LED_PORT             GPIOC
#define PIN_CDC_LED                  GPIO_Pin_6
#define PIN_CDC_LED_Bit              6

#define PIN_MSC_LED_PORT             GPIOC
#define PIN_MSC_LED                  GPIO_Pin_6
#define PIN_MSC_LED_Bit              6


#endif
