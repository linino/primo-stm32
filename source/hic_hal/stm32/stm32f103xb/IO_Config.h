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

// USB control pin
#define USB_CONNECT_PORT_ENABLE()    (RCC->APB2ENR |= RCC_APB2Periph_GPIOA)
#define USB_CONNECT_PORT_DISABLE()   (RCC->APB2ENR &= ~RCC_APB2Periph_GPIOA)
#define USB_CONNECT_PORT             GPIOA
#define USB_CONNECT_PIN              GPIO_Pin_15
#define USB_CONNECT_ON()             (USB_CONNECT_PORT->BRR  = USB_CONNECT_PIN)
#define USB_CONNECT_OFF()            (USB_CONNECT_PORT->BSRR = USB_CONNECT_PIN)

// When bootloader, disable the target port(not used)
#define POWER_EN_PORT		             GPIOC
#define POWER_EN_PIN                 GPIO_Pin_6
#define POWER_EN_Bit                 6

#define nRESET_PORT                  GPIOA
#define nRESET_PIN                   GPIO_Pin_6
#define nRESET_Bit               		 6

// SWDCLK
#define SWCLK_TCK_PORT               GPIOA
#define SWCLK_TCK_PIN                GPIO_Pin_4
#define SWCLK_TCK_Bit            		 4

// SWDIO/TMS
#define SWDIO_TMS_PORT    					 GPIOA
#define SWDIO_TMS_PIN			  			 	 GPIO_Pin_5
#define SWDIO_TMS_Bit						     5

// LEDs
// Connected LED
#define CONNECTED_LED_PORT           GPIOC
#define CONNECTED_LED_PIN            GPIO_Pin_7
#define CONNECTED_LED_Bit            7

// USB status LED
#define RUNNING_LED_PORT             GPIOC
#define RUNNING_LED_PIN		           GPIO_Pin_8
#define RUNNING_LED_Bit              8

#define HID_LED_PORT		             GPIOC
#define HID_LED_PIN                  GPIO_Pin_6
#define HID_LED_Bit     		         6

#define CDC_LED_PORT                 GPIOC
#define CDC_LED_PIN                  GPIO_Pin_6
#define CDC_LED_Bit                  6

#define MSC_LED_PORT                 GPIOC
#define MSC_LED_PIN                  GPIO_Pin_6
#define MSC_LED_Bit                  6

#define BLE_LED_PORT              	 GPIOA
#define BLE_LED_PIN                  GPIO_Pin_8
#define BLE_LED_Bit               	 8

#define USER2_LED_PORT               GPIOC
#define USER2_LED_PIN                GPIO_Pin_9
#define USER2_LED_Bit                9

// Pin configured as output to progrm external borard
#define SWD_PROG_PORT                GPIOC
#define SWD_PROG_PIN                 GPIO_Pin_11
#define SWD_PROG_Bit                 11

// Pin configured as output to control ESP 
#define ESP_PW_PORT               	 GPIOB
#define ESP_PW_PIN               		 GPIO_Pin_0
#define ESP_PW_Bit                   0

#define ESP_EN_PORT               	 GPIOB
#define ESP_EN_PIN               		 GPIO_Pin_1
#define ESP_EN_Bit                	 1

// Pin configured as input
#define USER1_PORT                	 GPIOC
#define USER1_PIN                    GPIO_Pin_10
#define USER1_Bit                 	 10

#define USER2_PORT                	 GPIOC
#define USER2_PIN                		 GPIO_Pin_4
#define USER2_Bit                 	 4

#define GND_DETECT_PORT              GPIOD
#define GND_DETECT_PIN               GPIO_Pin_2
#define GND_DETECT_Bit               2

#define VOL_DET_PORT                 GPIOA
#define VOL_DET_PIN                  GPIO_Pin_7
#define VOL_DET_Bit                  7

#define WKUP_PORT               	   GPIOA
#define WKUP_PIN                     GPIO_Pin_0
#define WKUP_Bit                		 0

#define BAT_DET_PORT                 GPIOC
#define BAT_DET_PIN                  GPIO_Pin_12
#define BAT_DET_Bit                  12

#define ESP_0_PORT                	 GPIOC
#define ESP_0_PIN                    GPIO_Pin_5
#define ESP_0_Bit                 	 5

#define ESP_4_PORT                   GPIOB
#define ESP_4_PIN                    GPIO_Pin_7
#define ESP_4_Bit                    7


#endif
