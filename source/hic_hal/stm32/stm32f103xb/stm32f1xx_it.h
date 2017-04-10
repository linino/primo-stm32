/**
  ******************************************************************************
  * @file    stm32f1xx_it.h 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    11-April-2014
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F1XX_IT_H
#define __STM32F1XX_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 
	 
#define USER2_LED_H 		0xE2
#define USER2_LED_L 		0xE3
#define POWER_LED_H 		0xE4
#define POWER_LED_L 		0xE5
#define BLE_LED_H 			0xE6
#define BLE_LED_L 			0xE7
#define GPIO_ESP_PW_H 	0xE8
#define GPIO_ESP_PW_L 	0xE9
#define GPIO_ESP_EN_H 	0xEA
#define GPIO_ESP_EN_L 	0xEB
#define BAT_VOLT_IN 		0xEC
#define USER2_BUTTON_IN 0xEE
#define GPIO_USER1_IT 	0xF8

#define CIR_ENABLE_RECIVER	  		0xA0
#define CIR_DISABLE_RECIVER 			0xA1
#define CIR_RECEIVER 							0xA2
#define CIR_ENABLE_TRANSMITTER 		0xA3
#define CIR_DISABLE_TRANSMITTER		0xA4
#define CIR_TRANSMITTER						0xA5


/* Includes ------------------------------------------------------------------*/  
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#ifdef __cplusplus
}
#endif

#endif /* __STM32F1XX_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
