/**
  ******************************************************************************
  * @file    stm32f1xx_it.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    11-April-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include "stm32f10x_i2c.h"
#include "DAP_config.h"
#include "ir_receiver.h"
#include "ir_transmitter.h"
#include "stm32f10x_adc.h"


/** @addtogroup Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint8_t I2C2_Buffer_Tx, I2C2_Buffer_Rx, CirReceiverData[], CirTransmitterData[];
extern __IO uint8_t CIR_Transmitter_Ready;
extern __IO uint8_t KeyPressed ;
extern __IO uint8_t BAT_Detect ;
extern __IO uint8_t Disable_StandbyMode;


__IO uint8_t GET_USER2_BUTTON = 0;
__IO uint8_t USER2_BUTTON_STATUS = 0xFF;
__IO uint8_t SEND_REC_FLAG = 0;
__IO uint8_t SEND_REC_DATA = 0;
__IO uint8_t GET_TRAN_DATA = 0;
__IO uint8_t GET_BAT_VOLT = 0;
__IO uint8_t cir_reciver_count =0;
__IO uint8_t cir_transmitter_count =0;
__IO uint8_t bat_transfer_count =0;
__IO uint32_t BAT_VOLT_ADC = 0;
__IO uint8_t BAT_VOLT_ADC_ARRAY[4];


/* Private function prototypes -----------------------------------------------*/
/* Private fun ctions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M Processor Exceptions Handlers                          */
/******************************************************************************/

/******************************************************************************/
/*                 STM32F1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_md.s).                                            */
/******************************************************************************/
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @brief  This function handles I2C2 Event interrupt request.
  * @param  None
  * @retval None
  */
void I2C2_EV_IRQHandler(void)
{
	I2C_ClearITPendingBit(I2C2, I2C_IT_AF);
  switch (I2C_GetLastEvent(I2C2))
  {
    /* Slave Transmitter ---------------------------------------------------*/
    case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED:  /* EV1 */		
      /* Transmit I2C2 data */
			//I2C_ClearITPendingBit(I2C2, I2C_IT_AF);
			if ( GET_USER2_BUTTON == 1)
				{
					I2C_SendData(I2C2, USER2_BUTTON_STATUS);
				}
			else if (SEND_REC_DATA == 1)
				{
					I2C_SendData(I2C2, CirReceiverData[cir_reciver_count++]);
				}
			else if (GET_BAT_VOLT == 1)
				{
					BAT_VOLT_ADC = ADC_GetConversionValue(ADC1);

					BAT_VOLT_ADC_ARRAY[0] = (BAT_VOLT_ADC >> 24);
					BAT_VOLT_ADC_ARRAY[1] = (BAT_VOLT_ADC >> 16);
					BAT_VOLT_ADC_ARRAY[2] = (BAT_VOLT_ADC >> 8);
					BAT_VOLT_ADC_ARRAY[3] = (BAT_VOLT_ADC);

					I2C_SendData(I2C2, BAT_VOLT_ADC_ARRAY[bat_transfer_count++]);
				}
			break;

    case I2C_EVENT_SLAVE_BYTE_TRANSMITTED:             /* EV3 */
      /* Transmit I2C2 data */
			//I2C_ClearITPendingBit(I2C2, I2C_IT_AF);
			if ( GET_USER2_BUTTON == 1)
				{
					I2C_SendData(I2C2, USER2_BUTTON_STATUS);
					GET_USER2_BUTTON = 0;
					USER2_BUTTON_STATUS = 0xFF;
				}
			else if (SEND_REC_DATA == 1)
				{
					int i;
					I2C_SendData(I2C2, CirReceiverData[cir_reciver_count++]);
					if (cir_reciver_count == 4)
						{
							cir_reciver_count = 0;
							SEND_REC_DATA = 0;
							for (i=0;i<4;i++)
								CirReceiverData[i] = 0;
						}
				}
			else if (GET_BAT_VOLT == 1)
				{
					int i;
					I2C_SendData(I2C2, BAT_VOLT_ADC_ARRAY[bat_transfer_count++]);
					if (bat_transfer_count == 4)
						{
							bat_transfer_count = 0;
							GET_BAT_VOLT = 0;
							for (i=0;i<4;i++)
								BAT_VOLT_ADC_ARRAY[i] = 0;
						}
				}
			break; 
  
    /* Slave Receiver ------------------------------------------------------*/
    case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED:     /* EV1 */
      break;

    case I2C_EVENT_SLAVE_BYTE_RECEIVED:                /* EV2 */	
			/* Store I2C2 received data */
			if(GET_TRAN_DATA == 0)
			{
				I2C2_Buffer_Rx = I2C_ReceiveData(I2C2);
				
			}
			else
			{
				CirTransmitterData[cir_transmitter_count++] = I2C_ReceiveData(I2C2);
				if (cir_transmitter_count == 4)
				{
					GET_TRAN_DATA = 0;
					cir_transmitter_count = 0;
					CIR_Transmitter_Ready = 1;
				}
				return;
			}
			switch(I2C2_Buffer_Rx)
				{
					case USER2_LED_H:
						LedUSER2On();
						break;
					case USER2_LED_L:
						LedUSER2Off();
						break;
					case POWER_LED_H:
						LedPowerOn();
						break;
					case POWER_LED_L:
						LedPowerOff();
						break;
					case BLE_LED_H:
						LedBLEOn();
						break;
					case BLE_LED_L:
						LedBLEOff();
						break;
					case GPIO_ESP_PW_H:
						PowerOn_ESP();
						break;
					case GPIO_ESP_PW_L:
						Poweroff_ESP();
						break;
					case GPIO_ESP_EN_H:
						Enable_ESP();
						break;
					case GPIO_ESP_EN_L:
						Disable_ESP();
						break;
					case BAT_VOLT_IN:
						Disable_StandbyMode = 1;
						break;
					case GET_BAT_VOLT_ADC:
						GET_BAT_VOLT = 1;
						break;
					case USER2_BUTTON_IN:
						GET_USER2_BUTTON = 1;
						if (USER2_PORT->IDR & (1 << 4))
							USER2_BUTTON_STATUS = 0xC3;
						else
							USER2_BUTTON_STATUS = 0x3C;
						break;
					case GPIO_USER1_IT:
						GPIO_USER1_BUTTON_SETUP();
						break;
					case CIR_ENABLE_RECIVER:
						enableIRIn();
						break;
					case CIR_DISABLE_RECIVER:
						disableIRIn();
						break;
					case CIR_RECEIVER:
						SEND_REC_DATA = 1;
						break;
					case CIR_ENABLE_TRANSMITTER:
						enableIROut(38);
						break;
					case CIR_DISABLE_TRANSMITTER:
						disableIROut();
						break;
					case CIR_TRANSMITTER:
						GET_TRAN_DATA = 1;
						break;
					default:
						break;
				}
				
			break;
				
    case I2C_EVENT_SLAVE_STOP_DETECTED:                /* EV4 */
      /* Clear I2C2 STOPF flag: read of I2C_SR1 followed by a write on I2C_CR1 */
		  I2C_SendData(I2C2, I2C2_Buffer_Tx);
      (void)(I2C_GetITStatus(I2C2, I2C_IT_STOPF));
      I2C_Cmd(I2C2, ENABLE);
      break;
   
		case I2C_EVENT_SLAVE_ACK_FAILURE:
			  /* Check on I2C2 AF flag and clear it */
			I2C_ClearITPendingBit(I2C2, I2C_IT_AF);
			break;
		
    default:
      break;
  }
}

void I2C2_ER_IRQHandler(void)
{
  /* Check on I2C2 AF flag and clear it */
  if (I2C_GetITStatus(I2C2, I2C_IT_AF)) 
  {
    I2C_ClearITPendingBit(I2C2, I2C_IT_AF);
  }
}

void EXTI15_10_IRQHandler(void)
{
  if ((EXTI_GetITStatus(EXTI_Line10) != RESET))
  {  
		//NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
		//NVIC_DisableIRQ(USART_IRQn);
		KeyPressed = 1;
    ///* Clear the USER Button EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line10);
		//SleepMode_Measure();
  }
	
	if ((EXTI_GetITStatus(EXTI_Line12) != RESET))
  {
		if ((GND_DETECT_PORT->IDR & (1 << 2)))
			{
				BAT_Detect = 1;
			}
    ///* Clear the USER Button EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line12);
  }
}

void EXTI2_IRQHandler(void)
{
	
  if ((EXTI_GetITStatus(EXTI_Line2) != RESET))
  {
		if ((GND_DETECT_PORT->IDR & (1 << 2)))
			Disable_External_SWD_Program();
		else
			Enable_External_SWD_Program();
		
    EXTI_ClearITPendingBit(EXTI_Line2);
  }
}

void TIM2_IRQHandler()
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
			{
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        // Timeout
        ir_nec_reset_transmission();
			}
}

void EXTI3_IRQHandler(void)
{
    static unsigned int counter;

    if (EXTI_GetITStatus(EXTI_Line3) != RESET)
    {
        u8 bit = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3);

        // Restart Timer
        counter = TIM_GetCounter(TIM2);
        TIM_SetCounter(TIM2, 0);
			
        ir_nec_state_machine(counter);

        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}

//Turn on or off ESP using USER2 Button
void EXTI4_IRQHandler(void)
{
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
