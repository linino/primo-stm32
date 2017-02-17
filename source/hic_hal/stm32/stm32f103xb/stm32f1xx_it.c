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



/** @addtogroup Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//extern uint8_t I2C2_Buffer_Tx, I2C2_Buffer_Rx, CirReceiverData[], CirTransmitterData[];
//extern __IO uint8_t CIR_Transmitter_Ready;
//extern __IO uint8_t KeyPressed ;
//extern __IO uint8_t BAT_Detect ;
//extern __IO uint8_t ESP_Status ;

__IO uint8_t GET_USER2_BUTTON = 0;
__IO uint8_t USER2_BUTTON_STATUS = 0xFF;
__IO uint8_t SEND_REC_FLAG = 0;
__IO uint8_t SEND_REC_DATA = 0;
__IO uint8_t GET_TRAN_DATA = 0;
__IO uint8_t cir_reciver_count =0;
__IO uint8_t cir_transmitter_count =0;


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



void EXTI15_10_IRQHandler(void)
{
  if ((EXTI_GetITStatus(EXTI_Line10) != RESET))
  {  
		NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
		//NVIC_DisableIRQ(USART_IRQn);
		//KeyPressed = 1;
    ///* Clear the USER Button EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line10);
		//SleepMode_Measure();
  }
	
	if ((EXTI_GetITStatus(EXTI_Line12) != RESET))
  {
		if ((GND_DETECT_PORT->IDR & (1 << 2)))
			{
				//BAT_Detect = 1;
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
    //if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    //{
        //TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        // Timeout
        //ir_nec_reset_transmission();
    //}
}

void EXTI3_IRQHandler(void)
{
    //static unsigned int counter;

    if (EXTI_GetITStatus(EXTI_Line3) != RESET)
    {
        u8 bit = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3);

        // Restart Timer
        //counter = TIM_GetCounter(TIM2);
        //TIM_SetCounter(TIM2, 0);
			
        //ir_nec_state_machine(counter);

        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}

//Turn on or off ESP using USER2 Button
void EXTI4_IRQHandler(void)
{
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
