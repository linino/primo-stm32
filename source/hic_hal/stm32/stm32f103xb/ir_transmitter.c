#include <stm32f10x_tim.h>
#include "ir_transmitter.h"
//#include "CMSIS-DAP.h"

#define NEC_HDR_MARK	9000
#define NEC_HDR_SPACE	4500
#define NEC_BIT_MARK	560
#define NEC_ONE_SPACE	1600
#define NEC_ZERO_SPACE 560
#define NEC_RPT_SPACE	2250

#define TOPBIT 0x80000000

void Delay_us(unsigned int);

void IR_TimerPWMOutEnable(void)
{
    TIM_CCxCmd(TIM2,TIM_Channel_3,TIM_CCx_Enable);
}

void IR_TimerPWMOutDisable(void)
{
    TIM_CCxCmd(TIM2,TIM_Channel_3,TIM_CCx_Disable);
}

void IR_SendMark(int time) 
{
    // Sends an IR IR_SendMark for the specified number of microseconds.
    // The IR_SendMark output is modulated at the PWM frequency.
    IR_TimerPWMOutEnable(); // Enable PWM output
    Delay_us(time);

}

/* Leave pin off for time (given in microseconds) */
void IR_SendSpace(int time) 
{
    // Sends an IR IR_SendSpace for the specified number of microseconds.
    // A IR_SendSpace is no output, so the PWM output is disabled.
    IR_TimerPWMOutDisable(); // Disable PWM output
    Delay_us(time);
}


void IR_SendNEC(unsigned long data, int nbits)
{
	int i;
	
	IR_SendMark(NEC_HDR_MARK);
  IR_SendSpace(NEC_HDR_SPACE);
    
	for (i = 0; i < nbits; i++) {

		if (data & TOPBIT) {

			IR_SendMark(NEC_BIT_MARK);
			IR_SendSpace(NEC_ONE_SPACE);

		} else {

      IR_SendMark(NEC_BIT_MARK);
      IR_SendSpace(NEC_ZERO_SPACE);
		}

		data <<= 1;
	}

	IR_SendMark(NEC_BIT_MARK);
  IR_SendSpace(0);

}

void Delay_us(unsigned int us)
{
    unsigned int temp = 0;;  
    //SysTick->LOAD=us*fac_us;   
    SysTick->LOAD=us*9 + 1;    
                               
    SysTick->VAL=0x00;         
    SysTick->CTRL=0x01 ;       
    do{
        temp=SysTick->CTRL;
    }
    while( (temp&0x01) && (!(temp&(1<<16))) );  
    SysTick->CTRL=0x00;       
    SysTick->VAL =0X00;       
}
