#ifndef __IR_RECEIVER_H
#define __IR_RECEIVER_H

#include <stm32f10x_gpio.h>
#include <stdbool.h>

#define TOLERANCE 20

typedef enum {
	IR_NEC_NONE = 0,
	IR_NEC_NDEF,
	IR_NEC_FIRST_BURST,
	IR_NEC_SECOND_BURST,
	IR_NEC_SECOND_BURST_REPEAT,
	IR_NEC_1
} ir_nec_state;

void ir_nec_init(u16 pin, GPIO_TypeDef *gpio);
void ir_nec_state_machine(unsigned int time);
void ir_nec_reset_transmission(void);

#endif // __IR_RECEIVER_H
