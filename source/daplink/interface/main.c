/**
 * @file    main.c
 * @brief   Entry point for interface program logic
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

#include "string.h"
#include "stdio.h"
#include "stdint.h"

#include "RTL.h"
#include "rl_usb.h"
#include "main.h"
#include "board.h"
#include "gpio.h"
#include "uart.h"
#include "serial.h"
#include "tasks.h"
#include "target_reset.h"
#include "swd_host.h"
#include "info.h"
#include "vfs_manager.h"
#include "settings.h"
#include "daplink.h"
#include "util.h"
#include "DAP.h"
#include "stm32f10x_lp_modes.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_adc.h"
//CIR Library
#include "ir_transmitter.h"
#include "ir_receiver.h"


// Event flags for main task
// Timers events
#define FLAGS_MAIN_90MS         (1 << 0)
#define FLAGS_MAIN_30MS         (1 << 1)
// Reset events
#define FLAGS_MAIN_RESET        (1 << 2)
// Other Events
#define FLAGS_MAIN_POWERDOWN    (1 << 4)
#define FLAGS_MAIN_DISABLEDEBUG (1 << 5)
#define FLAGS_MAIN_PROC_USB     (1 << 9)
// Used by msd when flashing a new binary
#define FLAGS_LED_BLINK_30MS    (1 << 6)
// Timing constants (in 90mS ticks)
// USB busy time
#define USB_BUSY_TIME           (33)
// Delay before a USB device connect may occur
#define USB_CONNECT_DELAY       (11)
// Delay before target may be taken out of reset or reprogrammed after startup
#define STARTUP_DELAY           (1)
// Decrement to zero
#define DECZERO(x)              (x ? --x : 0)

// Reference to our main task
OS_TID main_task_id;
OS_TID serial_task_id;

// USB busy LED state; when TRUE the LED will flash once using 30mS clock tick
static uint8_t hid_led_usb_activity = 0;
static uint8_t cdc_led_usb_activity = 0;
static uint8_t msc_led_usb_activity = 0;
static main_led_state_t hid_led_state = MAIN_LED_FLASH;
static main_led_state_t cdc_led_state = MAIN_LED_FLASH;
static main_led_state_t msc_led_state = MAIN_LED_FLASH;

// Global state of usb
main_usb_connect_t usb_state;

static U64 stk_timer_30_task[TIMER_TASK_30_STACK / sizeof(U64)];
static U64 stk_dap_task[DAP_TASK_STACK / sizeof(U64)];
static U64 stk_serial_task[SERIAL_TASK_STACK / sizeof(U64)];
static U64 stk_main_task[MAIN_TASK_STACK / sizeof(U64)];

uint8_t I2C2_Buffer_Tx;
uint8_t I2C2_Buffer_Rx;
uint8_t CirReceiverData[DataSize];
uint8_t CirTransmitterData[DataSize];
__IO uint8_t CIR_Transmitter_Ready = 0;
__IO uint8_t KeyPressed = 0;
__IO uint8_t BAT_Detect = 0;
__IO uint8_t Disable_StandbyMode = 0;

void GPIO_GND_DETECT_SETUP(void);
void GPIO_USER1_BUTTON_SETUP(void);
void GPIO_USER2_BUTTON_SETUP(void);
void WKUP_SETUP(void);
void BAT_DET_SETUP(void);
void IR_Init(void);
void IR_DeInit(void);
void TIM_Init(void);
void enableIRIn(void);
void disableIRIn(void);
void IR_Timer_PWM_ConfigFreqKHZ(unsigned char Khz);
void enableIROut(int khz);
void disableIROut(void);
void ADC_Configuration(void);

// Timer task, set flags every 30mS and 90mS
__task void timer_task_30mS(void)
{
    uint8_t i = 0;
    os_itv_set(3); // 30mS

    while (1) {
        os_itv_wait();
        os_evt_set(FLAGS_MAIN_30MS, main_task_id);

        if (!(i++ % 3)) {
            os_evt_set(FLAGS_MAIN_90MS, main_task_id);
        }
    }
}

// Forward reset from the user pressing the reset button
// Boards which tie the reset pin directly to the target
// should override this function with a stub that does nothing
__attribute__((weak))
void target_forward_reset(bool assert_reset)
{
    if (assert_reset) {
        target_set_state(RESET_HOLD);
    } else {
        target_set_state(RESET_RUN);
    }
}

// Functions called from other tasks to trigger events in the main task
// parameter should be reset type??
void main_reset_target(uint8_t send_unique_id)
{
    os_evt_set(FLAGS_MAIN_RESET, main_task_id);
    return;
}

// Flash HID LED using 30mS tick
void main_blink_hid_led(main_led_state_t permanent)
{
    hid_led_usb_activity = 1;
    hid_led_state = (permanent) ? MAIN_LED_FLASH_PERMANENT : MAIN_LED_FLASH;
    return;
}

// Flash CDC LED using 30mS tick
void main_blink_cdc_led(main_led_state_t permanent)
{
    cdc_led_usb_activity = 1;
    cdc_led_state = (permanent) ? MAIN_LED_FLASH_PERMANENT : MAIN_LED_FLASH;
    return;
}

// Flash MSC LED using 30mS tick
void main_blink_msc_led(main_led_state_t permanent)
{
    msc_led_usb_activity = 1;
    msc_led_state = (permanent) ? MAIN_LED_FLASH_PERMANENT : MAIN_LED_FLASH;
    return;
}

// Power down the interface
void main_powerdown_event(void)
{
    os_evt_set(FLAGS_MAIN_POWERDOWN, main_task_id);
    return;
}

// Disable debug on target
void main_disable_debug_event(void)
{
    os_evt_set(FLAGS_MAIN_DISABLEDEBUG, main_task_id);
    return;
}

void USBD_SignalHandler()
{
    isr_evt_set(FLAGS_MAIN_PROC_USB, main_task_id);
}

void HardFault_Handler()
{
    util_assert(0);
    NVIC_SystemReset();

    while (1); // Wait for reset
}

void PowerOn_ESP(void) 				{ ESP_PW_PORT->BSRR   = ESP_PW_PIN; }
void Poweroff_ESP(void) 			{ ESP_PW_PORT->BRR   = ESP_PW_PIN; }

void Enable_ESP(void) 				{ ESP_EN_PORT->BRR   = ESP_EN_PIN; }
void Disable_ESP(void)        { ESP_EN_PORT->BSRR   = ESP_EN_PIN; }

void LedPowerOn(void)			{	POWER_EN_PORT->BSRR = POWER_EN_PIN;	}
void LedPowerOff(void)		{	POWER_EN_PORT->BRR  = POWER_EN_PIN;	}
void LedPowerToggle(void)	{	POWER_EN_PORT->ODR ^= POWER_EN_PIN;	}

void LedUSER2On(void)			{	USER2_LED_PORT->BRR  = USER2_LED_PIN;	}
void LedUSER2Off(void)		{	USER2_LED_PORT->BSRR = USER2_LED_PIN;	}
void LedUSER2Toggle(void)	{	USER2_LED_PORT->ODR ^= USER2_LED_PIN;	}

void LedBLEOn(void)				{	BLE_LED_PORT->BRR  = BLE_LED_PIN;	}
void LedBLEOff(void)			{	BLE_LED_PORT->BSRR = BLE_LED_PIN;	}
void LedBLEToggle(void)		{	BLE_LED_PORT->ODR ^= BLE_LED_PIN;	}

void I2C2_RCC_Configuration(void);
void I2C2_GPIO_Configuration(void);
void I2C2_NVIC_Configuration(void);
void I2C2_Configuration(void);

os_mbx_declare(serial_mailbox, 20);
#define SIZE_DATA (64)
static uint8_t data[SIZE_DATA];

__task void serial_process()
{
    UART_Configuration config;
    int32_t len_data = 0;
    void *msg;

    while (1) {
        // Check our mailbox to see if we need to set anything up with the UART
        // before we do any sending or receiving
        if (os_mbx_wait(&serial_mailbox, &msg, 0) == OS_R_OK) {
            switch ((SERIAL_MSG)(unsigned)msg) {
                case SERIAL_INITIALIZE:
                    uart_initialize();
                    break;

                case SERIAL_UNINITIALIZE:
                    uart_uninitialize();
                    break;

                case SERIAL_RESET:
                    uart_reset();
                    break;

                case SERIAL_SET_CONFIGURATION:
                    serial_get_configuration(&config);
                    uart_set_configuration(&config);
                    break;

                default:
                    break;
            }
        }

        len_data = USBD_CDC_ACM_DataFree();

        if (len_data > SIZE_DATA) {
            len_data = SIZE_DATA;
        }

        if (len_data) {
            len_data = uart_read_data(data, len_data);
        }

        if (len_data) {
            if (USBD_CDC_ACM_DataSend(data , len_data)) {
                main_blink_cdc_led(MAIN_LED_OFF);
            }
        }

        len_data = uart_write_free();

        if (len_data > SIZE_DATA) {
            len_data = SIZE_DATA;
        }

        if (len_data) {
            len_data = USBD_CDC_ACM_DataRead(data, len_data);
        }

        if (len_data) {
            if (uart_write_data(data, len_data)) {
                main_blink_cdc_led(MAIN_LED_OFF);
            }
        }
    }
}

extern __task void hid_process(void);
__attribute__((weak)) void prerun_target_config(void) {}

__task void main_task(void)
{
	  //Power on and Enable ESP8266
		PowerOn_ESP();
		Enable_ESP();
		//I2c slave mode
		I2C2_RCC_Configuration();
		I2C2_NVIC_Configuration();
		I2C2_GPIO_Configuration();
		I2C_Cmd(I2C2, ENABLE);
		I2C2_Configuration(); 
		I2C_ITConfig(I2C2, I2C_IT_EVT | I2C_IT_BUF, ENABLE);
    // State processing
    uint16_t flags = 0;
    // LED
    gpio_led_state_t hid_led_value = GPIO_LED_ON;
    gpio_led_state_t cdc_led_value = GPIO_LED_ON;
    gpio_led_state_t msc_led_value = GPIO_LED_ON;
    // USB
    uint32_t usb_state_count = USB_BUSY_TIME;
    // thread running after usb connected started
    uint8_t thread_started = 0;
    // button state
    //main_reset_state_t main_reset_button_state = MAIN_RESET_RELEASED;
    // Initialize settings
    config_init();
    // Initialize our serial mailbox
    os_mbx_init(&serial_mailbox, sizeof(serial_mailbox));
    // Get a reference to this task
    main_task_id = os_tsk_self();
    // leds
    gpio_init();
		GPIO_GND_DETECT_SETUP();
		//GPIO_USER1_BUTTON_SETUP();
    GPIO_USER2_BUTTON_SETUP();
		WKUP_SETUP();
		BAT_DET_SETUP();

		if ((GND_DETECT_PORT->IDR & (1 << 2)))
			Disable_External_SWD_Program();
		else
			Enable_External_SWD_Program();

		/* Enable ADC Conversion */
		ADC_Configuration();

    // Turn on LED
    gpio_set_hid_led(GPIO_LED_ON);
    gpio_set_cdc_led(GPIO_LED_ON);
    gpio_set_msc_led(GPIO_LED_ON);		
    // Initialize the DAP
    DAP_Setup();
    // do some init with the target before USB and files are configured
    prerun_target_config();
    // Update versions and IDs
    info_init();
    // USB
    usbd_init();
    vfs_mngr_fs_enable(true);
    usbd_connect(0);
    usb_state = USB_CONNECTING;
    usb_state_count = USB_CONNECT_DELAY;
    // Start timer tasks
    os_tsk_create_user(timer_task_30mS, TIMER_TASK_30_PRIORITY, (void *)stk_timer_30_task, TIMER_TASK_30_STACK);

		if ((!(BAT_DET_PORT->IDR & (1 << 12))) & (Disable_StandbyMode == 0))
			StandbyRTCMode_Measure();

    while (1) {
				
			  if (KeyPressed == 1)	
					SleepMode_Measure();
				
				if (BAT_Detect == 1) {
					if (Disable_StandbyMode == 0) {
						RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
						PWR_WakeUpPinCmd(ENABLE);
						StandbyRTCMode_Measure();
					}
				}
					
        os_evt_wait_or(FLAGS_MAIN_RESET             // Put target in reset state
                       | FLAGS_MAIN_90MS            // 90mS tick
                       | FLAGS_MAIN_30MS            // 30mS tick
                       | FLAGS_MAIN_POWERDOWN       // Power down interface
                       | FLAGS_MAIN_DISABLEDEBUG    // Disable target debug
                       | FLAGS_MAIN_PROC_USB        // process usb events
                       , NO_TIMEOUT);
        // Find out what event happened
        flags = os_evt_get();

        if (flags & FLAGS_MAIN_PROC_USB) {
            USBD_Handler();
        }

        if (flags & FLAGS_MAIN_RESET) {
            target_set_state(RESET_RUN);
        }

        if (flags & FLAGS_MAIN_POWERDOWN) {
            // Disable debug
            //target_set_state(NO_DEBUG);
            // Disconnect USB
            usbd_connect(0);
            // Turn off LED
            gpio_set_hid_led(GPIO_LED_OFF);
            gpio_set_cdc_led(GPIO_LED_OFF);
            gpio_set_msc_led(GPIO_LED_OFF);

            // TODO: put the interface chip in sleep mode
            while (1);
        }
				
				if (CIR_Transmitter_Ready	== 1) {
						unsigned long data;
						data = 	(CirTransmitterData[0] << 24);
						data |= (CirTransmitterData[1] << 16);
						data |= (CirTransmitterData[2] <<  8);
						data |=  CirTransmitterData[3];
			
						IR_SendNEC(data, 32);
						CIR_Transmitter_Ready = 0;
				}

        //if (flags & FLAGS_MAIN_DISABLEDEBUG) {
            // Disable debug
            //target_set_state(NO_DEBUG);
        //}

        if (flags & FLAGS_MAIN_90MS) {
            // Update USB busy status
            vfs_mngr_periodic(90); // FLAGS_MAIN_90MS

            // Update USB connect status
            switch (usb_state) {
                case USB_DISCONNECTING:
                    usb_state = USB_DISCONNECTED;
                    usbd_connect(0);
                    break;

                case USB_CONNECTING:

                    // Wait before connecting
                    if (DECZERO(usb_state_count) == 0) {
                        usbd_connect(1);
                        usb_state = USB_CHECK_CONNECTED;
                    }

                    break;

                case USB_CHECK_CONNECTED:
                    if (usbd_configured()) {
                        if (!thread_started) {
                            os_tsk_create_user(hid_process, DAP_TASK_PRIORITY, (void *)stk_dap_task, DAP_TASK_STACK);
                            serial_task_id = os_tsk_create_user(serial_process, SERIAL_TASK_PRIORITY, (void *)stk_serial_task, SERIAL_TASK_STACK);
                            thread_started = 1;
                        }

                        usb_state = USB_CONNECTED;
                    }

                    break;

                case USB_CONNECTED:
                case USB_DISCONNECTED:
                default:
                    break;
            }
        }

        // 30mS tick used for flashing LED when USB is busy
        if (flags & FLAGS_MAIN_30MS) {
					  // handle reset button without eventing
            //switch (main_reset_button_state) {
                //default:
                //case MAIN_RESET_RELEASED:
                    //if (0 == gpio_get_sw_reset()) {
                        //main_reset_button_state = MAIN_RESET_PRESSED;
                        //target_forward_reset(true);
                    //}

                    //break;

                //case MAIN_RESET_PRESSED:

                    // ToDo: add a counter to do a mass erase or target recovery after xxx seconds of being held
                    //if (1 == gpio_get_sw_reset()) {
                        //main_reset_button_state = MAIN_RESET_TARGET;
                    //}

                    //break;

                //case MAIN_RESET_TARGET:
                    //target_forward_reset(false);
                    //main_reset_button_state = MAIN_RESET_RELEASED;
                    //break;
            //}

            if (hid_led_usb_activity && ((hid_led_state == MAIN_LED_FLASH) || (hid_led_state == MAIN_LED_FLASH_PERMANENT))) {
                // Flash DAP LED ONCE
                if (hid_led_value) {
                    hid_led_value = GPIO_LED_OFF;
                } else {
                    hid_led_value = GPIO_LED_ON; // Turn on

                    if (hid_led_state == MAIN_LED_FLASH) {
                        hid_led_usb_activity = 0;
                    }
                }

                // Update hardware
                gpio_set_hid_led(hid_led_value);
            }

            if (msc_led_usb_activity && ((msc_led_state == MAIN_LED_FLASH) || (msc_led_state == MAIN_LED_FLASH_PERMANENT))) {
                // Flash MSD LED ONCE
                if (msc_led_value) {
                    msc_led_value = GPIO_LED_OFF;
                } else {
                    msc_led_value = GPIO_LED_ON; // Turn on

                    if (msc_led_state == MAIN_LED_FLASH) {
                        msc_led_usb_activity = 0;
                    }
                }

                // Update hardware
                gpio_set_msc_led(msc_led_value);
            }

            if (cdc_led_usb_activity && ((cdc_led_state == MAIN_LED_FLASH) || (cdc_led_state == MAIN_LED_FLASH_PERMANENT))) {
                // Flash CDC LED ONCE
                if (cdc_led_value) {
                    cdc_led_value = GPIO_LED_OFF;
                } else {
                    cdc_led_value = GPIO_LED_ON; // Turn on

                    if (cdc_led_state == MAIN_LED_FLASH) {
                        cdc_led_usb_activity = 0;
                    }
                }

                // Update hardware
                gpio_set_cdc_led(cdc_led_value);
            }
        }
    }
}

int main(void)
{
    // Explicitly set the vector table since the bootloader might not set
    // it to what we expect.
#if DAPLINK_ROM_BL_SIZE > 0
    SCB->VTOR = SCB_VTOR_TBLOFF_Msk & DAPLINK_ROM_IF_START;
#endif
    os_sys_init_user(main_task, MAIN_TASK_PRIORITY, stk_main_task, MAIN_TASK_STACK);
}

void GPIO_GND_DETECT_SETUP(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable the GPIO Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
	
	/* Configure Button pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GND_DETECT_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GND_DETECT_PORT, &GPIO_InitStructure);
	
  /* Connect Button EXTI Line to Button GPIO Pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource2);

  /* Configure Button EXTI line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line2;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set Button EXTI Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);  
}

void GPIO_USER1_BUTTON_SETUP(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable the GPIO Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	
	/* Configure Button pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = USER1_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(USER1_PORT, &GPIO_InitStructure);
	
  /* Connect Button EXTI Line to Button GPIO Pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource10);

  /* Configure Button EXTI line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line10;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set Button EXTI Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);  
}

void GPIO_USER1_BUTTON_Disable(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  EXTI_InitStructure.EXTI_Line = EXTI_Line10;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);
}

void GPIO_USER2_BUTTON_SETUP(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  //EXTI_InitTypeDef EXTI_InitStructure;
  //NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable the GPIO Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	
	/* Configure Button pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = USER2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(USER2_PORT, &GPIO_InitStructure);
	
  /* Connect Button EXTI Line to Button GPIO Pin */
  //GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);

  /* Configure Button EXTI line */
  //EXTI_InitStructure.EXTI_Line = EXTI_Line4;
  //EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  //EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  //EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  //EXTI_Init(&EXTI_InitStructure);

  /* Enable and set Button EXTI Interrupt to the lowest priority */
  //NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn ;
  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  //NVIC_Init(&NVIC_InitStructure); 
}

void WKUP_SETUP(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable the GPIO Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Pin = WKUP_PIN;
  GPIO_Init(WKUP_PORT, &GPIO_InitStructure); 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	PWR_WakeUpPinCmd(ENABLE);	
}

void BAT_DET_SETUP(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable the GPIO Clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Pin = BAT_DET_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(BAT_DET_PORT, &GPIO_InitStructure);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource12);

  EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);  
}

void BAT_DET_EXTI_LINE_Enable(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

void BAT_DET_EXTI_LINE_Disable(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);
}

void I2C2_RCC_Configuration(void)
{	
  /* Enable I2C2 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
  /* Enable GPIOB clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
}

void I2C2_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure I2C2 pins: SCL and SDA ----------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void I2C2_NVIC_Configuration(void)
{
 // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
  /* Configure and enable I2C2 event interrupt -------------------------------*/
	NVIC_InitTypeDef  NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd =ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable I2C2 error interrupt -------------------------------*/  
  NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
}

void I2C2_Configuration(void)
{
		/* I2C2 configuration ------------------------------------------------------*/
	I2C_InitTypeDef   I2C_InitStructure;
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = I2C2_SLAVE_ADDRESS7;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;
  I2C_Init(I2C2, &I2C_InitStructure); 
}

void IR_Init(void) {
	// Use PA3 as input from IR receiver
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Enable clock and its interrupts
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource3);
	
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  EXTI_InitStructure.EXTI_Line = EXTI_Line3;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

void IR_DeInit(void) 
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);

  EXTI_InitStructure.EXTI_Line = EXTI_Line3;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);
}

void TIM_Init(void) 
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	
	TIM_DeInit(TIM2);
	
	// Enable clock and its interrupts
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Prescaler = 72- 1 ;
	TIM_InitStructure.TIM_Period = 10000 - 1; // Update event every 10000 us / 10 ms
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_InitStructure);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

void enableIRIn(void)
{
 //CIR Init
 IR_Init();
 TIM_Init();
 ir_nec_init(GPIO_Pin_3, GPIOA);
}

void disableIRIn(void)
{
 //CIR DeInit
 IR_DeInit();
}

void IR_Timer_PWM_ConfigFreqKHZ(unsigned char Khz)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;
    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_OCInitTypeDef        TIM_OCInitStruct;
    
    TIM_DeInit(TIM2);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseInitStruct.TIM_Prescaler = (72000000/F_TIM_CLK)-1;  //Prescaler=(2+1)=3,So here TIM clock is 48M/3=16M
	  TIM_TimeBaseInitStruct.TIM_Period = F_TIM_CLK/(1000*Khz);
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0x00;
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStruct);

    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    //TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
    //TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    //TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    //TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = (F_TIM_CLK/(1000*Khz))/3;  //Duty is 50%

    TIM_OC3Init(TIM2,&TIM_OCInitStruct);
    
    TIM_Cmd(TIM2,ENABLE);

    IR_TimerPWMOutDisable(); 
}

void enableIROut(int khz)
{
    IR_Timer_PWM_ConfigFreqKHZ(khz);
}

void disableIROut(void)
{
		TIM_Cmd(TIM2,DISABLE);
}

void ADC_Configuration(void)
{
		ADC_InitTypeDef ADC_InitStructure;
		GPIO_InitTypeDef  GPIO_InitStructure;

		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
		GPIO_InitStructure.GPIO_Pin   = VOL_DET_PIN;
		GPIO_Init(VOL_DET_PORT, &GPIO_InitStructure);

		RCC_ADCCLKConfig (RCC_PCLK2_Div6);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfChannel = 7;

		ADC_RegularChannelConfig(ADC1,ADC_Channel_7, 1,ADC_SampleTime_28Cycles5);
		ADC_Init (ADC1, &ADC_InitStructure);
		ADC_Cmd (ADC1,ENABLE);

		ADC_ResetCalibration(ADC1);
		while(ADC_GetResetCalibrationStatus(ADC1));
		ADC_StartCalibration(ADC1);
		while(ADC_GetCalibrationStatus(ADC1));

		ADC_Cmd (ADC1,ENABLE);
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
