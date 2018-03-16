/*****************************************************************************
* Auteur : Maxime Turenne
* Copyright : Maxime Turenne
* Description: Demo d'utilisation de FreeRTOS dans le cadre d'un thermostate numérique.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Environment header files. */
#include "power_clocks_lib.h"

#include "board.h"
#include "compiler.h"
#include "dip204.h"
#include "intc.h"
#include "gpio.h"
#include "pm.h"
#include "delay.h"
#include "spi.h"
#include "conf_clock.h"
#include "adc.h"
#include "usart.h"
#include <print_funcs.h>

/* Scheduler header files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// tasks
void vUART_Cmd_RX(void *pvParameters);

// initialization functions
void initialiseUSART(void);

// global var
volatile U32 CHAR_RECU = 'd';
volatile char START = 0;

// semaphore
static xSemaphoreHandle CHAR_RECU_SEMAPHORE = NULL;


int main(void) {
	pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP);
	initialiseUSART();

	CHAR_RECU_SEMAPHORE = xSemaphoreCreateCounting(1,1);

	xTaskCreate(vUART_Cmd_RX, (signed char*)"Receive", configMINIMAL_STACK_SIZE*2, NULL, 1, NULL );
	
	print_dbg("xTaskCreate \n");
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}

/************************************************************************/
/* FreeRTOS Task that receive a command from the usart.                 */
/************************************************************************/
void vUART_Cmd_RX(void *pvParameters) {
	print_dbg("vUART_Cmd_RX started \n");
	
	while (1) {

		// recois un caractere
		usart_read_char(&AVR32_USART1, &CHAR_RECU);
		
		// envoi un caractere
		usart_write_char(&AVR32_USART1, CHAR_RECU);
		
		
		if(CHAR_RECU == 's' || CHAR_RECU == 'S'){
			//xSemaphoreTake(CHAR_RECU_SEMAPHORE, portMAX_DELAY);
			START = 1;
			//xSemaphoreGive(CHAR_RECU_SEMAPHORE);
		}
		else if(CHAR_RECU == 'x' || CHAR_RECU == 'X'){
			//xSemaphoreTake(CHAR_RECU_SEMAPHORE, portMAX_DELAY);
			START = 0;
			//xSemaphoreGive(CHAR_RECU_SEMAPHORE);
		}
		
		vTaskDelay(200);
	}
}

/************************************************************************/
/* Fonction that initialise USART                                       */
/************************************************************************/
void initialiseUSART(void){
	static const gpio_map_t USART_GPIO_MAP =
	{
		{AVR32_USART1_RXD_0_0_PIN, AVR32_USART1_RXD_0_0_FUNCTION},
		{AVR32_USART1_TXD_0_0_PIN, AVR32_USART1_TXD_0_0_FUNCTION}
	};

	static const usart_options_t USART_OPTIONS =
	{
		.baudrate     = 57600,
		.charlength   = 8,
		.paritytype   = USART_NO_PARITY,
		.stopbits     = USART_1_STOPBIT,
		.channelmode  = USART_NORMAL_CHMODE
	};

	// Assign GPIO to USART.
	gpio_enable_module(USART_GPIO_MAP,
	sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0]));

	// Initialize USART in RS232 mode.
	usart_init_rs232(&AVR32_USART1, &USART_OPTIONS, FOSC0);
}