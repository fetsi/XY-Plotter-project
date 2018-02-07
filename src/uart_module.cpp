/*
 * uart_module.cpp
 *
 *  Created on: Oct 21, 2017
 *      Author: Tuomas-laptop
 */
#include "uart_module.h"

#include "FreeRTOS.h"
#include "task.h"
#include <string>
#include "ITM_write.h"
#include <mutex>
#include "Handles.h"


QueueHandle_t char_queue;
myMutex serial_guard;

#define INT_MASK  (uint32_t)(1 << 0)



extern "C"
{
void UART0_IRQHandler(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint32_t c;
	{
		//Take a mutex before accessing the UART receive register to prevent race conditions
		std::lock_guard<myMutex> locker(serial_guard);
		c = LPC_USART0->RXDATA;
	}

	xQueueSendToBackFromISR(char_queue, &c, &xHigherPriorityTaskWoken );
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	Chip_UART_IntDisable(LPC_USART0, INT_MASK);
}
}


/*Create queue for messaging characters from UART interrupt to reader task.  Configure UART interrupts.*/
void UARTModule_init() {
	configUARTInterrupt();
	ITM_init();

	char_queue = xQueueCreate(1, sizeof(uint32_t));
}


static void configUARTInterrupt() {
	uint32_t mask = (1 << 0);
	Chip_UART_IntEnable(LPC_USART0, mask);
	/* I added the macro here for safety so we don't interfere with system interrupts */
	NVIC_SetPriority(UART0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
	NVIC_EnableIRQ(UART0_IRQn);
}

/*Read characters from UART, append them to our command string and finally place the entire string into command queue*/
void dtaskUARTReader(void *pvParameters) {
	Handles *commonHandles = (Handles*) pvParameters;

	while(1) {
		std::string *str = new std::string("");
		uint32_t c;

		while (1) {
			xQueueReceive(char_queue, &c, portMAX_DELAY);

			(*str).push_back((char)c);
			Chip_UART_IntEnable(LPC_USART0, INT_MASK);
			
			if(c == '\n' || c == '\r') {
				//We've reached a newline so we have an entire command to pass forward
				xQueueSendToBack(commonHandles->commandQueue_raw, &str, portMAX_DELAY);
				break;
			}
		}
	}
}

/* Send an "OK" string to mDraw to signal that we have processed the previous command and are ready
 * for a new one */
void taskSendOK(void *pvParameters) {
	Handles *commonHandles = (Handles*) pvParameters;
	while(1) {
		if( xSemaphoreTake(commonHandles->readyToReceive, portMAX_DELAY)  == pdTRUE ) {

			{
			std::lock_guard<myMutex> locker(serial_guard);
			Board_UARTPutSTR("OK\n");
			}
			vTaskDelay(5);
		}
	}
}




