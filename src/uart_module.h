#ifndef UART_MODULE_H_
#define UART_MODULE_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "myMutex.h"



extern "C" { void UART0_IRQHandler(void); }
void UARTModule_init();
void dtaskUARTReader(void *pvParameters);
void taskSendOK(void *pvParameters);   		//just a temporary one for testing reader functionality. Will be replaced by a write to command queue
static void configUARTInterrupt();



#endif

