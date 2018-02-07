/*
 * sw_btn_interrupts.cpp
 *
 *  Created on: 20.10.2017
 *      Author: Simo
 */

#include "sw_btn_interrupts.h"
#include "FreeRTOS.h"
#include "task.h"
#include "chip.h"
#include "semphr.h"

SemaphoreHandle_t limit_sem;

extern "C" {
void PIN_INT0_IRQHandler(void) {
	//
	uint32_t pin_n = 1 << 0;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, pin_n);
	xSemaphoreGiveFromISR(limit_sem, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

}

/* X limit switch 2 GPIO interrupt handler */
void PIN_INT1_IRQHandler(void) {
	uint32_t pin_n = 1 << 1;

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, pin_n);
	xSemaphoreGiveFromISR(limit_sem, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

}

/* Y limit switch 1 GPIO interrupt handler */
void PIN_INT2_IRQHandler(void) {
	uint32_t pin_n = 1 << 2;

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, pin_n);
	xSemaphoreGiveFromISR(limit_sem, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

}

/* Y limit switch 2 GPIO interrupt handler */
void PIN_INT3_IRQHandler(void) {
	uint32_t pin_n = 1 << 3;

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, pin_n);
	xSemaphoreGiveFromISR(limit_sem, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

}

void PIN_INT4_IRQHandler(void) {
	uint32_t pin_n = 1 << 4;

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, pin_n);
	xSemaphoreGiveFromISR(limit_sem, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

}

void PIN_INT5_IRQHandler(void) {
	uint32_t pin_n = 1 << 5;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, pin_n);
	xSemaphoreGiveFromISR(limit_sem, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

}

}

void GPIO_interrupt_init(void) {

	limit_sem = xSemaphoreCreateBinary();

	uint8_t x1_register = 0;
	uint8_t x1_port = 0;
	uint8_t x1_pin = 29;

	uint8_t x2_register = 1;
	uint8_t x2_port = 0;
	uint8_t x2_pin = 9;

	uint8_t y1_register = 2;
	uint8_t y1_port = 1;
	uint8_t y1_pin = 3;

	uint8_t y2_register = 3;
	uint8_t y2_port = 0;
	uint8_t y2_pin = 0;

	uint8_t sw1_register = 4;
	uint8_t sw1_port = 0;
	uint8_t sw1_pin = 8;

	uint8_t sw2_register = 5;
	uint8_t sw2_port = 1;
	uint8_t sw2_pin = 8;

	/* Bundles all registers into one big bitwise ORed uint8_t for when
	 * we need to use a bitmask for all of them */
	uint8_t all_registers = PININTCH(x1_register) | PININTCH(x2_register)/* | PININTCH(y1_register)
			| PININTCH(y2_register) */| PININTCH(sw1_register) | PININTCH(sw2_register);

	/* Interrupt priority number for all hard stop interrupts */
	uint32_t hardstop_interrupt_priority = 1;

	Chip_PININT_Init(LPC_GPIO_PIN_INT);

	/* Maps GPIO pins to interrupt channels */
	Chip_INMUX_PinIntSel(x1_register, x1_port, x1_pin);
	Chip_INMUX_PinIntSel(x2_register, x2_port, x2_pin);
	Chip_INMUX_PinIntSel(y1_register, y1_port, y1_pin);
	Chip_INMUX_PinIntSel(y2_register, y2_port, y2_pin);
	Chip_INMUX_PinIntSel(sw1_register, sw1_port, sw1_pin);
	Chip_INMUX_PinIntSel(sw2_register, sw2_port, sw2_pin);

	/*  */
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, all_registers); // edge mode
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, all_registers); // clear status
	Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, all_registers); // rising edge

	/* Sets interrupt priority for all interrupts */
	NVIC_SetPriority(PIN_INT0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + hardstop_interrupt_priority);
	NVIC_SetPriority(PIN_INT1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + hardstop_interrupt_priority);
	NVIC_SetPriority(PIN_INT2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + hardstop_interrupt_priority);
	NVIC_SetPriority(PIN_INT3_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + hardstop_interrupt_priority);
	NVIC_SetPriority(PIN_INT4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + hardstop_interrupt_priority);
	NVIC_SetPriority(PIN_INT5_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + hardstop_interrupt_priority);

	/* Enables the created interrupts in the NVIC */
	NVIC_EnableIRQ(PIN_INT0_IRQn);
	NVIC_EnableIRQ(PIN_INT1_IRQn);
	NVIC_EnableIRQ(PIN_INT2_IRQn);
	NVIC_EnableIRQ(PIN_INT3_IRQn);
	NVIC_EnableIRQ(PIN_INT4_IRQn);
	NVIC_EnableIRQ(PIN_INT5_IRQn);
}

/* Disables interrupts for hard stop */
static void disableInterrupts(void) {
	NVIC_DisableIRQ(PIN_INT0_IRQn);
	NVIC_DisableIRQ(PIN_INT1_IRQn);
	NVIC_DisableIRQ(PIN_INT2_IRQn);
	NVIC_DisableIRQ(PIN_INT3_IRQn);
	NVIC_DisableIRQ(PIN_INT4_IRQn);
	NVIC_DisableIRQ(PIN_INT5_IRQn);
	NVIC_DisableIRQ(RITIMER_IRQn);
}

/* Set to highest priority so this hogs all CPU time */
void dtaskHardStop(void *pvParameters) {

	/* Wait for a sempahore from IRQ handlers */
	xSemaphoreTake(limit_sem, portMAX_DELAY);

	/* Disable interrupts for switches and the RIT timer to
	 * make sure the stepper motors stop, at this point
	 * we've done goofed inrecoverably, so it goes into
	 * an infinite loop */
	disableInterrupts();
	while(1) {

	}
}

