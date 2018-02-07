/*
 * Driver.cpp
 *
 *  Created on: 23.10.2017
 *      Author: micromikko
 */

#include "Driver.h"
#include "DigitalIoPin.h"
#include "ITM_write.h"
#include <cstring>


#include "Parser.h"
#include "Handles.h"
#include "stdlib.h"
#include <cmath>

xSemaphoreHandle sbRIT;
xSemaphoreHandle motorSemaphore;
volatile uint32_t RIT_count;
uint8_t motorNum;

extern "C" {
	void RIT_IRQHandler(void) {
		// This used to check if a context switch is required
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		// Tell timer that we have processed the interrupt.
		// Timer then removes the IRQ until next match occurs
		Chip_RIT_ClearIntStatus(LPC_RITIMER); // clear IRQ flag
		if(RIT_count > 0) {
//			char asd = RIT_count + '0';
//			Board_UARTPutSTR("OLLAA IRSSISSA!\r\n");
//			Board_UARTPutChar(asd);
//			Board_UARTPutSTR("\r\n");
			RIT_count--;
			// do something useful here...
			xSemaphoreGiveFromISR( motorSemaphore, &xHigherPriorityTaskWoken );
		} else {
			Chip_RIT_Disable(LPC_RITIMER); // disable timer
			// Give semaphore and set context switch flag if a higher priority task was woken up
			xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityTaskWoken);
		}
		// End the ISR and (possibly) do a context switch
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
}

void RIT_start(int count, int us) {
	uint64_t cmp_value;
	// Determine approximate compare value based on clock rate and passed interval
	cmp_value = (uint64_t) Chip_Clock_GetSystemClockRate() * (uint64_t) us / 1000000;

	// disable timer during configuration
	Chip_RIT_Disable(LPC_RITIMER);
	RIT_count = count;
	// enable automatic clear on when compare value==timer value
	// this makes interrupts trigger periodically
	Chip_RIT_EnableCompClear(LPC_RITIMER);
	// reset the counter
	Chip_RIT_SetCounter(LPC_RITIMER, 0);
	Chip_RIT_SetCompareValue(LPC_RITIMER, cmp_value);
	// start counting
	Chip_RIT_Enable(LPC_RITIMER);
	// Enable the interrupt signal in NVIC (the interrupt controller)
	NVIC_EnableIRQ(RITIMER_IRQn);
	// wait for ISR to tell that we're done
	if(xSemaphoreTake(sbRIT, portMAX_DELAY) == pdTRUE) {
		// Disable the interrupt signal in NVIC (the interrupt controller)
		NVIC_DisableIRQ(RITIMER_IRQn);
//		Board_UARTPutSTR("SEMAFOORI OTETTU\r\n");
	} else {
		// unexpected error
	}
}

void RIT_init() {
	Chip_RIT_Init(LPC_RITIMER);
	NVIC_SetPriority(RITIMER_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 3 );
	RIT_count = 0;
	sbRIT = xSemaphoreCreateBinary();
	motorSemaphore = xSemaphoreCreateBinary();
}

void taskExecute(void *pvParameters) {
	Handles *commonHandles = (Handles*) pvParameters;
	PlotterData plotdat;
	Parser parser;
	std::string *rawCommand;

	for(;;) {

		xQueueReceive(commonHandles->commandQueue_raw, &rawCommand, portMAX_DELAY);
		parser.generalParse(plotdat, *rawCommand);
		delete rawCommand;

		plotdat.resetCompack(); // -.,-.,-.,
		xSemaphoreGive(commonHandles->readyToReceive);
	}
}



void calculateDrive(PlotterData &pd) {
	pd.dX = pd.targetX - pd.currentX;
	pd.dY = pd.targetY - pd.currentY;


	pd.currentStepsX = pd.convertToSteps(pd.currentX);
	pd.currentStepsY = pd.convertToSteps(pd.currentY);

	pd.targetStepsX = pd.convertToSteps(pd.targetX);
	pd.targetStepsY = pd.convertToSteps(pd.targetY);

	pd.dStepsX = pd.targetStepsX - pd.currentStepsX;
	pd.dStepsY = pd.targetStepsY - pd.currentStepsY;

	pd.dStepsMax = std::max(abs(pd.dStepsX), abs(pd.dStepsY));

	pd.stepIntervalX = (double) abs(pd.dStepsX) / (double) pd.dStepsMax;
	pd.stepIntervalY = (double) abs(pd.dStepsY) / (double) pd.dStepsMax;

}

void justDrive(PlotterData &pd) {
	int countX = 0;
	int countY = 0;

	while((pd.currentStepsX != pd.targetStepsX) || (pd.currentStepsY != pd.targetStepsY)) {
		// move A
		if(pd.currentStepsX != pd.targetStepsX){
			countX += pd.stepIntervalX;

			if(countX >= 1) {
				if(pd.dStepsX > 0) {
					pd.dirX = true;
				} else {
					pd.dirX = false;
				}

				if(pd.dStepsX > 0) {
					pd.currentStepsX += 1;
				} else {
					pd.currentStepsX -= 1;
				}
				motorNum = 1;
				RIT_start(1, 1000);
				countX -= 1;
			}
		}

		if(pd.currentStepsY != pd.targetStepsY){
			countY += pd.stepIntervalY;

			if(countY >= 1) {
				if(pd.dStepsY > 0) {
					pd.dirY = true;
				} else {
					pd.dirY = false;
				}

				if(pd.dStepsY > 0) {
					pd.currentStepsY += 1;
				} else {
					pd.currentStepsY -= 1;
				}
				motorNum = 2;
				RIT_start(1, 1000);
				countX -= 1;
			}
		}
		motorNum = 0;
	}

	pd.currentStepsX = pd.targetStepsX;
	pd.currentStepsY = pd.targetStepsY;
	pd.currentX = pd.targetX;
	pd.currentY = pd.targetY;
}


void dtaskMotor(void *pvParameters) {

	DigitalIoPin stepPinX(0, 27, DigitalIoPin::output, false);
	DigitalIoPin dirPinX(0, 28, DigitalIoPin::output, false);

	DigitalIoPin stepPinY(0, 24, DigitalIoPin::output, false);
	DigitalIoPin dirPinY(1, 0, DigitalIoPin::output, false);

	while(1) {
		xSemaphoreTake(motorSemaphore, (TickType_t) portMAX_DELAY );

		switch(motorNum) {
		case 1:
			stepPinX.write(true);
			stepPinX.write(false);
			break;
		case 2:
			stepPinY.write(true);
			stepPinY.write(false);
			break;
		}

	}
}



