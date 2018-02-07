/*
 * Servo.cpp
 *
 *  Created on: Oct 23, 2017
 *      Author: Tuomas-laptop
 */

#include "Servo.h"

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include "FreeRTOS.h"
#include "task.h"

#include <cr_section_macros.h>

Servo::Servo(int portNum, int pinNum) {
	this->portNum = portNum;
	this->pinNum = pinNum;
	this->pin = new DigitalIoPin (portNum, pinNum, DigitalIoPin::pullup, true);
	init();
}

Servo::~Servo() {
	delete pin;
}

void Servo::init() {

	/*Configure SCTimer for controlling the Pen servo with PWM*/
	Chip_SCT_Init(LPC_SCTLARGE0);
	LPC_SCTLARGE0->CONFIG |= (1 << 17); // Set auto limit
	LPC_SCTLARGE0->CTRL_L |= (72-1) << 5; // set prescaler, SCTimer/PWM clock = 1 MHz


	LPC_SCTLARGE0->EVENT[0].STATE = 0xFFFFFFFF; // event 0 happens in all states
	LPC_SCTLARGE0->MATCHREL[0].L = 20000; // match 0 @ 20 000/1MHz = (50 Hz PWM freq)
	LPC_SCTLARGE0->MATCHREL[1].L = 1500; // match 1 used for duty cycle (by default 1.5 ms which positions the servo in the center)
	LPC_SCTLARGE0->EVENT[0].CTRL = (1 << 12); // match 0 condition only
	LPC_SCTLARGE0->EVENT[1].STATE = 0xFFFFFFFF; // event 1 happens in all states
	LPC_SCTLARGE0->EVENT[1].CTRL = (1 << 0) | (1 << 12); // match 1 condition only
	LPC_SCTLARGE0->OUT[0].SET = (1 << 0); // event 0 will set SCTx_OUT0
	LPC_SCTLARGE0->OUT[0].CLR = (1 << 1); // event 1 will clear SCTx_OUT0
	Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT0_O, this->portNum, this->pinNum); //route the SCT output to the GPIO pin that supplies the servo's control signal
	LPC_SCTLARGE0->CTRL_L &= ~(1 << 2); // unhalt timer by clearing bit 2 of CTRL reg
}

void Servo::penDown() {
	while(LPC_SCTLARGE0->MATCHREL[1].L <= penDown_cycle_length) {
		++LPC_SCTLARGE0->MATCHREL[1].L;
		vTaskDelay(1);
	}
}

void Servo::penUp() {
	while(LPC_SCTLARGE0->MATCHREL[1].L >= penUp_cycle_length) {
		--LPC_SCTLARGE0->MATCHREL[1].L;
		vTaskDelay(1);
	}
}

