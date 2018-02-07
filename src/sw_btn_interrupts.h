/*
 * sw_btn_interrupts.h
 *
 *  Created on: 20.10.2017
 *      Author: Simo
 */

#ifndef SW_BTN_INTERRUPTS_H_
#define SW_BTN_INTERRUPTS_H_

#include "DigitalIoPin.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/* I was using the DigitalIoPin objects just for their constructors
 * but I'm not entirely sure I even need them...?
 * Seems to run fine without them... I'll keep them here though,
 * commented out. - Simo */

/* kisko / onboard napit */
//DigitalIoPin sw1(0, 17, DigitalIoPin::pullup, true);
//DigitalIoPin sw3(1, 9, DigitalIoPin::pullup, true);
//DigitalIoPin x1(0, 27, DigitalIoPin::pullup, true);
//DigitalIoPin x2(0, 28, DigitalIoPin::pullup, true);

// TODO: muista valita oikeet jutut
/* oikeet jutut */
//DigitalIoPin sw1asd(0, 8, DigitalIoPin::pullup, true);
//DigitalIoPin sw2asd(1, 6, DigitalIoPin::pullup, true);
//DigitalIoPin x1asd(0, 29, DigitalIoPin::pullup, true);
//DigitalIoPin x2asd(0, 9, DigitalIoPin::pullup, true);
//DigitalIoPin y1asd(1, 3, DigitalIoPin::pullup, true);
//DigitalIoPin y2asd(0, 0, DigitalIoPin::pullup, true);



extern "C" {
void PIN_INT0_IRQHandler(void);
void PIN_INT1_IRQHandler(void);
void PIN_INT2_IRQHandler(void);
void PIN_INT3_IRQHandler(void);
void PIN_INT4_IRQHandler(void);
void PIN_INT5_IRQHandler(void);
}

void dtaskHardStop(void *pvParameters);

void GPIO_interrupt_init(void);

#endif /* SW_BTN_INTERRUPTS_H_ */
