/*
 * Handles.h
 *
 *  Created on: Oct 23, 2017
 *      Author: Tuomas-laptop
 */

#ifndef SRC_HANDLES_H_
#define SRC_HANDLES_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

typedef struct {
	QueueHandle_t commandQueue_raw;		    //queue of length 1 to communicate raw input from UART to command parser
	SemaphoreHandle_t readyToReceive;		//semaphore to signal UART task that there is room in the command queue, we can send "OK" back to mDraw
}Handles;



#endif /* SRC_HANDLES_H_ */
