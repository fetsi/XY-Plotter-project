/*
 * Driver.h
 *
 *  Created on: 23.10.2017
 *      Author: micromikko
 */

#ifndef DRIVER_H_
#define DRIVER_H_

#include "PlotterData.h"

extern "C" { void RIT_IRQHandler(void); }
void RIT_start(int count, int us);
void RIT_init();

void taskExecute(void *pvParameters);
void calculateDrive(PlotterData &pd);
void justDrive(PlotterData &pd);
void dtaskMotor(void *pvParameters);


#endif /* DRIVER_H_ */
