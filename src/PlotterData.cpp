/*
 * PlotterData.cpp
 *
 *  Created on: 21.10.2017
 *      Author: micromikko
 */

#include "PlotterData.h"


PlotterData::PlotterData(plotterOpMode mode, int axisX, int axisY) {
	switch(mode) {
		case none:
			this->currentPenPos = 0;
			this->currentLaserPos = 0;
			break;
		case pen:
			this->currentPenPos = 130;
			this->currentLaserPos = 0;
			break;
		case laser:
			this->currentPenPos = 0;
			this->currentLaserPos = 0;
			break;
	}

	this->dirX = false;
	this->dirY = false;

	this->axisLengthX = axisX;		// mm
	this->axisLengthY = axisY;		// mm

	this->stepdelay_min = 0;		//=200;
	this->stepdelay_max = 0;		//=1000;

	/*
	 * Set during calibration
	 */
	this->axisStepCountX = 0;		// steps
	this->axisStepCountY = 0;		// steps

	// #define STEPS_PER_MM 87.58
	// SHOULD BE THE SAME, SO
	this->stepsPerMM = 87.58; // -.,-.,-.,


	/*
	 * Current
	 */
	this->currentX = 0;		// mm
	this->currentY = 0;		// mm

	this->currentStepsX = 0;		// steps
	this->currentStepsY = 0;		// steps

	/*
	 * CommandPacket
	 */
	this->gorm = 0;				// G or M
	this->gormNum = 0;			// G: 1, 28; M: 1, 4, 10

	this->targetX = 0;			// mm
	this->targetY = 0;			// mm

	this->targetStepsX = 0;	// steps
	this->targetStepsY = 0;	// steps

	this->targetPen = 0;		// pwm
	this->targetLaser = 0;		// pwm

	this->auxDelay = 0;			// us

	this->dX = 0;				// mm
	this->dY = 0;				// mm

	this->dStepsX = 0;			// steps
	this->dStepsY = 0;			// steps

	this->dStepsMax = 0;		// steps

	this->stepIntervalX = 0;	// fraction of step
	this->stepIntervalY = 0;	// fraction of step
}

PlotterData::~PlotterData() {
	// TODO Auto-generated destructor stub
}

void PlotterData::calculateStepsPerMM() {
	int stepsPerMMX = this->axisStepCountX / this->axisLengthX;
	int stepsPerMMY = this->axisStepCountY / this->axisLengthY;

	if(stepsPerMMX == stepsPerMMY) {
		this->stepsPerMM = stepsPerMMX;
	} else {
		while(1);
	}
}

void PlotterData::resetCompack() {
	this->gorm = 0;
	this->gormNum = 0;

	this->targetX = 0;
	this->targetY = 0;
	this->auxDelay = 0;

	this->targetPen = 0;
	this->targetLaser = 0;
}

int PlotterData::convertToSteps(const double before) {
	int steps = (int) (before * this->stepsPerMM);
	return steps;
}

