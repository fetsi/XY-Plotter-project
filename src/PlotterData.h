/*
 * PlotterData.h
 *
 *  Created on: 21.10.2017
 *      Author: micromikko
 */

/*
 * the very newest of tests
 */

#ifndef PLOTTERDATA_H_
#define PLOTTERDATA_H_


class PlotterData {
public:

	enum plotterOpMode {
		none,
		pen,
		laser
	};

	PlotterData(plotterOpMode kayYouElElEye = none, int axisX=380, int axisY=310);

	virtual ~PlotterData();

	int plotterMode;

	bool dirX;
	bool dirY;

	int axisLengthX;		// mm
	int axisLengthY;		// mm

	int stepdelay_min;		//=200;
	int stepdelay_max;		//=1000;

	/*
	 * Set during caribouration
	 */
	int axisStepCountX;		// steps
	int axisStepCountY;		// steps

	// #define STEPS_PER_MM 87.58
	// SHOULD BE THE SAME, SO
	double stepsPerMM;
//	int stepsPerMMX;		// steps
//	int stepsPerMMY;		// steps


	/*
	 * Current
	 */
	double currentX;		// mm
	double currentY;		// mm

	int currentStepsX;		// steps
	int currentStepsY;		// steps

	int currentPenPos;		// pwm
	int currentLaserPos;	// pwm

	/*
	 * CommandPacket
	 */
	char gorm;				// G or M
	int gormNum;			// G: 1, 28; M: 1, 4, 10

	double targetX;			// mm
	double targetY;			// mm

	double targetStepsX;	// steps
	double targetStepsY;	// steps

	int targetPen;			// pwm
	int targetLaser;		// pwm

	long auxDelay;			// us

	double dX;				// mm
	double dY;				// mm

	int dStepsX;			// steps
	int dStepsY;			// steps

	int dStepsMax;			// steps

	double stepIntervalX;	// fraction of step
	double stepIntervalY;	// fraction of step

	void calculateStepsPerMM();
	int convertToSteps(const double before);

//	CommandPacket *compack;
	void resetCompack();
private:

};

#endif /* PLOTTERDATA_H_ */

