/*
 * Servo.h
 *
 *  Created on: Oct 23, 2017
 *      Author: Tuomas-laptop
 */

#ifndef SRC_SERVO_H_
#define SRC_SERVO_H_

#include "DigitalIoPin.h"

class Servo {

private:
	DigitalIoPin *pin;
	int portNum;
	int pinNum;
	const int penUp_cycle_length = 1300;
	const int penDown_cycle_length = 1450;
	void init();
public:
	Servo(int portNum, int pinNum);
	virtual ~Servo();
	void penDown();
	void penUp();

};

#endif /* SRC_SERVO_H_ */
