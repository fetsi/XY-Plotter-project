#ifndef MYMUTEX_H_
#define MYMUTEX_H_

#include "FreeRTOS.h"
#include <mutex>
#include "semphr.h"

class myMutex {
public:
	myMutex();
	virtual ~myMutex();

	void lock();
	void unlock();

private:
	xSemaphoreHandle mutex;
};

#endif /* MYMUTEX_H_ */


