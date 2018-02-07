#include "myMutex.h"

myMutex::myMutex() {
	 mutex = xSemaphoreCreateMutex();
}

myMutex::~myMutex() {
	// TODO Auto-generated destructor stub
}

void myMutex::lock() {
	xSemaphoreTake(mutex, portMAX_DELAY);
}

void myMutex::unlock() {
	 xSemaphoreGive(mutex);
}
