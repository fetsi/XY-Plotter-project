#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

// TODO: insert other include files here
#include <string>
#include <cstring>
#include <cstdlib>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "ITM_write.h"
#include "myMutex.h"
#include "DigitalIoPin.h"

#include "Handles.h"
#include "uart_module.h"
#include "sw_btn_interrupts.h"

#include "PlotterData.h"
#include "Driver.h"
#include "Parser.h"
#include "Servo.h"


extern "C" {
	void vConfigureTimerForRunTimeStats( void ) {
		Chip_SCT_Init(LPC_SCTSMALL1);
		LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
		LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
	}
}

static void prvSetupHardware(void) {
	SystemCoreClockUpdate();
	Board_Init();
}


int main(void) {
	prvSetupHardware();
	ITM_init();

	Handles *commonHandles = new Handles;
	commonHandles->commandQueue_raw = xQueueCreate(1, sizeof(std::string*));
	commonHandles->readyToReceive = xSemaphoreCreateBinary();

	/*
	 * tasks
	 */
	xTaskCreate(taskExecute, "taskExecute", 500, (void*) commonHandles, (tskIDLE_PRIORITY + 1UL), NULL);
	xTaskCreate(taskSendOK, "taskSendOK", 256, (void*) commonHandles, (tskIDLE_PRIORITY + 4UL), NULL);
	
	/*
	 * dtasks (tasks to which the execution is deferred from interrupt service routines)
	 */
	xTaskCreate(dtaskUARTReader, "UARTReaderdTask", 256, (void*) commonHandles, (tskIDLE_PRIORITY +3UL), NULL);
	xTaskCreate(dtaskMotor, "motordTask", 200, NULL, (tskIDLE_PRIORITY + 2UL), NULL); // keep at highest priority!

	xSemaphoreGive(commonHandles->readyToReceive);			 //This has to be initially available, so the first draw command can be processed

	UARTModule_init();
	GPIO_interrupt_init();
	RIT_init();

	vTaskStartScheduler();

	//Since the FREERTOS scheduler is started, execution should never reach this point unless something is wrong
	for(;;);

    return 0 ;
}

