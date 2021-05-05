/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "board_api.h"
#include "pin_mux.h"
#include "FreeRTOS.h"
#include "task.h"
#include "StaticAllocation.h"
#include "hooks.h"

#define STACK_SIZE 200
#define TASK_COUNT
 /* good place to look if dynamic allocations aren't working as expected... */
void *malloc (size_t sz) {return NULL; }
void free(void*ptr) { }

StaticTask_t xLEDTask0Buffer;
StaticTask_t xLEDTask1Buffer;
StaticTask_t xLEDTask2Buffer;

StackType_t xStack0[ configMINIMAL_STACK_SIZE ];
StackType_t xStack1[ configMINIMAL_STACK_SIZE ];
StackType_t xStack2[ configMINIMAL_STACK_SIZE ];
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* LED0 toggle thread */
static void vLEDTask0 (void *pvParameters) {
	while (1) {
		LED_BLUE_TOGGLE();
		vTaskDelay(configTICK_RATE_HZ/2);
	}
}

/* LED1 toggle thread */
static void vLEDTask1 (void *pvParameters) {
	while (1) {
		LED_GREEN_TOGGLE();
		vTaskDelay(configTICK_RATE_HZ*2);
	}
}

/* LED2 toggle thread */
static void vLEDTask2 (void *pvParameters) {
	while (1) {
		LED_RED_TOGGLE();
		vTaskDelay(configTICK_RATE_HZ);
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	main routine for FreeRTOS blinky example
 * @return	Nothing, function should not exit
 */
int main(void)
{
    /* Board pin init */
    BOARD_InitPins();
    BOARD_InitBootClocks();
    LED_BLUE_INIT(1);
    LED_RED_INIT(1);
    LED_GREEN_INIT(1);

  xTaskCreateStatic(vLEDTask0, "vLEDTask0", configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL), xStack0, &xLEDTask0Buffer);
  xTaskCreateStatic(vLEDTask1, "vLEDTask1", configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL), xStack1, &xLEDTask1Buffer);
  xTaskCreateStatic(vLEDTask2, "vLEDTask2", configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL), xStack2, &xLEDTask2Buffer);

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}

void vApplicationIdleHook( void )
{
const unsigned long ulMSToSleep = 5;

	/* This function is called on each cycle of the idle task if
	configUSE_IDLE_HOOK is set to 1 in FreeRTOSConfig.h.  Sleep to reduce CPU
	load. */
	//while(1){}//Sleep( ulMSToSleep );
}

