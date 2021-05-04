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

	/* LED1 toggle thread */
	xTaskCreate(vLEDTask1, "vTaskLed1",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

	/* LED2 toggle thread */
	xTaskCreate(vLEDTask2, "vTaskLed2",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

	/* LED0 toggle thread */
	xTaskCreate(vLEDTask0, "vTaskLed0",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}


/**
 * Override stack overflow function so that it won't use printf [DEBUGOUT]
 */
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	DEBUGSTR("Stack Over flow in :");
	DEBUGSTR(pcTaskName);
	/* Run time stack overflow checking is performed if
	   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	   function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for (;; ) {}
}

void vApplicationIdleHook( void )
{
const unsigned long ulMSToSleep = 5;

	/* This function is called on each cycle of the idle task if
	configUSE_IDLE_HOOK is set to 1 in FreeRTOSConfig.h.  Sleep to reduce CPU
	load. */
	//while(1){}//Sleep( ulMSToSleep );
}

void vApplicationMallocFailedHook(void) {
	while(1) {}
}
