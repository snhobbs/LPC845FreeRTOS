/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_ctimer.h"

#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CTIMER          CTIMER0         /* Timer 0 */
#define CTIMER_MAT_OUT  kCTIMER_Match_1 /* Match output 1 */
#define CTIMER_CLK_FREQ CLOCK_GetFreq(kCLOCK_CoreSysClk)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint32_t g_pwmPeriod   = 0U;
volatile uint32_t g_pulsePeriod = 0U;

/*******************************************************************************
 * Code
 ******************************************************************************/
status_t CTIMER_GetPwmPeriodValue(uint32_t pwmFreqHz, uint8_t dutyCyclePercent, uint32_t timerClock_Hz)
{
    /* Calculate PWM period match value */
    g_pwmPeriod = (timerClock_Hz / pwmFreqHz) - 1U;

    /* Calculate pulse width match value */
    g_pulsePeriod = (g_pwmPeriod + 1U) * (100 - dutyCyclePercent) / 100;

    return kStatus_Success;
}

/*!
 * @brief Main function
 */
int main(void)
{
    ctimer_config_t config;
    uint32_t srcClock_Hz;
    uint32_t timerClock;

    /* Init hardware*/
    /* Attach 12 MHz clock to USART0 (debug console) */
    CLOCK_Select(BOARD_DEBUG_USART_CLK_ATTACH);

    BOARD_InitPins();
    BOARD_BootClockFRO30M();
    BOARD_InitDebugConsole();

    /* Turn on LED RED */
    LED_RED_INIT(LOGIC_LED_ON);

    /* CTimer0 counter uses the AHB clock, some CTimer1 modules use the Aysnc clock */
    srcClock_Hz = CTIMER_CLK_FREQ;

    PRINTF("CTimer example to generate a PWM signal\r\n");

    CTIMER_GetDefaultConfig(&config);
    timerClock = srcClock_Hz / (config.prescale + 1);

    CTIMER_Init(CTIMER, &config);

    /* Get the PWM period match value and pulse width match value of 20Khz PWM signal with 20% dutycycle */
    CTIMER_GetPwmPeriodValue(20000, 20, timerClock);
    CTIMER_SetupPwmPeriod(CTIMER, CTIMER_MAT_OUT, g_pwmPeriod, g_pulsePeriod, false);
    CTIMER_StartTimer(CTIMER);

    while (1)
    {
    }
}