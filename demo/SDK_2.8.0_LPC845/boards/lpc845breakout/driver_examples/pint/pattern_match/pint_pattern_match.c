/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_pint.h"

#include "pin_mux.h"
#include "fsl_syscon.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_PINT_BSLICE0_SRC kPINT_PatternMatchInp0Src
#define DEMO_PINT_BSLICE1_SRC kPINT_PatternMatchInp0Src
#define DEMO_PINT_BSLICE2_SRC kPINT_PatternMatchInp1Src

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Call back for PINT Pin interrupt 0-7.
 */
void pint_intr_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
    PRINTF("\f\r\nPINT Pin Interrupt %d event detected. PatternMatch status = %8b\r\n", pintr, pmatch_status);
}

/*!
 * @brief Main function
 */
int main(void)
{
    pint_pmatch_cfg_t pmcfg;

    /* Board pin, clock, debug console init */
    /* Select the main clock as source clock of USART0 (debug console) */
    CLOCK_Select(BOARD_DEBUG_USART_CLK_ATTACH);

    BOARD_InitPins();
    BOARD_BootClockFRO30M();
    BOARD_InitDebugConsole();

    /* Connect trigger sources to PINT */
    SYSCON_AttachSignal(SYSCON, kPINT_PatternMatchInp0Src, kSYSCON_GpioPort0Pin12ToPintsel);
    SYSCON_AttachSignal(SYSCON, kPINT_PatternMatchInp1Src, kSYSCON_GpioPort0Pin4ToPintsel);

    /* Turn on LED RED */
    LED_RED_INIT(LOGIC_LED_ON);

    /* Clear screen*/
    PRINTF("%c[2J", 27);
    /* Set cursor location at [0,0] */
    PRINTF("%c[0;0H", 27);
    PRINTF("\f\r\nPINT Pattern Match example\r\n");

    /* Initialize PINT */
    PINT_Init(PINT);

    /* configure kPINT_PatternMatchBSlice0 to show the single inputsrc */
    /* Setup Pattern Match Bit Slice 0 */
    pmcfg.bs_src    = DEMO_PINT_BSLICE0_SRC;
    pmcfg.bs_cfg    = kPINT_PatternMatchStickyFall;
    pmcfg.callback  = pint_intr_callback;
    pmcfg.end_point = true;
    PINT_PatternMatchConfig(PINT, kPINT_PatternMatchBSlice0, &pmcfg);

    /* Enable callbacks for PINT0 by Index */
    PINT_EnableCallbackByIndex(PINT, kPINT_PinInt0);

    /* configure kPINT_PatternMatchBSlice1 and kPINT_PatternMatchBSlice2 to show the combined inputsrc */
#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 1U)
    /* Setup Pattern Match Bit Slice 1 */
    pmcfg.bs_src    = DEMO_PINT_BSLICE1_SRC;
    pmcfg.bs_cfg    = kPINT_PatternMatchStickyRise;
    pmcfg.callback  = pint_intr_callback;
    pmcfg.end_point = false;
    PINT_PatternMatchConfig(PINT, kPINT_PatternMatchBSlice1, &pmcfg);
#endif

#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 2U)
    /* Setup Pattern Match Bit Slice 2 for falling edge detection */
    pmcfg.bs_src    = DEMO_PINT_BSLICE2_SRC;
    pmcfg.bs_cfg    = kPINT_PatternMatchStickyRise;
    pmcfg.callback  = pint_intr_callback;
    pmcfg.end_point = true;
    PINT_PatternMatchConfig(PINT, kPINT_PatternMatchBSlice2, &pmcfg);

    /* Enable callbacks for PINT2 by Index */
    PINT_EnableCallbackByIndex(PINT, kPINT_PinInt2);
#endif

    /* Enable PatternMatch */
    PINT_PatternMatchEnable(PINT);

    PRINTF("\r\nPINT Pattern match events are configured\r\n");
    PRINTF("\r\nPress corresponding switches to generate events\r\n");
    while (1)
    {
        __WFI();
    }
}
