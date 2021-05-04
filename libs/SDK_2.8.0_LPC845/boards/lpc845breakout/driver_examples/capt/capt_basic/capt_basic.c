/*
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "fsl_capt.h"
#include "fsl_debug_console.h"

#include "pin_mux.h"
#include <stdbool.h>
#include "fsl_power.h"
#include "fsl_gpio.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_CAPT_BASE             CAPT
#define DEMO_CAPT_NOISE            0U
#define DEMO_CAPT_IRQ_NUMBER       CMP_CAPT_IRQn
#define DEMO_CAPT_IRQ_HANDLER_FUNC CMP_CAPT_DriverIRQHandler
#define DEMO_CAPT_ENABLE_PINS      kCAPT_X0Pin
#define DEMO_CAPT_FILTER_NUM 10U
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_InitLED(void);
void BOARD_LED_ON(uint32_t index);
void BOARD_LED_OFF(void);

static uint32_t CAPT_DoCalibration(CAPT_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint32_t g_value[16] = {0U};
capt_touch_data_t g_data;
volatile bool g_YesTouchFlag;
/*******************************************************************************
 * Code
 ******************************************************************************/

void BOARD_InitLED(void)
{
    LED_GREEN_INIT(LOGIC_LED_OFF);
}

void BOARD_LED_ON(uint32_t index)
{
    switch (index)
    {
        case 0U:
            LED_GREEN_ON();
            break;
        default:
            break;
    }
}

void BOARD_LED_OFF(void)
{
    LED_GREEN_OFF();
}
/*!
 * @brief ISR for CAPT.
 */
void DEMO_CAPT_IRQ_HANDLER_FUNC(void)
{
    uint32_t n, mask, largest = 0U;
    static uint32_t filter_index;
    static uint8_t largests[DEMO_CAPT_FILTER_NUM];

    CAPT_GetTouchData(DEMO_CAPT_BASE, &g_data);
    g_value[g_data.XpinsIndex] = g_data.count;
    mask                       = CAPT_GetInterruptStatusFlags(DEMO_CAPT_BASE);
    CAPT_ClearInterruptStatusFlags(DEMO_CAPT_BASE, mask);
    if (mask & kCAPT_InterruptOfYesTouchEnable)
    {
        g_YesTouchFlag = true;
    }
    if (mask & kCAPT_InterruptOfPollDoneEnable)
    {
        if (g_YesTouchFlag)
        {
            for (n = 0; n < CAPT_GET_XMAX_NUMBER(DEMO_CAPT_BASE->STATUS); n++)
            {
                if (g_value[n + 1] > g_value[largest])
                {
                    largest = n + 1;
                }
            }
            largests[filter_index++] = largest;
            if (filter_index == DEMO_CAPT_FILTER_NUM)
            {
                filter_index = 0U;
            }
            for (n = 0; n < DEMO_CAPT_FILTER_NUM - 1; n++)
            {
                if (largests[n] != largests[n + 1])
                {
                    g_YesTouchFlag = false;
                    break;
                }
            }
            if (g_YesTouchFlag)
            {
                BOARD_LED_ON(largest);
                g_YesTouchFlag = false;
            }
            else
            {
                BOARD_LED_OFF();
            }
        }
        else
        {
            for (n = 0; n < CAPT_GET_XMAX_NUMBER(DEMO_CAPT_BASE->STATUS); n++)
            {
                largests[n] = n;
            }
            BOARD_LED_OFF();
        }
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    uint32_t thresold;
    capt_config_t captConfig;

    /* Initialize board hardware. */
    /* Attach main clock to CAPT */
    CLOCK_Select(kCAPT_Clk_From_Fro);
    POWER_DisablePD(kPDRUNCFG_PD_ACMP);
    /* Select the main clock as source clock of USART0 (debug console) */
    CLOCK_Select(BOARD_DEBUG_USART_CLK_ATTACH);

    BOARD_InitPins();
    BOARD_BootClockFRO30M();
    BOARD_InitDebugConsole();
    BOARD_InitLED();

    PRINTF("CAPT basic example.\r\n");

    /* Initialize CAPT module. */
    CAPT_GetDefaultConfig(&captConfig);

    /* Calculate the clock divider to make sure CAPT work in 2Mhz fclk. */
    captConfig.clockDivider = (CLOCK_GetFroFreq() / 2000000U - 1U);
    captConfig.enableXpins  = DEMO_CAPT_ENABLE_PINS;
    CAPT_Init(DEMO_CAPT_BASE, &captConfig);

    /* Do the self-calibration. */
    thresold = CAPT_DoCalibration(DEMO_CAPT_BASE);
    CAPT_SetThreshold(DEMO_CAPT_BASE, thresold - DEMO_CAPT_NOISE);
    PRINTF("Calibration has finished.\r\n");
    BOARD_LED_OFF();

    /* Enable the interrupts. */
    CAPT_EnableInterrupts(DEMO_CAPT_BASE, kCAPT_InterruptOfYesTouchEnable | kCAPT_InterruptOfNoTouchEnable |
                                              kCAPT_InterruptOfPollDoneEnable);
    NVIC_EnableIRQ(DEMO_CAPT_IRQ_NUMBER);

    /* Set polling mode and start poll. */
    CAPT_SetPollMode(DEMO_CAPT_BASE, kCAPT_PollContinuousMode);

    while (1)
    {
    }
}

static uint32_t CAPT_DoCalibration(CAPT_Type *base)
{
    uint16_t xpinIndex, repeatCounter;
    uint32_t temp_count, temp_count_high, temp_count_low;
    uint32_t temp_xPins, max_xpins;
    uint32_t average_count[16] = {0U};

    max_xpins  = CAPT_GET_XMAX_NUMBER(CAPT_GetStatusFlags(base));
    temp_xPins = base->CTRL & CAPT_CTRL_XPINSEL_MASK;
    if (base->POLL_TCNT & CAPT_POLL_TCNT_TCHLOW_ER_MASK)
    {
        temp_count = 0xFFFU;
    }
    else
    {
        temp_count = 0U;
    }
    for (xpinIndex = 0U; xpinIndex <= max_xpins; xpinIndex++)
    {
        if (temp_xPins & (1U << (CAPT_CTRL_XPINSEL_SHIFT + xpinIndex)))
        {
            /* Before writing into CTRL register, INCHANGE(bit 15)should equal '0'. */
            while (CAPT_CTRL_INCHANGE_MASK == (CAPT_CTRL_INCHANGE_MASK & base->CTRL))
            {
            }
            base->CTRL = (base->CTRL & ~CAPT_CTRL_XPINSEL_MASK) | 1U << (CAPT_CTRL_XPINSEL_SHIFT + xpinIndex);
            for (repeatCounter = 0U; repeatCounter < 100U; ++repeatCounter)
            {
                /* Before writing into CTRL register, INCHANGE(bit 15)should equal '0'. */
                while (CAPT_CTRL_INCHANGE_MASK == (CAPT_CTRL_INCHANGE_MASK & base->CTRL))
                {
                }
                /* Start poll-now mode. */
                base->CTRL &= ~CAPT_CTRL_POLLMODE_MASK;
                base->CTRL |= CAPT_CTRL_POLLMODE(kCAPT_PollNowMode);
                /* Wait for poll-now done. */
                while (!(base->STATUS & CAPT_STATUS_POLLDONE_MASK))
                {
                }
                /* Clear the status flags. */
                base->STATUS |= base->STATUS;
                average_count[xpinIndex] += ((base->TOUCH & CAPT_TOUCH_COUNT_MASK) >> CAPT_TOUCH_COUNT_SHIFT);
            }
            average_count[xpinIndex] /= 100U;
        }
    }
    /* Restore the xpins. */
    base->CTRL = (base->CTRL & ~CAPT_CTRL_XPINSEL_MASK) | temp_xPins;

    temp_count_high = 0U;
    temp_count_low  = 0xFFFU;
    for (xpinIndex = 0U; xpinIndex <= max_xpins; xpinIndex++)
    {
        if (temp_xPins & (1U << (CAPT_CTRL_XPINSEL_SHIFT + xpinIndex)))
        {
            if (temp_count_high < average_count[xpinIndex])
            {
                temp_count_high = average_count[xpinIndex];
            }
            if (temp_count_low > average_count[xpinIndex])
            {
                temp_count_low = average_count[xpinIndex];
            }
        }
    }
    /* For touchlower mode.*/
    if (base->POLL_TCNT & CAPT_POLL_TCNT_TCHLOW_ER_MASK)
    {
        temp_count = 2 * temp_count_low - temp_count_high;
    }
    else /* For touchhiger mode. */
    {
        temp_count = 2 * temp_count_high - temp_count_low;
    }

    return temp_count;
}
