/*
 * Copyright  2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v5.0
processor: LPC845
package_id: LPC845M301JBD48
mcu_data: ksdk2_0
processor_version: 5.0.0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_iocon.h"
#include "fsl_swm.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '20', peripheral: USART0, signal: RXD, pin_signal: PIO0_24, mode: pullUp, invert: disabled, hysteresis: enabled, opendrain: disabled, smode: bypass,
    clkdiv: div0}
  - {pin_num: '19', peripheral: USART0, signal: TXD, pin_signal: PIO0_25, mode: pullUp, invert: disabled, hysteresis: enabled, opendrain: disabled, smode: bypass,
    clkdiv: div0}
  - {pin_num: '1', peripheral: CAPT, signal: CAPTYL, pin_signal: PIO1_8/CAPT_YL, mode: inactive, invert: disabled, hysteresis: enabled, opendrain: disabled, smode: bypass,
    clkdiv: div0}
  - {pin_num: '3', peripheral: CAPT, signal: CAPTYH, pin_signal: PIO1_9/CAPT_YH, mode: inactive, invert: disabled, hysteresis: enabled, opendrain: disabled, smode: bypass,
    clkdiv: div0}
  - {pin_num: '9', peripheral: CAPT, signal: 'CAPTX, 0', pin_signal: PIO0_31/CAPT_X0, mode: inactive, invert: disabled, hysteresis: enabled, opendrain: disabled,
    smode: bypass, clkdiv: div0}
  - {pin_num: '11', peripheral: GPIO, signal: 'PIO1, 0', pin_signal: PIO1_0/CAPT_X1, mode: inactive, invert: disabled, hysteresis: enabled, opendrain: disabled, smode: bypass,
    clkdiv: div0}
  - {pin_num: '42', peripheral: ACMP, signal: ACMP_IN5, pin_signal: PIO0_30/ACMP_I5, mode: inactive, invert: disabled, hysteresis: enabled, opendrain: disabled, smode: bypass,
    clkdiv: div0}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M0P */
void BOARD_InitPins(void)
{
    /* Enables clock for IOCON.: enable */
    CLOCK_EnableClock(kCLOCK_Iocon);
    /* Enables clock for switch matrix.: enable */
    CLOCK_EnableClock(kCLOCK_Swm);

    const uint32_t IOCON_INDEX_PIO0_24_config = (/* Selects pull-up function */
                                                 IOCON_PIO_MODE_PULLUP |
                                                 /* Enable hysteresis */
                                                 IOCON_PIO_HYS_EN |
                                                 /* Input not invert */
                                                 IOCON_PIO_INV_DI |
                                                 /* Disables Open-drain function */
                                                 IOCON_PIO_OD_DI |
                                                 /* Bypass input filter */
                                                 IOCON_PIO_SMODE_BYPASS |
                                                 /* IOCONCLKDIV0 */
                                                 IOCON_PIO_CLKDIV0);
    /* PORT0 PIN24 (coords: ) is configured as  */
    IOCON_PinMuxSet(IOCON, IOCON_INDEX_PIO0_24, IOCON_INDEX_PIO0_24_config);

    const uint32_t IOCON_INDEX_PIO0_25_config = (/* Selects pull-up function */
                                                 IOCON_PIO_MODE_PULLUP |
                                                 /* Enable hysteresis */
                                                 IOCON_PIO_HYS_EN |
                                                 /* Input not invert */
                                                 IOCON_PIO_INV_DI |
                                                 /* Disables Open-drain function */
                                                 IOCON_PIO_OD_DI |
                                                 /* Bypass input filter */
                                                 IOCON_PIO_SMODE_BYPASS |
                                                 /* IOCONCLKDIV0 */
                                                 IOCON_PIO_CLKDIV0);
    /* PORT0 PIN25 (coords: ) is configured as  */
    IOCON_PinMuxSet(IOCON, IOCON_INDEX_PIO0_25, IOCON_INDEX_PIO0_25_config);

    const uint32_t IOCON_INDEX_PIO0_30_config = (/* No addition pin function */
                                                 IOCON_PIO_MODE_INACT |
                                                 /* Enable hysteresis */
                                                 IOCON_PIO_HYS_EN |
                                                 /* Input not invert */
                                                 IOCON_PIO_INV_DI |
                                                 /* Disables Open-drain function */
                                                 IOCON_PIO_OD_DI |
                                                 /* Bypass input filter */
                                                 IOCON_PIO_SMODE_BYPASS |
                                                 /* IOCONCLKDIV0 */
                                                 IOCON_PIO_CLKDIV0);
    /* PORT0 PIN30 (coords: ) is configured as  */
    IOCON_PinMuxSet(IOCON, IOCON_INDEX_PIO0_30, IOCON_INDEX_PIO0_30_config);

    const uint32_t IOCON_INDEX_PIO0_31_config = (/* No addition pin function */
                                                 IOCON_PIO_MODE_INACT |
                                                 /* Enable hysteresis */
                                                 IOCON_PIO_HYS_EN |
                                                 /* Input not invert */
                                                 IOCON_PIO_INV_DI |
                                                 /* Disables Open-drain function */
                                                 IOCON_PIO_OD_DI |
                                                 /* Bypass input filter */
                                                 IOCON_PIO_SMODE_BYPASS |
                                                 /* IOCONCLKDIV0 */
                                                 IOCON_PIO_CLKDIV0);
    /* PORT0 PIN31 (coords: ) is configured as  */
    IOCON_PinMuxSet(IOCON, IOCON_INDEX_PIO0_31, IOCON_INDEX_PIO0_31_config);

    const uint32_t IOCON_INDEX_PIO1_0_config = (/* No addition pin function */
                                                IOCON_PIO_MODE_INACT |
                                                /* Enable hysteresis */
                                                IOCON_PIO_HYS_EN |
                                                /* Input not invert */
                                                IOCON_PIO_INV_DI |
                                                /* Disables Open-drain function */
                                                IOCON_PIO_OD_DI |
                                                /* Bypass input filter */
                                                IOCON_PIO_SMODE_BYPASS |
                                                /* IOCONCLKDIV0 */
                                                IOCON_PIO_CLKDIV0);
    /* PORT1 PIN0 (coords: ) is configured as  */
    IOCON_PinMuxSet(IOCON, IOCON_INDEX_PIO1_0, IOCON_INDEX_PIO1_0_config);

    const uint32_t IOCON_INDEX_PIO1_8_config = (/* No addition pin function */
                                                IOCON_PIO_MODE_INACT |
                                                /* Enable hysteresis */
                                                IOCON_PIO_HYS_EN |
                                                /* Input not invert */
                                                IOCON_PIO_INV_DI |
                                                /* Disables Open-drain function */
                                                IOCON_PIO_OD_DI |
                                                /* Bypass input filter */
                                                IOCON_PIO_SMODE_BYPASS |
                                                /* IOCONCLKDIV0 */
                                                IOCON_PIO_CLKDIV0);
    /* PORT1 PIN8 (coords: ) is configured as  */
    IOCON_PinMuxSet(IOCON, IOCON_INDEX_PIO1_8, IOCON_INDEX_PIO1_8_config);

    const uint32_t IOCON_INDEX_PIO1_9_config = (/* No addition pin function */
                                                IOCON_PIO_MODE_INACT |
                                                /* Enable hysteresis */
                                                IOCON_PIO_HYS_EN |
                                                /* Input not invert */
                                                IOCON_PIO_INV_DI |
                                                /* Disables Open-drain function */
                                                IOCON_PIO_OD_DI |
                                                /* Bypass input filter */
                                                IOCON_PIO_SMODE_BYPASS |
                                                /* IOCONCLKDIV0 */
                                                IOCON_PIO_CLKDIV0);
    /* PORT1 PIN9 (coords: ) is configured as  */
    IOCON_PinMuxSet(IOCON, IOCON_INDEX_PIO1_9, IOCON_INDEX_PIO1_9_config);

    /* USART0_TXD connect to P0_25 */
    SWM_SetMovablePinSelect(SWM0, kSWM_USART0_TXD, kSWM_PortPin_P0_25);

    /* USART0_RXD connect to P0_24 */
    SWM_SetMovablePinSelect(SWM0, kSWM_USART0_RXD, kSWM_PortPin_P0_24);

    /* ACMP_INPUT5 connect to P0_30 */
    SWM_SetFixedPinSelect(SWM0, kSWM_ACMP_INPUT5, true);

    /* CAPT_X0 connect to P0_31 */
    SWM_SetFixedPinSelect(SWM0, kSWM_CAPT_X0, true);

    /* CAPT_YL connect to P1_8 */
    SWM_SetFixedPinSelect(SWM0, kSWM_CAPT_YL, true);

    /* CAPT_YH connect to P1_9 */
    SWM_SetFixedPinSelect(SWM0, kSWM_CAPT_YH, true);

    /* Disable clock for switch matrix. */
    CLOCK_DisableClock(kCLOCK_Swm);
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
