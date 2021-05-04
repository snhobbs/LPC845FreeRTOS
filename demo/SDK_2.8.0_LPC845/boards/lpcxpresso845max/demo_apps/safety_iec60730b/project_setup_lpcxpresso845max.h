/*
 * Copyright 2019 NXP.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PROJECT_SETUP_H_
#define _PROJECT_SETUP_H_

#include "safety_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @name Project setup functions
 * @{
 */
/*******************************************************************************
 * API
 ******************************************************************************/
void WatchdogEnable(void);
void WatchdogDisable(void);
void CTIMERInit(void);
void ReferenceTimerInit(void); /* Second timer for CLOCK TEST */
void SystickInit(uint32_t reloadValue);
void ClockInit(void);
void PortInit(uint8_t *pByte, uint32_t *pDir, uint32_t *pIocon, uint32_t pinDir, uint32_t pinNum, uint32_t pull, uint32_t clockEnableShift);
void AdcInit(void);
void SerialInit(USART_Type *Uart_X, uint32_t brate, uint64_t bclk);

 #ifdef __cplusplus
}
#endif

#endif /* _PROJECT_SETUP_H_ */
