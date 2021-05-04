/*
 * Copyright 2019 NXP.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _SAFETY_CONFIG_H_
#define _SAFETY_CONFIG_H_

#include "LPC845.h"
#include "iec60730b.h"
#include "project_setup_lpcxpresso845max.h"
#include "safety_cm0_lpc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* This macro enables infinity while loop in SafetyErrorHandling() function */
#define SAFETY_ERROR_ACTION 1

/* TEST SWITCHES - for debugging it is better to turn the flash test and watchdog OFF */
#define ADC_TEST_ENABLED   0
#define CLOCK_TEST_ENABLED 0
#define DIO_TEST_ENABLED   1
#define FLASH_TEST_ENABLED 0
#define PC_TEST_ENABLED    1
#define WATCHDOG_ENABLED   0

#define CHANNEL_ZERO 0  /* use Chanel 0 for MRT timer- for clarity */

/* CLOCK test */
#define REF_TIMER_USED             WKT
#define REF_TIMER_CLOCK_FREQUENCY  10e03
#define SYSTICK_RELOAD_VALUE       30000
#define ISR_FREQUENCY              1000 /* Hz */
#define START_VALUE                10000 /* Start value for WKT timer, this timer decreasing */
#define CLOCK_TEST_TOLERANCE       100  /* % */

/*WDOG test */
#define WDOG_REF_TIMER_BASE     CTIMER0
#define USED_WDOG               WWDT
#define RESET_DETECT_REGISTER   &(SYSCON->SYSRSTSTAT) /* Address of detect register */
#define RESET_DETECT_MASK       SYSCON_SYSRSTSTAT_WDT_MASK
#define REFRESH_INDEX           NULL
#define CLEAR_FLAG              0
#define REG_WIDE                FS_WDOG_SRS_WIDE_32b

#define Watchdog_refresh              WWDT->FEED = 0xAA;WWDT->FEED = 0x55;

#define ENDLESS_LOOP_ENABLE           1    /* set 1 or 0 */
#define WATCHDOG_RESETS_LIMIT         1000
#define WATCHDOG_REFRESH_RATIO        1
#define WATCHDOG_TIMEOUT_VALUE        40000 /* Wdog TICK number for  */
#define WD_REF_TIMER_CLOCK_FREQUENCY  30e06 
#define WATCHDOG_CLOCK                500e3    
#define WD_TEST_TOLERANCE             20    /* % */ 

/* GPIO macros */
#define PIN_DIRECTION_IN  0
#define PIN_DIRECTION_OUT 1

#define PIN_PULL_DISABLE  0
#define PIN_PULL_DOWN     1
#define PIN_PULL_UP       2

#define LOGICAL_ONE   1
#define LOGICAL_ZERO  0

/* DIO test */
#define DIO_WAIT_CYCLE 75

#define DIO_BACKUP_ENABLE  1
#define DIO_BACKUP_DISABLE 0
#define DIO_BACKUP DIO_BACKUP_ENABLE 

#define DIO_SHORT_TO_GND_TEST   1
#define DIO_SHORT_TO_VDD_TEST   0
 
/* Program Counter TEST */
#define PC_TEST_PATTERN    0x10001000/* test address for Program counter test (in RAM region) */

/* SERIAL macros */
#define USE_UART 1 /* 1 = UART, 0 = LPUART */
#define APPLICATION_SERIAL_BASE USART0      
#define SERIAL_BAUD_RATE      9600
#define SERIAL_CLOCK          30e6
#define UART_TX_PIN           49 // P1_17
#define UART_RX_PIN           48 // P1_16

/* FLASH test */
#define HW_FLASH_TEST 1 /* HW=1, SW=0 */

#define FLASH_TEST_BLOCK_SIZE 0x20

#if defined(__IAR_SYSTEMS_ICC__) /* IAR */
    #define FLASH_TEST_CONDITION_SEED 0x0000 
#endif

#if defined(__GNUC__) || (defined(__GNUC__) && ( __ARMCC_VERSION >= 6010050)) /* MCUXpresso + KEIL */
    #define FLASH_TEST_CONDITION_SEED 0x1D0F
    /* This must be in consistence with setting in "User AFTER BUILD = srec_cat*/
    #define FLASH_TEST_START_ADDRESS  0x410
    #define FLASH_TEST_END_ADDRESS    0x4DFE
    #define CRC_VALUE_ADDR            0x4DFE
#endif

/* RAM test */
#define RAM_TEST_BLOCK_SIZE           0x4 /* size of block for runtime testing */
#if defined(__IAR_SYSTEMS_ICC__) || (defined(__GNUC__) && ( __ARMCC_VERSION >= 6010050)) /* IAR + KEIL */
    #define RAM_TEST_BACKUP_SIZE     0x20 /* must fit with the setup from linker configuration file */
    #define STACK_TEST_BLOCK_SIZE    0x10 /* must fit with the setup from linker configuration file */
#endif

/* STACK test */
#define STACK_TEST_PATTERN 0x77777777

/* ADC test */
#define TESTED_ADC ADC0 /* Which ADC is use for AIO test */
#define ADC_RESOLUTION  12
#define ADC_MAX  ((1<<(ADC_RESOLUTION))-1)
#define ADC_REFERENCE  3.3
#define ADC_BANDGAP_LEVEL  1.65   /* depends on power supply configuration */
#define ADC_BANDGAP_LEVEL_RAW  (((ADC_BANDGAP_LEVEL)*(ADC_MAX))/(ADC_REFERENCE))
#define ADC_DEVIATION_PERCENT 20
#define ADC_MIN_LIMIT(val)  (uint16_t)(((val) * (100 - ADC_DEVIATION_PERCENT)) / 100)
#define ADC_MAX_LIMIT(val)  (uint16_t)(((val) * (100 + ADC_DEVIATION_PERCENT)) / 100)
#define FS_CFG_AIO_CHANNELS_CNT    3
#define FS_CFG_AIO_CHANNELS_LIMITS_INIT \
{\
  {ADC_MIN_LIMIT(0), ADC_MAX_LIMIT(60)}, \
  {ADC_MIN_LIMIT(ADC_MAX), ADC_MAX_LIMIT(ADC_MAX)},\
  {ADC_MIN_LIMIT(ADC_BANDGAP_LEVEL_RAW), ADC_MAX_LIMIT(ADC_BANDGAP_LEVEL_RAW)}\
}

#define FS_CFG_AIO_CHANNELS_INIT {2, 3, 7}  /* ADC Channels for V_refl, V_refh, bandgap */

/* NULL definition */
#ifndef NULL
    #define NULL 0
#endif

#endif /* _SAFETY_CONFIG_H_ */
