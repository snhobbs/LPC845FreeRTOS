/*
 * Copyright 2019 NXP.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "project_setup_lpcxpresso845max.h"
#include "board.h" 

/*******************************************************************************
* Code
******************************************************************************/
/*!
* @brief   Watchdog configuration function
*
*          Enables the watchdog. Also in Wait and Stop mode. Updates are allowed
*
* @param   None
*
* @return  None
*/
void WatchdogEnable(void)
{
    SYSCON->WDTOSCCTRL &= ~(SYSCON_WDTOSCCTRL_DIVSEL_MASK | SYSCON_WDTOSCCTRL_FREQSEL_MASK);
    SYSCON->WDTOSCCTRL |= SYSCON_WDTOSCCTRL_DIVSEL(0) | SYSCON_WDTOSCCTRL_FREQSEL(0xC);/* WDOg oscilator set to 4MHz => WDOG_osc 2 Mhz => 500kHz Wdog cnt cnt(see RM page 418 figure 52)*/
    SYSCON->PDRUNCFG &= ~(SYSCON_PDRUNCFG_WDTOSC_PD_MASK); /*Power WDOG oscilator */

    SYSCON->SYSAHBCLKCTRL0 |= SYSCON_SYSAHBCLKCTRL0_WWDT_MASK; /*Enable clock to WWDT*/

    WWDT->TC = WWDT_TC_COUNT(WATCHDOG_TIMEOUT_VALUE); /*refresh value */ 
    WWDT->MOD = (WWDT_MOD_LOCK(0))|WWDT_MOD_WDEN(1)|(WWDT_MOD_WDRESET(1));
    WWDT->WINDOW = 0xFFFFFF; /* Disable Window mode */
    WWDT->WARNINT = 0; 

    WWDT->FEED = 0xAA; /* Start WDOG */
    WWDT->FEED = 0x55; 
}

/*!
* @brief   Watchdog disabling function
*
* @param   None
*
* @return  None
*/
void WatchdogDisable(void)
{
    /* WDOG is disabled on LPC after reset by default */
}

/*!
* @brief   Initialization of Systick timer
*
*          This function configures the Systick as a source of interrupt
*
* @param   reloadValue - defines the period of counter refresh
*
* @return  None
*/
void SystickInit(uint32_t reloadValue)
{
    SysTick->VAL = 0;
    SysTick->LOAD = reloadValue;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
}

/* Second timer for CLOCK test */
void ReferenceTimerInit(void)
{
    SYSCON->SYSAHBCLKCTRL0 |= SYSCON_SYSAHBCLKCTRL0_WKT_MASK; /* Enable Clock to WKT regiter */ 
    PMU->DPDCTRL |= ( PMU_DPDCTRL_LPOSCEN_MASK | PMU_DPDCTRL_LPOSCDPDEN_MASK | PMU_DPDCTRL_WAKECLKPAD_DISABLE_MASK ); /*Enable LPO */
    //     LPC_PMU->DPDCTRL |= (1<<LPOSCEN)|          // LPOSC powered
    //                      (1<<LPOSCDPDEN)|       // LPOSC stays powered in deep pd mode
    //                      (1<<WAKEPAD_DISABLE)|  // Disable wakeup pin function
    //                      (1<<WAKE2PAD_DISABLE); // Disable wakeup2 pin function 
    //    
    SYSCON->PRESETCTRL0 &= ~(SYSCON_PRESETCTRL0_WKT_RST_N_MASK);    // Reset the WKT
    SYSCON->PRESETCTRL0 |= (SYSCON_PRESETCTRL0_WKT_RST_N_MASK);

    WKT->CTRL  = 0x0; /*Set LPO osc as clokc for WKT */
    
    WKT->CTRL  |= WKT_CTRL_CLKSEL_MASK;  /* 10kHz LPOosc */
    
    WKT->CTRL  |= WKT_CTRL_CLEARCTR_MASK; /* Clear */
    
    WKT->COUNT  = START_VALUE; /* Set start value for Decreasin of Counter */
}

/*!
* @brief   Setup of clock
*
* @param   void
*
* @return  None
*
*   FRO set to 30MHz ------------------------- MAINCLKSEL -------- MAINCLKPLLSEL ----- MAINCLK = 30MHZ
*
*/
void ClockInit(void)
{
    // Enable clocks to IOCON and SWM upon entry, disable them upon exit
    SYSCON->SYSAHBCLKCTRL0 |= ( SYSCON_SYSAHBCLKCTRL0_SWM(1) | SYSCON_SYSAHBCLKCTRL0_IOCON(1) );

//    // Step 0. Configure the FRO subsystem (choose the source for clocks fro and fro_div)
//    LPC_PWRD_API->set_fro_frequency(30000); /*30MHz*/
//
//    SYSCON->FROOSCCTRL |= (1 << 17);     // Set direct FRO Clock  (bit 17 ) 
//    SYSCON->FRODIRECTCLKUEN = 0;                    // Toggle the update register for the output mux
//    SYSCON->FRODIRECTCLKUEN = 1;
//    while (!(SYSCON->FRODIRECTCLKUEN & 1)) __asm("nop"); // Wait for update to take effect
//
//    // Choose source for main_clk_pre_pll
//    SYSCON->MAINCLKSEL        = 0;     // Select FRO as MAIN clokc
//    SYSCON->MAINCLKUEN        = 0;                  // Toggle update register
//    SYSCON->MAINCLKUEN        = 1;
//    while (!(SYSCON->MAINCLKUEN & 1)) __asm("nop");      // Wait until updated
//
//    // Choose source for main_clk, either main_clk_pre_pll (0) or sys_pll0_clk_src_i (1)
//    SYSCON->MAINCLKPLLSEL     = 0;  // Set PRE PLL CLOCK 
//
//    SYSCON->MAINCLKPLLUEN     = 0;                  // Toggle update register
//    SYSCON->MAINCLKPLLUEN     = 1;
//    while (!(SYSCON->MAINCLKPLLUEN & 1)) __asm("nop");   // Wait until updated
//
//    SYSCON->CLKOUTSEL = 4; // CLKOUT = FRO 
//    SYSCON->CLKOUTDIV =  1; //DIVIDE BY 1  
//    SWM0->PINASSIGN.PINASSIGN11 &= ~(SWM_PINASSIGN11_CLKOUT_O(0xFF));
//    SWM0->PINASSIGN.PINASSIGN11 |= SWM_PINASSIGN11_CLKOUT_O(38); // PIO1_6 CLKOUT 
//  
//    // Configure the main_clock divider
//    SYSCON->SYSAHBCLKDIV      =  1;  // Divided by 1 
  
  BOARD_BootClockFRO30M();
  
}

/*!
* @brief   Initialization of CTIMER
*
*          This function initializes the CTIMER. CTIMER is used for After reset WDog test.
*
* @param   void
*
* @return  None
*/
void CTIMERInit(void)
{
    SYSCON->SYSAHBCLKCTRL0 |= SYSCON_SYSAHBCLKCTRL0_CTIMER_MASK;  // Enable clock
    
    SYSCON->PRESETCTRL0    &= ~(SYSCON_PRESETCTRL0_CTIMER_RST_N_MASK);
    SYSCON->PRESETCTRL0    |=  (SYSCON_PRESETCTRL0_CTIMER_RST_N_MASK);
    
    CTIMER0->CTCR = 0; /* Give DEFAULT state, Timer mode selected */
    CTIMER0->MCR  = 0; /* Default state*/
    CTIMER0->CCR  = 0; /* Default state*/
    CTIMER0->EMR  = 0; /* Default state*/
    CTIMER0->PWMC = 0; /* Default state*/

    CTIMER0->PR = 0; /* Every APB bus clock */;
}

/*!
* @brief  Sets port direction and mux
*
* @param 
*
* @return  None
*/
void PortInit(uint8_t *pByte, uint32_t *pDir, uint32_t *pIocon, uint32_t pinDir, uint32_t pinNum, uint32_t pull, uint32_t clock_enable_shift)
{
    /* Enable clock to GPIO module */
    SYSCON->SYSAHBCLKCTRL0 |= (1<< clock_enable_shift);

    *pIocon &= ~(IOCON_PIO_MODE_MASK); /*Clear PULL setting*/
    *pIocon |= IOCON_PIO_MODE(pull); /*Set pullup*/

    if (pinDir == PIN_DIRECTION_OUT)
    {
        *pDir |= (1 << pinNum);    /* PINx = 1 = output */
    }
    else if (pinDir == PIN_DIRECTION_IN)
    {
        *pDir &= ~(1 << pinNum); /* PINx = 0 = input */
    }
}

/*!
* @brief   Initialization of ADC0
*
*          8 MHz System Oscillator Bus Clock is the source clock.
*          single-ended 12-bit conversion
*
* @param   void
*
* @return  None
*/
#define FRO_CLK 30000000
void AdcInit(void)
{
    uint8_t clkdiv = 0;
    uint32_t temp;
    // Step 1. Power up and reset the ADC, and enable clocks to peripheral
    SYSCON->PDRUNCFG       &= ~(SYSCON_PDRUNCFG_ADC_PD_MASK);
    SYSCON->PRESETCTRL0    &= ~(SYSCON_PRESETCTRL0_ADC_RST_N_MASK);
    SYSCON->PRESETCTRL0    |=  (SYSCON_PRESETCTRL0_ADC_RST_N_MASK);
    SYSCON->SYSAHBCLKCTRL0 |= (SYSCON_SYSAHBCLKCTRL0_ADC_MASK); /* Enable Clock to ADC */
    SYSCON->ADCCLKDIV       = 1;                 // Enable clock, and divide-by-1 at this clock divider
    SYSCON->ADCCLKSEL       = 0; // Use fro_clk as source for ADC async clock

    temp =  ADC0->CTRL;
    // Step 2. Perform a self-calibration
    // Choose a CLKDIV divider value that yields about 500 KHz.
    clkdiv = (FRO_CLK / 500000) - 1;

    // Start the self-calibration
    // Calibration mode = true, low power mode = false, CLKDIV = appropriate for 500,000 Hz
    ADC0->CTRL = ( (ADC_CTRL_CALMODE(1)) | (ADC_CTRL_LPWRMODE(0)) | (ADC_CTRL_CLKDIV(clkdiv)) );

    // Poll the calibration mode bit until it is cleared
    while (ADC0->CTRL & ADC_CTRL_CALMODE_MASK);
    /*  Here is ADC calibrated */

    ADC0->CTRL = temp; /*REstroe ADC setting*/
    ADC0->CTRL |= 0x1; /*Clock Divider 2*/
    ADC0->TRM = 0; /*HIGH voltage range 2,7 - 3,6V VDD*/
    ADC0->SEQ_CTRL[0] |=  ADC_SEQ_CTRL_TRIGPOL_MASK; //polarity of triger

    //TODO Some macro for easy edit? 
    SWM0->PINENABLE0 &= ~(SWM_PINENABLE0_ADC_2_MASK|SWM_PINENABLE0_ADC_3_MASK|SWM_PINENABLE0_ADC_7_MASK); 
}

/* Configure UART0 for "brate" baud, 8 data bits, no parity, 1 stop bit.
* For asynchronous mode (UART mode) the formula is:
* (BRG + 1) * (1 + (m/256)) * (16 * baudrate Hz.) = FRG_in Hz.
* We proceed in 2 steps.
* Step 1: Let m = 0, and round (down) to the nearest integer value of BRG for the desired baudrate.
* Step 2: Plug in the BRG from step 1, and find the nearest integer value of m, (for the FRG fractional part).
*
* Step 1 (with m = 0)
* BRG = ((bclk Hz.) / (16 * brate )) - 1
*
* Step 2.
* m = 256 * [-1 + {(bclk Hz.) / (16 * brate )(BRG + 1)}]

**********************************************************/
void SerialInit(USART_Type *Uart_X, uint32_t brate, uint64_t bclk)
{
    uint32_t brg = (bclk / (16 * brate)) - 1;
    uint32_t m = 256 * (-1 + bclk / (16 * brate *(brg + 1)));

    SYSCON->SYSAHBCLKCTRL0 |= SYSCON_SYSAHBCLKCTRL0_UART0(1); //Enable clock to UART 
    SYSCON->SYSAHBCLKCTRL0 |= SYSCON_SYSAHBCLKCTRL0_SWM(1); //Enable clock to SWM 

    SWM0->PINASSIGN.PINASSIGN0 &= ~(SWM_PINASSIGN0_U0_TXD_O(0xFF)); // ASIGN fuction to coresponding GPIO pin
    SWM0->PINASSIGN.PINASSIGN0 |= SWM_PINASSIGN0_U0_TXD_O(UART_TX_PIN); // ASIGN fuction to coresponding GPIO pin

    SWM0->PINASSIGN.PINASSIGN0 &= ~(SWM_PINASSIGN0_U0_RXD_I(0xFF));
    SWM0->PINASSIGN.PINASSIGN0 |= SWM_PINASSIGN0_U0_RXD_I(UART_RX_PIN);

    SYSCON->FRG[0].FRGMULT = m; 
    SYSCON->FRG[0].FRGDIV = 255;
    SYSCON->FRG[0].FRGCLKSEL = 1; // Select main clock as Clock source for FRG0 

    SYSCON->FCLKSEL[0] = 2 ; // select FRG0 as clock source for UART0

    // Give USART0 a reset
    SYSCON->PRESETCTRL0 &= ~(SYSCON_PRESETCTRL0_UART0_RST_N(1));  
    __asm("nop");
    SYSCON->PRESETCTRL0 |= SYSCON_PRESETCTRL0_UART0_RST_N(1);  

    USART0->BRG = brg; // configure baud rate
    USART0->CFG |= USART_CFG_DATALEN(1); //8bit data length
    USART0->CFG &= ~(USART_CFG_PARITYSEL(0)); // PARITY NONE
    USART0->CFG &= ~(USART_CFG_STOPLEN(0)); // one stop bit
    USART0->CTL = 0; // do nothing

    USART0->STAT = 0xFFFF; // clear any pending stat 
    USART0->CFG |= 0x1; // Enable uart 
    // USART0->INTENSET = 1; //RXREADY  
}
