/*
 * Copyright 2019 NXP.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "safety_config.h"

#if defined(__IAR_SYSTEMS_ICC__)
    #pragma location =  ".safety_ram"
#endif

/*******************************************************************************
*                                                                              *
*                           STRUCTURE FOR DIO Initialization and TEST          *
*                                                                              *
*******************************************************************************/

fs_dio_test_lpc_t dio_safety_test_item_0 = /* P1_14 */
{
    .iocon_mode_shift = IOCON_PIO_MODE_SHIFT, /*Device depend */  
    .pPort_byte  = (uint8_t *)&(GPIO->B[1][14]), /* Address of byte register in GPIO */
    .pPort_dir   = (uint32_t *)&(GPIO->DIR[1]),  /* Address of dir1 register */
    .pPort_Iocon = (uint32_t *)&(IOCON->PIO[IOCON_INDEX_PIO1_14]), /* Address of concrete IOCON register */
      
    .pinNum = 14, /*Position in DIR registor*/
    .gpio_clkc_shift = SYSCON_SYSAHBCLKCTRL0_GPIO1_SHIFT    
};

fs_dio_test_lpc_t dio_safety_test_item_1 = /* P1_20 */
{
    .iocon_mode_shift = IOCON_PIO_MODE_SHIFT, /*Device depend */  
    .pPort_byte  = (uint8_t *)&(GPIO->B[1][20]), /* Address of byte register in GPIO */
    .pPort_dir   = (uint32_t *)&(GPIO->DIR[1]),  /* Address of dir1 register */
    .pPort_Iocon = (uint32_t *)&(IOCON->PIO[IOCON_INDEX_PIO1_20]), /* Address of concrete IOCON register */
      
    .pinNum = 20, /* Position in DIR register */
    .gpio_clkc_shift = SYSCON_SYSAHBCLKCTRL0_GPIO1_SHIFT,
};

/* NULL terminated array of pointers to dio_test_t items for safety DIO test */
fs_dio_test_lpc_t *dio_safety_test_items[] = { &dio_safety_test_item_0, &dio_safety_test_item_1, NULL };
