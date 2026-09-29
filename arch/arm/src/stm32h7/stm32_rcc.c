/****************************************************************************
 * arch/arm/src/stm32h7/stm32_rcc.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/stm32_flash.h"
#ifdef CONFIG_ARM_TRUSTRAM_AW_CPU
#  include "hardware/stm32_syscfg.h"
#endif
#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "stm32_pwr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allow up to 100 milliseconds for the high speed clock to become ready.
 * that is a very long delay, but if the clock does not become ready we are
 * hosed anyway.
 */

#define HSERDY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Include chip-specific clocking initialization logic */

#ifdef CONFIG_ARM_TRUSTRAM_AW_CPU
#  if defined(CONFIG_STM32H7_RTC) || defined(CONFIG_STM32H7_IWDG) || \
      defined(CONFIG_PM) || defined(CONFIG_STM32H7_CUSTOM_CLOCKCONFIG)
#    error "CPU reset-only RCC closure excludes RTC/watchdog/PM/custom clocks"
#  endif
#  define stm32_stdclockconfig aw_cpu_stdclockconfig
#endif
#if defined(CONFIG_STM32H7_STM32H7X3XX)
#  include "stm32h7x3xx_rcc.c"
#elif defined(CONFIG_STM32H7_STM32H7B3XX)
#  include "stm32h7x3xx_rcc.c"
#elif defined(CONFIG_STM32H7_STM32H7X5XX)
#  include "stm32h7x3xx_rcc.c"
#elif defined(CONFIG_STM32H7_STM32H7X7XX)
#  include "stm32h7x7xx_rcc.c"
#else
#  error "Unsupported STM32 H7 chip"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_clockconfig
 *
 * Description:
 *   Called to establish the clock settings based on the values in board.h.
 *   This function (by default) will reset most everything, enable the PLL,
 *   and enable peripheral clocking for all peripherals enabled in the NuttX
 *   configuration file.
 *
 *   If CONFIG_STM32H7_CUSTOM_CLOCKCONFIG is defined, then clocking will be
 *   enabled by an externally provided, board-specific function called
 *   stm32_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARM_TRUSTRAM_AW_CPU
/* Called only by the trusted reset prefix, before any ordinary compiled C.
 * Entire source and its fixed iocompensation helper form a no-FP private boot
 * closure. There is no runtime clockconfig entry or ordinary dynamic selector.
 * No scheduler/global backup-counter calls are made before BSS initialization.
 */
#ifdef CONFIG_STM32H7_SYSCFG_IOCOMPENSATION
static void aw_cpu_iocompensation_boot(void)
{
  /* The same fixed compensation operation as the native reset path. Keep it
   * in this reset-only source: runtime GPIO APIs remain instrumented services.
   */
  putreg32(SYSCFG_CCCSR_EN, STM32_SYSCFG_CCCSR);
  while ((getreg32(STM32_SYSCFG_CCCSR) & SYSCFG_CCCSR_READY) == 0)
    {
    }
}
#endif

void aw_cpu_clock_boot(void)
{
  uint32_t regval;
  rcc_reset();
  regval = getreg32(STM32_PWR_CR1);
  putreg32(regval & ~PWR_CR1_DBP, STM32_PWR_CR1);
  stm32_stdclockconfig();
  rcc_enableperipherals();
#ifdef CONFIG_STM32H7_SYSCFG_IOCOMPENSATION
  aw_cpu_iocompensation_boot();
#endif
}
#else
void stm32_clockconfig(void)
{
  /* Make sure that we are starting in the reset state */

  rcc_reset();

#if defined(CONFIG_STM32H7_PWR)

  /* Insure the bkp is initialized */

  stm32_pwr_initbkp(false);
#endif

#if defined(CONFIG_STM32H7_CUSTOM_CLOCKCONFIG)

  /* Invoke Board Custom Clock Configuration */

  stm32_board_clockconfig();

#else

  /* Invoke standard, fixed clock configuration based on definitions in
   * board.h
   */

  stm32_stdclockconfig();

#endif

  /* Enable peripheral clocking */

  rcc_enableperipherals();

#ifdef CONFIG_STM32H7_SYSCFG_IOCOMPENSATION
  /* Enable I/O Compensation */

  stm32_iocompensation();
#endif
}

#endif /* CONFIG_ARM_TRUSTRAM_AW_CPU */

/****************************************************************************
 * Name: stm32_clockenable
 *
 * Description:
 *   Re-enable the clock and restore the clock settings based on settings in
 *   board.h. This function is only available to support low-power modes of
 *   operation:  When re-awakening from deep-sleep modes, it is necessary to
 *   re-enable/re-start the PLL
 *
 *   This functional performs a subset of the operations performed by
 *   stm32_clockconfig():  It does not reset any devices, and it does not
 *   reset the currently enabled peripheral clocks.
 *
 *   If CONFIG_STM32H7_CUSTOM_CLOCKCONFIG is defined, then clocking will be
 *   enabled by an externally provided, board-specific function called
 *   stm32_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void stm32_clockenable(void)
{
#if defined(CONFIG_STM32H7_CUSTOM_CLOCKCONFIG)

  /* Invoke Board Custom Clock Configuration */

  stm32_board_clockconfig();

#else

  /* Invoke standard, fixed clock configuration based on definitions in
   * board.h
   */

  stm32_stdclockconfig();

#endif
}
#endif
