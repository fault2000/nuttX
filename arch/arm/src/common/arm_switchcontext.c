/****************************************************************************
 * arch/arm/src/common/arm_switchcontext.c
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

#include <arch/syscall.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_switchcontext
 *
 * Description:
 *   Save the current thread context and restore the specified context.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_ARM_TRUSTRAM_COOPERATIVE_TEST) || defined(CONFIG_ARM_TRUSTRAM_NATIVE_BOOT)
#  if !defined(CONFIG_BUILD_FLAT) || !defined(CONFIG_ARCH_CORTEXM7) || \
      !defined(CONFIG_ARCH_FPU) || !defined(CONFIG_ARMV7M_USEBASEPRI) || \
      defined(CONFIG_SMP) || defined(CONFIG_ARCH_HIPRI_INTERRUPT)
#    error "Cooperative machine diagnostic requires the reviewed Flat M7 FPU ABI"
#  endif
_Static_assert(SYS_switch_context == 2, "fixed native cooperative SVC command");

/* A fixed SVC continuation is shared by the normal native return and the
 * explicitly armed diagnostic vectors. No C prologue may precede it. */
void arm_switchcontext(uint32_t **saveregs __attribute__((unused)),
                       uint32_t *restoreregs __attribute__((unused)))
  __attribute__((naked));
void arm_switchcontext(uint32_t **saveregs __attribute__((unused)),
                       uint32_t *restoreregs __attribute__((unused)))
{
  __asm__ volatile
    (
      "mov r2, r1\n"
      "mov r1, r0\n"
      "movs r0, #2\n"
      "svc #0\n"
#ifdef CONFIG_ARM_TRUSTRAM_NATIVE_BOOT
      ".global up_trustram_native_after_svc\n"
      ".hidden up_trustram_native_after_svc\n"
      "up_trustram_native_after_svc:\n"
#else
      ".global up_trustram_coop_after_svc\n"
      ".hidden up_trustram_coop_after_svc\n"
      "up_trustram_coop_after_svc:\n"
#endif
      "bx lr\n"
    );
}
#else
void arm_switchcontext(uint32_t **saveregs, uint32_t *restoreregs)
{
  sys_call2(SYS_switch_context, (uintptr_t)saveregs, (uintptr_t)restoreregs);
}
#endif
