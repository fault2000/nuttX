/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef __INCLUDE_NUTTX_TRUSTRAM_EXIT_BOOT_H
#define __INCLUDE_NUTTX_TRUSTRAM_EXIT_BOOT_H
#include <nuttx/trustram_exit_init.h>
#if !defined(TRUSTRAM_EXIT_BOOT_PROBE) || !defined(TRUSTRAM_EXIT_STORAGE_PROBE) || \
    !defined(TRUSTRAM_EXIT_NATIVE_SERVICES_PROBE) || !defined(CONFIG_ARM_MPU_EARLY_RESET) || \
    !defined(CONFIG_ARMV7M_DTCM) || defined(CONFIG_ARCH_RAMFUNCS) || \
    defined(CONFIG_ARMV7M_ITCM) || defined(CONFIG_ARMV7M_STACKCHECK) || \
    defined(CONFIG_SCHED_TRUSTRAM_OBSERVE) || defined(TRUSTRAM_BOOT_IDLE_PROBE) || \
    defined(TRUSTRAM_BOOT_TASK_PUBLISH_PROBE) || CONFIG_SMP_NCPUS != 1 || \
    CONFIG_IDLETHREAD_STACKSIZE != 1024
# error "Exit boot requires its exclusive masked single-CPU reset profile"
#endif
/* Trusted reset only, after data/BSS and TCM setup, before any peripheral,
 * DMA, heap or native task initialization. Does not return to normal boot. */
void board_trustram_exit_boot_prepare(void);
__attribute__((noreturn)) void board_trustram_exit_boot_entry(void);
#ifdef TRUSTRAM_EXIT_FIRST_PROBE
# if defined(TRUSTRAM_BOOT_TASK_HANDOFF_PROBE) || !defined(__arm__) || !defined(__thumb__)
#  error "Exit first entry requires its exclusive ARM boot profile"
# endif
__attribute__((noreturn)) void board_trustram_exit_first_startup(void);
#endif
#endif
