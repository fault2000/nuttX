/* SPDX-License-Identifier: Apache-2.0 */
#ifndef __INCLUDE_NUTTX_TRUSTRAM_AW_BOOT_H
#define __INCLUDE_NUTTX_TRUSTRAM_AW_BOOT_H

#include <nuttx/config.h>

#ifdef CONFIG_ARM_TRUSTRAM_AW_BOOT
#if !defined(CONFIG_BUILD_FLAT) || !defined(CONFIG_ARCH_CORTEXM7) || \
    !defined(CONFIG_ARCH_FPU) || !defined(CONFIG_ARMV7M_USEBASEPRI) || \
    !defined(CONFIG_ARM_MPU) || defined(CONFIG_SMP) || \
    defined(CONFIG_ARCH_HIPRI_INTERRUPT) || \
    defined(CONFIG_ARM_TRUSTRAM_NATIVE_BOOT) || \
    defined(CONFIG_ARM_TRUSTRAM_COOPERATIVE_TEST) || \
    defined(CONFIG_ARCH_TRUSTRAM_CONTEXT_HOOKS)
#  error "AW native boot needs its exclusive single-CPU Flat M7 FPU profile"
#endif
#include <stddef.h>
#include <stdint.h>
#include <nuttx/sched.h>

/* One immutable native task, created before IRQ/device bring-up.  The address
 * selects a board service; it is not an ownership or generation credential.
 * Every service must validate its retained protected lifetime before using
 * ordinary TCB fields.  All other NuttX tasks keep their normal paths.
 */

extern struct task_tcb_s g_trustram_aw_tcb;
extern struct tcb_s *const g_trustram_aw_boot_idle;
#define TRUSTRAM_AW_TASK(tcb) \
  ((tcb) == (struct tcb_s *)&g_trustram_aw_tcb)

void board_aw_boot_entry(void);
void board_aw_boot_check_create(struct tcb_s *tcb, void *stack,
                                size_t bytes, main_t entry);
int board_aw_boot_use_stack(struct tcb_s *tcb, void *stack, size_t bytes);
int board_aw_boot_prepare(struct tcb_s *tcb);
void *board_aw_boot_stack_frame(struct tcb_s *tcb, size_t bytes);
void board_aw_boot_initial(struct tcb_s *tcb);
int board_aw_boot_seal(struct tcb_s *tcb);
void board_aw_boot_abort(struct tcb_s *tcb);
void board_aw_boot_activate(struct tcb_s *tcb);

/* Immutable initial PC: admit the AW root before entering nxtask_start's C
 * prologue.  The native startup service returns the approved main's status,
 * and the actual exit() path reaches the off-stack exit entry below.
 */

void board_aw_boot_root(void) __attribute__((noreturn));
int board_aw_boot_startup(struct tcb_s *tcb);
void board_aw_boot_switch(struct tcb_s *outgoing, struct tcb_s *incoming);
void board_aw_boot_exit(int status) __attribute__((noreturn));

/* Native cleanup requests release; it must never free reserved backing or
 * reclaim the live CPU stack.  The board manager separately proves the
 * off-stack handoff and reference drain before scrub/reclaim.
 */

void board_aw_boot_release_begin(struct tcb_s *tcb);
void board_aw_boot_release_stack(struct tcb_s *tcb, uint8_t type);
void board_aw_boot_cleanup_complete(struct tcb_s *tcb);
void board_aw_boot_fault(void) __attribute__((noreturn));
#endif
#endif
