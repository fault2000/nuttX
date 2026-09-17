/****************************************************************************
 * include/nuttx/trustram_boot_idle.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_TRUSTRAM_BOOT_IDLE_H
#define __INCLUDE_NUTTX_TRUSTRAM_BOOT_IDLE_H

#include <nuttx/config.h>

/* Fixed detached idle initialization only.  These declarations do not enable
 * the host-only context hooks or provide ordinary task/stack services.
 */

#if !defined(TRUSTRAM_BOOT_IDLE_LIFECYCLE_PROBE) || \
    !defined(TRUSTRAM_BOOT_IDLE_PROBE) || \
    !defined(TRUSTRAM_BOOT_ACCESS_PROBE) || \
    !defined(TRUSTRAM_BOOT_ROOT_PROBE) || \
    !defined(TRUSTRAM_BOOT_LAYOUT_PROBE) || \
    defined(CONFIG_ARCH_TRUSTRAM_CONTEXT_HOOKS)
#  error "Detached idle lifecycle requires boot probes and excludes runtime hooks"
#endif

#if !defined(CONFIG_BUILD_FLAT) || defined(CONFIG_BUILD_PROTECTED) || \
    defined(CONFIG_BUILD_KERNEL) || defined(CONFIG_SMP) || \
    defined(CONFIG_TLS_ALIGNED) || defined(CONFIG_SCHED_THREAD_LOCAL)
#  error "Detached idle lifecycle requires Flat single-CPU fixed native TLS"
#endif

#include <stddef.h>

struct tcb_s;

struct trustram_boot_idle_stack_s
{
  void *allocation;
  void *base;
  size_t size;
};

struct trustram_boot_idle_frame_s
{
  void *frame;
  size_t size;
  struct trustram_boot_idle_stack_s remaining;
};

/* All calls below require the one reviewed boot call chain and exclusive
 * CPU/DMA ownership.  Pointer/size arguments are comparisons against fixed
 * protected registration, never authorization supplied by ordinary callers.
 * The sole no-argument ASM entry opens/closes the fixed arena transaction.
 * Native helpers receive write plans only while that transaction is active.
 * There is no synthetic idle register frame, coloration or MSP replacement.
 */

void board_trustram_boot_idle_lifecycle_entry_probe(void);
void board_trustram_boot_idle_initial_probe(
  struct tcb_s *tcb, struct trustram_boot_idle_stack_s *out);
void board_trustram_boot_idle_carve_probe(
  struct tcb_s *tcb, size_t requested,
  struct trustram_boot_idle_frame_s *out);
void board_trustram_boot_fault(void) __attribute__((noreturn));

#endif /* __INCLUDE_NUTTX_TRUSTRAM_BOOT_IDLE_H */
