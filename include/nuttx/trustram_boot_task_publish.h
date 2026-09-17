/****************************************************************************
 * include/nuttx/trustram_boot_task_publish.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_TRUSTRAM_BOOT_TASK_PUBLISH_H
#define __INCLUDE_NUTTX_TRUSTRAM_BOOT_TASK_PUBLISH_H

#include <nuttx/config.h>

/* Fixed trusted-boot publication only.  This does not install the runtime
 * context hooks or authorize ordinary task creation.  The detached nx_start
 * endpoint never unlocks the scheduler or executes the created worker.
 */

#if !defined(TRUSTRAM_BOOT_TASK_PUBLISH_PROBE) || \
    !defined(TRUSTRAM_BOOT_TASK_CANDIDATE_PROBE) || \
    !defined(TRUSTRAM_BOOT_IDLE_LIFECYCLE_PROBE) || \
    defined(TRUSTRAM_BOOT_EXCEPTION_PROBE) || \
    defined(CONFIG_ARCH_TRUSTRAM_CONTEXT_HOOKS)
#  error "Detached task publication requires candidate lifecycle and excludes other context probes"
#endif

#if !defined(CONFIG_BUILD_FLAT) || defined(CONFIG_BUILD_PROTECTED) || \
    defined(CONFIG_BUILD_KERNEL) || defined(CONFIG_SMP) || \
    defined(CONFIG_PAGING) || !defined(CONFIG_SCHED_WORKQUEUE) || \
    !defined(CONFIG_SCHED_HPWORK) || !defined(CONFIG_SCHED_HPNTHREADS) || \
    CONFIG_SCHED_HPNTHREADS != 1
#  error "Detached task publication requires Flat single-CPU single HPWORK without paging"
#endif

#include <stddef.h>
#include <stdint.h>
#include <nuttx/sched.h>

struct kwork_wqueue_s;

/* Native immutable recipe, exported by the translation unit that actually
 * owns work_thread and g_hpwork.  kind=2 is the fixed kernel-startup ABI;
 * the board checks it against its own protected startup enum.  The policy
 * never supplies authority to an ordinary caller and is not a new generic
 * public work-queue constructor.
 */

struct trustram_boot_hpwork_s
{
  uint32_t kind;
  start_t start;
  main_t entry;
  const char *name;
  int priority;
  size_t stack_bytes;
  unsigned int count;
  struct kwork_wqueue_s *queue;
};

extern const struct trustram_boot_hpwork_s g_trustram_boot_hpwork_policy;

void board_trustram_boot_task_publish_entry_probe(void);
void board_trustram_boot_task_publish_stop_probe(void)
  __attribute__((noreturn));

#ifdef TRUSTRAM_BOOT_TASK_HANDOFF_PROBE
void board_trustram_boot_fault(void) __attribute__((noreturn));
void board_trustram_boot_task_handoff_entry_probe(void)
  __attribute__((noreturn));
void board_trustram_boot_task_handoff_startup_probe(void)
  __attribute__((noreturn));
#endif

#endif /* __INCLUDE_NUTTX_TRUSTRAM_BOOT_TASK_PUBLISH_H */
