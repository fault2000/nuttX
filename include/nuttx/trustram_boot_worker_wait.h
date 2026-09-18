/****************************************************************************
 * include/nuttx/trustram_boot_worker_wait.h
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_TRUSTRAM_BOOT_WORKER_WAIT_H
#define __INCLUDE_NUTTX_TRUSTRAM_BOOT_WORKER_WAIT_H

#include <nuttx/config.h>
#include <stdint.h>

#if !defined(TRUSTRAM_BOOT_WORKER_WAIT_PROBE) || \
    !defined(TRUSTRAM_BOOT_TASK_HANDOFF_PROBE) || \
    !defined(TRUSTRAM_BOOT_TASK_PUBLISH_PROBE) || \
    !defined(CONFIG_BUILD_FLAT) || defined(CONFIG_SMP) || \
    defined(CONFIG_ARCH_TRUSTRAM_CONTEXT_HOOKS) || \
    defined(CONFIG_PRIORITY_INHERITANCE) || \
    defined(CONFIG_SCHED_SPORADIC) || defined(CONFIG_SCHED_CRITMONITOR) || \
    defined(CONFIG_SCHED_INSTRUMENTATION) || defined(CONFIG_IRQCOUNT) || \
    defined(CONFIG_SCHED_SUSPENDSCHEDULER) || \
    defined(CONFIG_SCHED_RESUMESCHEDULER) || CONFIG_RR_INTERVAL > 0
#error "First worker wait requires the detached Flat native semaphore subset"
#endif

/* The actual worker body becomes visible only in this detached profile.
 * A fixed reviewed machine call reaches it with the approved native argv. */
int work_thread(int argc, char *argv[]);
void board_trustram_boot_fault(void) __attribute__((noreturn));

/* Same argument shape as native arm_switchcontext, but neither pointer grants
 * authority. The optional fixed wake profile alone admits a normal return;
 * general wait/wake and arbitrary task switching are not implemented. */
void board_trustram_boot_worker_wait_entry_probe(uint32_t **save,
                                               uint32_t *restore)
#ifndef TRUSTRAM_BOOT_WORKER_WAKE_PROBE
  __attribute__((noreturn));
#else
  ;
# include <nuttx/trustram_boot_worker_wake.h>
#endif

#endif /* __INCLUDE_NUTTX_TRUSTRAM_BOOT_WORKER_WAIT_H */
