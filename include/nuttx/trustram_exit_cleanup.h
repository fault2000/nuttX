/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef __INCLUDE_NUTTX_TRUSTRAM_EXIT_CLEANUP_H
#define __INCLUDE_NUTTX_TRUSTRAM_EXIT_CLEANUP_H
#include <nuttx/trustram_exit_source.h>
#include <stdint.h>
#if !defined(TRUSTRAM_EXIT_CLEANUP_PROBE) || !defined(CONFIG_DISABLE_POSIX_TIMERS) || \
 defined(CONFIG_CANCELLATION_POINTS) || defined(CONFIG_SCHED_SPORADIC) || \
 defined(CONFIG_SCHED_RESUMESCHEDULER) || defined(CONFIG_SCHED_SUSPENDSCHEDULER) || CONFIG_RR_INTERVAL != 0
# error "Exit cleanup requires its bounded single-pair profile"
#endif
void board_trustram_boot_fault(void) __attribute__((noreturn));
void board_trustram_exit_cleanup_begin(struct tcb_s *);
void board_trustram_exit_cleanup_release(struct tcb_s *);
void board_trustram_exit_cleanup_stack(struct tcb_s *,uint8_t);
void board_trustram_exit_cleanup_complete_tcb(struct tcb_s *);
void board_trustram_exit_cleanup_finish(struct tcb_s *,struct tcb_s *,int);
void board_trustram_exit_native_handoff(const void *,const void *) __attribute__((noreturn));
#endif
