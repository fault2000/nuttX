/****************************************************************************
 * include/nuttx/trustram_boot_worker_cycle.h
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_TRUSTRAM_BOOT_WORKER_CYCLE_H
#define __INCLUDE_NUTTX_TRUSTRAM_BOOT_WORKER_CYCLE_H

#include <nuttx/trustram_boot_worker_wake.h>

#if !defined(TRUSTRAM_BOOT_WORKER_CYCLE_PROBE) || \
    !defined(TRUSTRAM_BOOT_WORKER_WAKE_PROBE)
#error "First worker cycle requires the detached native wake subset"
#endif

/* The fixed idle return resumes exactly after up_unblock_task's real bridge
 * BL. Final-link validation checks both the opcode and this code landmark;
 * an arbitrary ordinary LR is not a continuation authority. */
void board_trustram_boot_worker_wake_native_return(void);

#endif /* __INCLUDE_NUTTX_TRUSTRAM_BOOT_WORKER_CYCLE_H */
