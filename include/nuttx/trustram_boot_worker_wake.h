/****************************************************************************
 * include/nuttx/trustram_boot_worker_wake.h
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_TRUSTRAM_BOOT_WORKER_WAKE_H
#define __INCLUDE_NUTTX_TRUSTRAM_BOOT_WORKER_WAKE_H

#include <nuttx/trustram_boot_worker_wait.h>

#if !defined(TRUSTRAM_BOOT_WORKER_WAKE_PROBE) || \
    !defined(TRUSTRAM_BOOT_WORKER_WAIT_PROBE) || defined(CONFIG_SCHED_TICKLESS)
#error "First worker wake requires the detached non-timed native wait subset"
#endif

/* Only the original worker block boundary returns in this profile. The idle
 * post's unblock boundary retains its native frames and never resumes here. */
void board_trustram_boot_worker_wake_entry_probe(uint32_t **save,
                                               uint32_t *restore)
  __attribute__((noreturn));

/* A code landmark immediately after the real block boundary's BL.  The final
 * ARM verifier checks its placement before accepting the protected LR. */
void board_trustram_boot_worker_wait_native_return(void);

/* The actual native wait result is checked at the fixed completion boundary;
 * no callback, another wait iteration, or worker exit may execute. */
void board_trustram_boot_worker_wake_complete_probe(int result)
  __attribute__((noreturn));

#endif /* __INCLUDE_NUTTX_TRUSTRAM_BOOT_WORKER_WAKE_H */
