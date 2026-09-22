/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef __INCLUDE_NUTTX_TRUSTRAM_EXIT_INIT_H
#define __INCLUDE_NUTTX_TRUSTRAM_EXIT_INIT_H
#include <nuttx/config.h>
#include <stddef.h>
#if !defined(TRUSTRAM_EXIT_NATIVE_INIT_PROBE) || !defined(CONFIG_BUILD_FLAT) || \
    defined(CONFIG_ARCH_TRUSTRAM_CONTEXT_HOOKS) || defined(CONFIG_ARM_TRUSTRAM_NATIVE_BOOT) || \
    defined(CONFIG_SMP) || defined(CONFIG_BUILD_PROTECTED) || defined(CONFIG_BUILD_KERNEL)
# error "Exit native init requires the explicit detached Flat profile"
#endif
#if defined(TRUSTRAM_EXIT_INIT_HOST_TEST) && (defined(__arm__) || defined(__thumb__))
# error "Host exit init substitutions cannot be used on ARM"
#endif
struct tcb_s;
/* Fixed two-task detached constructor callbacks. Not general runtime hooks.
 * Trusted boot owns admission, serialization and all native/kernel writers.
 * Every unknown pointer or invalid transition terminates before TCB access. */
void board_trustram_exit_init_check(struct tcb_s *);
int board_trustram_exit_init_stack(struct tcb_s *, void *, size_t);
void *board_trustram_exit_init_carve(struct tcb_s *, size_t);
void board_trustram_exit_init_initial(struct tcb_s *);
int board_trustram_exit_init_prepare(struct tcb_s *);
int board_trustram_exit_init_seal(struct tcb_s *);
void board_trustram_exit_init_abort(struct tcb_s *);
void board_trustram_exit_init_activate(struct tcb_s *);
#endif
