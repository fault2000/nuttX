/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef __INCLUDE_NUTTX_TRUSTRAM_EXIT_SOURCE_H
#define __INCLUDE_NUTTX_TRUSTRAM_EXIT_SOURCE_H
#include <nuttx/trustram_exit_boot.h>
#if !defined(TRUSTRAM_EXIT_SOURCE_PROBE) || !defined(TRUSTRAM_EXIT_FIRST_PROBE) || \
    defined(CONFIG_ARCH_TRUSTRAM_CONTEXT_HOOKS)
# error "Source exit requires the exclusive first-task boot profile"
#endif
struct tcb_s;
/* Admitted source Thread/MSP only. Stops before native cleanup writers. */
void board_trustram_exit_source_exit_begin(const struct tcb_s *, int);
void board_trustram_exit_source_stop(void) __attribute__((noreturn));
void board_trustram_exit_pair_startup(void) __attribute__((noreturn));
#endif
