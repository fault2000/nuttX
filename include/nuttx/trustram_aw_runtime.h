/* SPDX-License-Identifier: Apache-2.0 */
#ifndef __INCLUDE_NUTTX_TRUSTRAM_AW_RUNTIME_H
#define __INCLUDE_NUTTX_TRUSTRAM_AW_RUNTIME_H
#include <nuttx/config.h>
#ifdef CONFIG_ARM_TRUSTRAM_AW_RUNTIME
#if !defined(CONFIG_ARM_TRUSTRAM_AW_BOOT) || defined(CONFIG_ARCH_TRUSTRAM_CONTEXT_HOOKS)
# error "AW runtime checkpoint requires the bounded AW boot storage profile"
#endif
#include <stddef.h>
#include <stdint.h>
#include <nuttx/sched.h>
/* Exact reserved identity selects a service, never grants caller authority.
 * Every service checks its retained protected task generation. Other native
 * tasks keep the ordinary scheduler path; whole-image protection is absent. */
extern struct task_tcb_s g_trustram_aw_runtime_tcb;
#define TRUSTRAM_AW_RUNTIME_TASK(t) ((t)==(struct tcb_s *)&g_trustram_aw_runtime_tcb)
void board_aw_runtime_check_create(struct tcb_s *,void *,size_t,main_t);
int board_aw_runtime_use_stack(struct tcb_s *,void *,size_t);
int board_aw_runtime_prepare(struct tcb_s *);
void *board_aw_runtime_stack_frame(struct tcb_s *,size_t);
void board_aw_runtime_initial(struct tcb_s *);
int board_aw_runtime_seal(struct tcb_s *);
void board_aw_runtime_abort(struct tcb_s *);
void board_aw_runtime_check_uninit(struct tcb_s *);
void board_aw_runtime_activate(struct tcb_s *);
void board_aw_runtime_root(void) __attribute__((noreturn));
int board_aw_runtime_startup(struct tcb_s *);
void board_aw_runtime_switch(struct tcb_s *,struct tcb_s *);
void board_aw_runtime_exit(int) __attribute__((noreturn));
void board_aw_runtime_release_begin(struct tcb_s *);
void board_aw_runtime_release_stack(struct tcb_s *,uint8_t);
void board_aw_runtime_cleanup_complete(struct tcb_s *);
void board_aw_runtime_check_block(void);
void board_aw_runtime_fault(void) __attribute__((noreturn));
#endif
#endif
