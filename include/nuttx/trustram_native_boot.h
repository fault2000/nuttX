/* SPDX-License-Identifier: Apache-2.0 */
#ifndef __INCLUDE_NUTTX_TRUSTRAM_NATIVE_BOOT_H
#define __INCLUDE_NUTTX_TRUSTRAM_NATIVE_BOOT_H
#include <nuttx/config.h>
#ifdef CONFIG_ARM_TRUSTRAM_NATIVE_BOOT
#include <stddef.h>
#include <stdint.h>
#include <nuttx/sched.h>
struct kwork_wqueue_s;
struct trustram_native_hpwork_s
{
  start_t start;
  main_t entry;
  const char *name;
  int priority;
  size_t stack_bytes;
  unsigned int count;
  struct kwork_wqueue_s *queue;
};
extern const struct trustram_native_hpwork_s g_trustram_native_hpwork;
extern struct task_tcb_s g_trustram_native_tcb;
#define TRUSTRAM_NATIVE_TASK(tcb) \
  ((tcb) == (struct tcb_s *)&g_trustram_native_tcb)
/* Sole fixed HPWORK constructor. No ordinary request parameters. */
void board_trustram_native_boot_create(void);
/* These internal adapters require the already-open protected creation
 * service. They are NOT independent service entry points. */
void board_trustram_native_check_create(struct tcb_s *tcb);
int board_trustram_native_use_stack(struct tcb_s *tcb, void *stack, size_t size);
void *board_trustram_native_stack_frame(struct tcb_s *tcb, size_t bytes);
void board_trustram_native_initial_state(struct tcb_s *tcb);
void board_trustram_boot_fault(void) __attribute__((noreturn));
extern struct tcb_s *const g_trustram_native_boot_idle;
/* Exactly around nx_start's final scheduler release, under trusted boot
 * ownership. Not a runtime registration or general context interface. */
void board_trustram_native_boot_prepare(void);
void board_trustram_native_boot_complete(void);
#endif
#endif
