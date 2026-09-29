/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef __INCLUDE_NUTTX_TRUSTRAM_CPU_TASK_H
#define __INCLUDE_NUTTX_TRUSTRAM_CPU_TASK_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
void *aw_cpu_native_tcb_allocate(uint32_t type);
void *aw_cpu_native_stack_allocate(uint32_t bytes);
uint32_t aw_cpu_native_task_allocation(uint32_t tcb);
uint32_t aw_cpu_native_task_base(uint32_t tcb);
uint32_t aw_cpu_native_task_bytes(uint32_t tcb);
uint32_t aw_cpu_native_task_carve(uint32_t tcb,uint32_t bytes);
uint32_t aw_cpu_native_task_seal(uint32_t tcb,uint32_t entry,uint32_t argc,uint32_t argv);
uint32_t aw_cpu_native_task_started(uint32_t tcb);
uint32_t aw_cpu_native_task_self_entry(void);
uint32_t aw_cpu_native_task_self_argc(void);
uint32_t aw_cpu_native_task_self_argv(void);
uint32_t aw_cpu_native_task_self_identity(void);
uint32_t aw_cpu_native_task_self_type(void);
uint32_t aw_cpu_native_task_stop(uint32_t tcb);
uint32_t aw_cpu_native_task_cleanup_done(uint32_t tcb);
uint32_t aw_cpu_native_task_cancel(uint32_t tcb);
uint32_t aw_cpu_native_task_release_check(uint32_t tcb);
uint32_t aw_cpu_native_task_abort_reclaim(uint32_t tcb);
void aw_cpu_native_task_exit(void) __attribute__((noreturn));
void aw_cpu_native_task_switch(void);
#ifdef __cplusplus
}
#endif
#endif
