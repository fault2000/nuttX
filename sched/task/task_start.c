/****************************************************************************
 * sched/task/task_start.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_ARM_TRUSTRAM_AW_CPU
#  include <nuttx/trustram_cpu_task.h>
#endif
#ifdef CONFIG_ARM_TRUSTRAM_AW_RUNTIME
#  include <nuttx/trustram_aw_runtime.h>
#endif

#ifdef CONFIG_ARM_TRUSTRAM_AW_BOOT
#  include <nuttx/trustram_aw_boot.h>
#endif

#ifdef CONFIG_SCHED_TRUSTRAM_OBSERVE
#  include <nuttx/trustram_observe.h>
#endif

#include <stdlib.h>
#include <sched.h>
#include <assert.h>
#include <debug.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/tls.h>

#ifdef CONFIG_ARCH_TRUSTRAM_CONTEXT_HOOKS
#  include <nuttx/trustram_context.h>
#endif

#include "group/group.h"
#include "sched/sched.h"
#include "signal/signal.h"
#include "task/task.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is an artificial limit to detect error conditions where an argv[]
 * list is not properly terminated.
 */

#define MAX_START_ARGS 256

#ifdef CONFIG_ARM_TRUSTRAM_AW_CPU
/* Internal libc startup contract, also declared in libs/libc/libc.h. */

void lib_cxx_initialize(void);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Native initial task entry. The detached handoff profile stops after
 * protected dispatch admission, before the actual HPWORK entry executes.
 */
#if defined(TRUSTRAM_EXIT_FIRST_PROBE)
#  include <nuttx/trustram_exit_boot.h>
__attribute__((naked, noreturn)) void nxtask_start(void)
{
#ifdef TRUSTRAM_EXIT_SOURCE_PROBE
  __asm__ volatile ("b.w board_trustram_exit_pair_startup");
#else
  __asm__ volatile ("b.w board_trustram_exit_first_startup");
#endif
}
#else
#ifdef TRUSTRAM_BOOT_TASK_HANDOFF_PROBE
#  include <nuttx/trustram_boot_task_publish.h>
#  if defined(__arm__) && defined(__thumb__)
__attribute__((naked, noreturn)) void nxtask_start(void)
{
  __asm__ volatile ("b.w board_trustram_boot_task_handoff_startup_probe");
}
#  else
void nxtask_start(void) { board_trustram_boot_task_handoff_startup_probe(); }
#  endif
#else


void nxtask_start(void)
{
#ifdef CONFIG_ARM_TRUSTRAM_AW_CPU
  /* Keep the native task/kernel startup distinction, using the retained
   * creation record rather than the writable TCB flags.  Pthreads have their
   * own initial entry and must never pass through this root. */

  const uint32_t type = aw_cpu_native_task_self_type();

  if (type != 1u && type != 3u)
    {
      __builtin_trap();
    }

#ifdef CONFIG_SIG_DEFAULT
  if (type == 1u)
    {
      FAR struct tcb_s *tcb =
        (FAR struct tcb_s *)(uintptr_t)aw_cpu_native_task_self_identity();
      nxsig_default_initialize(tcb);
    }
#endif

  main_t entry = (main_t)(uintptr_t)aw_cpu_native_task_self_entry();
  int argc = (int)aw_cpu_native_task_self_argc();
  char **argv = (char **)(uintptr_t)aw_cpu_native_task_self_argv();

  if (type == 1u)
    {
      /* Same libc initialization used by nxtask_startup().  Keep the actual
       * retained entry call here so its fixed indirect source site remains
       * the native-entry policy boundary. */

      lib_cxx_initialize();
    }

  /* The final indirect template rechecks this physical target against the
   * retained entry of this activation; the ordinary local is only a mirror. */
  exit(entry(argc, argv));
#elif defined(CONFIG_ARCH_TRUSTRAM_CONTEXT_HOOKS)
  FAR struct tcb_s *tcb = this_task();
  struct trustram_task_start_s plan;

  up_trustram_task_start(tcb, &plan);

#ifdef CONFIG_SIG_DEFAULT
  if (!plan.kernel)
    {
      nxsig_default_initialize(tcb);
    }
#endif

  if (plan.kernel)
    {
      plan.entry = up_trustram_task_entry(plan.entry, plan.argc,
                                          plan.argv, true);
      exit(plan.entry(plan.argc, plan.argv));
    }
  else
    {
      nxtask_startup(plan.entry, plan.argc, plan.argv);
    }

  /* The startup/exit services must not return to this initial entry point. */

  PANIC();
#else
  FAR struct task_tcb_s *tcb = (FAR struct task_tcb_s *)this_task();
  int exitcode = EXIT_FAILURE;
  int argc;

#ifdef CONFIG_ARM_TRUSTRAM_AW_RUNTIME
  if (TRUSTRAM_AW_RUNTIME_TASK(&tcb->cmn))
    {
      exit(board_aw_runtime_startup(&tcb->cmn));
    }
#endif
#ifdef CONFIG_ARM_TRUSTRAM_AW_BOOT
  if (TRUSTRAM_AW_TASK(&tcb->cmn))
    {
      /* The immutable assembly root has already admitted its AW shadow.
       * The board checks the retained creation recipe before calling main;
       * ordinary TCB entry/argv fields are not independent authority.
       */

      exit(board_aw_boot_startup(&tcb->cmn));
    }
#endif

  DEBUGASSERT((tcb->cmn.flags & TCB_FLAG_TTYPE_MASK) != \
              TCB_FLAG_TTYPE_PTHREAD);

#ifdef CONFIG_SIG_DEFAULT
  if ((tcb->cmn.flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_KERNEL)
    {
      /* Set up default signal actions for NON-kernel thread */

      nxsig_default_initialize(&tcb->cmn);
    }
#endif

#ifdef CONFIG_SCHED_TRUSTRAM_OBSERVE
  tr_observe_note((uintptr_t)tcb, TR_OBS_DISPATCH,
                  (uintptr_t)tcb->cmn.entry.main);
#endif

  /* Execute the start hook if one has been registered */

#ifdef CONFIG_SCHED_STARTHOOK
  if (tcb->starthook != NULL)
    {
      tcb->starthook(tcb->starthookarg);
    }
#endif

  /* Count how many non-null arguments we are passing. The first non-null
   * argument terminates the list .
   */

  argc = 1;
  while (tcb->cmn.group->tg_info->argv[argc])
    {
      /* Increment the number of args.  Here is a sanity check to
       * prevent running away with an unterminated argv[] list.
       * MAX_START_ARGS should be sufficiently large that this never
       * happens in normal usage.
       */

      if (++argc > MAX_START_ARGS)
        {
          exit(EXIT_FAILURE);
        }
    }

  /* Call the 'main' entry point passing argc and argv.  In the kernel build
   * this has to be handled differently if we are starting a user-space task;
   * we have to switch to user-mode before calling the task.
   */

  if ((tcb->cmn.flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL)
    {
      exitcode = tcb->cmn.entry.main(argc, tcb->cmn.group->tg_info->argv);
    }
  else
    {
#ifdef CONFIG_BUILD_FLAT
      nxtask_startup(tcb->cmn.entry.main, argc,
                     tcb->cmn.group->tg_info->argv);
#else
      up_task_start(tcb->cmn.entry.main, argc,
                    tcb->cmn.group->tg_info->argv);
#endif
    }

  /* Call exit() if/when the task returns */

  exit(exitcode);
#endif /* CONFIG_ARCH_TRUSTRAM_CONTEXT_HOOKS */
}

#endif /* TRUSTRAM_BOOT_TASK_HANDOFF_PROBE */
#endif /* TRUSTRAM_EXIT_FIRST_PROBE */
