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
  __asm__ volatile ("b.w board_trustram_exit_first_startup");
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
#ifdef CONFIG_ARCH_TRUSTRAM_CONTEXT_HOOKS
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
