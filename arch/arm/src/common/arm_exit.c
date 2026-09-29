/****************************************************************************
 * arch/arm/src/common/arm_exit.c
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

#include <sched.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/trustram_context.h>
#ifdef TRUSTRAM_EXIT_SOURCE_PROBE
#  include <nuttx/trustram_exit_source.h>
#  ifdef TRUSTRAM_EXIT_CLEANUP_PROBE
#    include <nuttx/trustram_exit_cleanup.h>
#  endif
#endif
#ifdef CONFIG_DUMP_ON_EXIT
#  include <nuttx/fs/fs.h>
#endif

#include "task/task.h"
#include "sched/sched.h"
#include "group/group.h"
#include "irq/irq.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_DEBUG_SCHED_INFO
#  undef CONFIG_DUMP_ON_EXIT
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _up_dumponexit
 *
 * Description:
 *   Dump the state of all tasks whenever on task exits.  This is debug
 *   instrumentation that was added to check file-related reference counting
 *   but could be useful again sometime in the future.
 *
 ****************************************************************************/

#ifdef CONFIG_DUMP_ON_EXIT
static void _up_dumponexit(struct tcb_s *tcb, void *arg)
{
  struct filelist *filelist;
  int i;
  int j;

  sinfo("  TCB=%p name=%s pid=%d\n", tcb, tcb->name, tcb->pid);
  sinfo("    priority=%d state=%d\n", tcb->sched_priority, tcb->task_state);

  filelist = &tcb->group->tg_filelist;
  for (i = 0; i < filelist->fl_rows; i++)
    {
      for (j = 0; j < CONFIG_NFILE_DESCRIPTORS_PER_BLOCK; j++)
        {
          struct inode *inode = filelist->fl_files[i][j].f_inode;
          if (inode)
            {
              sinfo("      fd=%d refcount=%d\n",
                    i * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK + j,
                    inode->i_crefs);
            }
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_exit
 *
 * Description:
 *   This function causes the currently executing task to cease
 *   to exist.  This is a special case of task_delete() where the task to
 *   be deleted is the currently executing task.  It is more complex because
 *   a context switch must be perform to the next ready to run task.
 *
 ****************************************************************************/

void up_exit(int status)
{
  struct tcb_s *tcb = this_task();

#ifdef CONFIG_ARM_TRUSTRAM_AW_RUNTIME
  if (TRUSTRAM_AW_RUNTIME_TASK(tcb))
    {
      board_aw_runtime_exit(status);
    }
#endif
#ifdef CONFIG_ARM_TRUSTRAM_AW_BOOT
  if (TRUSTRAM_AW_TASK(tcb))
    {
      /* A naked board entry moves to a private stack before native teardown.
       * It must invoke nxtask_exit(), validate the actual selected idle TCB,
       * then restore the protected capture rather than tcb->xcp.regs.
       */

      board_aw_boot_exit(status);
    }
#endif

#ifdef TRUSTRAM_EXIT_SOURCE_PROBE
  /* Exclusive boot probe: admit native exit. The cleanup extension runs
   * bounded native teardown and retained-target restore; otherwise stop
   * before the first writer. Backing reclamation stays disabled.
   */

  board_trustram_exit_source_exit_begin(tcb, status);
#ifdef TRUSTRAM_EXIT_CLEANUP_PROBE
  board_trustram_exit_cleanup_begin(tcb);
  int ret = nxtask_exit();
  struct tcb_s *next = this_task();
  board_trustram_exit_cleanup_finish(tcb, next, ret);
  board_trustram_exit_native_handoff(tcb, next);
#else
  board_trustram_exit_source_stop();
#endif
#else

#ifdef CONFIG_ARM_TRUSTRAM_AW_CPU
  /* Group/file cleanup may block. Complete its idempotent native hook while
   * the physical caller is still the current scheduler task. The later
   * nonblocking termination pass observes EXIT_PROCESSING and does no work. */
  nxtask_exithook(tcb, status, true);
#endif

  /* Make sure that we are in a critical section with local interrupts.
   * The IRQ state will be restored when the next task is started.
   */

  enter_critical_section();
#ifdef CONFIG_ARM_TRUSTRAM_AW_CPU
  aw_cpu_native_task_stop((uintptr_t)tcb);
#endif

#ifdef CONFIG_ARCH_TRUSTRAM_CONTEXT_HOOKS
  /* Retain the outgoing lifetime before native teardown changes this_task. */

  struct tcb_s *exiting = tcb;
  up_trustram_exit_begin(exiting);
#endif

  sinfo("TCB=%p exiting\n", tcb);

  /* Destroy the task at the head of the ready to run list. */

  nxtask_exit();

#ifdef CONFIG_DUMP_ON_EXIT
  sinfo("Other tasks:\n");
  nxsched_foreach(_up_dumponexit, NULL);
#endif

  /* Now, perform the context switch to the new ready-to-run task at the
   * head of the list.
   */

  tcb = this_task();

  /* Adjusts time slice for SCHED_RR & SCHED_SPORADIC cases
   * NOTE: the API also adjusts the global IRQ control for SMP
   */

  nxsched_resume_scheduler(tcb);
#ifdef CONFIG_ARM_TRUSTRAM_AW_CPU
  aw_cpu_native_task_exit();
#endif

  /* Then switch contexts */

#ifdef CONFIG_ARCH_TRUSTRAM_CONTEXT_HOOKS
  /* No next-TCB frame read and no native fallback after this boundary. */

  up_trustram_exit_handoff(exiting, tcb);
#else
  arm_fullcontextrestore(tcb->xcp.regs);
#endif
#endif /* TRUSTRAM_EXIT_SOURCE_PROBE */
}
