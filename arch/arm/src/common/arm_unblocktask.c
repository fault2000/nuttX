/****************************************************************************
 * arch/arm/src/common/arm_unblocktask.c
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

#include <sched.h>
#include <assert.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>

#ifdef CONFIG_ARM_TRUSTRAM_COOPERATIVE_TEST
#  include <nuttx/trustram_cooperative.h>
#endif

#include "sched/sched.h"
#include "group/group.h"
#include "clock/clock.h"
#include "arm_internal.h"

#ifdef TRUSTRAM_BOOT_WORKER_WAKE_PROBE
#  include <nuttx/trustram_boot_worker_wake.h>
#endif

/****************************************************************************
 * Name: up_unblock_task
 *
 * Description:
 *   A task is currently in an inactive task list
 *   but has been prepped to execute.  Move the TCB to the
 *   ready-to-run list, restore its context, and start execution.
 *
 * Input Parameters:
 *   tcb: Refers to the tcb to be unblocked.  This tcb is
 *     in one of the waiting tasks lists.  It must be moved to
 *     the ready-to-run list and, if it is the highest priority
 *     ready to run task, executed.
 *
 ****************************************************************************/

void up_unblock_task(struct tcb_s *tcb)
{
  struct tcb_s *rtcb = this_task();

#ifdef CONFIG_ARM_TRUSTRAM_COOPERATIVE_TEST
  /* Admit only the protected early-captured diagnostic IRQ request before
   * changing any queue, including when no context switch would be needed. */

  if (board_trustram_coop_scheduler_active() &&
      (CURRENT_REGS != NULL || getipsr() != 0))
    {
      board_trustram_coop_scheduler_irq_admit(tcb);
    }
#endif

  /* Verify that the context switch can be performed */

  DEBUGASSERT((tcb->task_state >= FIRST_BLOCKED_STATE) &&
              (tcb->task_state <= LAST_BLOCKED_STATE));

  /* Remove the task from the blocked task list */

  nxsched_remove_blocked(tcb);

  /* Add the task in the correct location in the prioritized
   * ready-to-run task list
   */

  if (nxsched_add_readytorun(tcb))
    {
      /* The currently active task has changed! We need to do
       * a context switch to the new task.
       */

      /* Update scheduler parameters */

      nxsched_suspend_scheduler(rtcb);

      /* Are we in an interrupt handler? */

      if (CURRENT_REGS)
        {
#ifdef TRUSTRAM_BOOT_WORKER_WAKE_PROBE
          board_trustram_boot_fault();
#else
#ifdef CONFIG_ARM_TRUSTRAM_COOPERATIVE_TEST
          if (board_trustram_coop_scheduler_active())
            {
              struct tcb_s *nexttcb = this_task();

              nxsched_resume_scheduler(nexttcb);
              board_trustram_coop_scheduler_irq_select(&rtcb->xcp.regs,
                                                       nexttcb->xcp.regs);
            }
          else
#endif
            {
              /* The ordinary IRQ path retains its native frame handling. */
              arm_savestate(rtcb->xcp.regs);

              /* Restore the exception context of the rtcb at the (new) head
               * of the ready-to-run task list.
               */

              rtcb = this_task();

              /* Update scheduler parameters */

              nxsched_resume_scheduler(rtcb);

              /* Then switch contexts.  Any necessary address environment
               * changes will be made when the interrupt returns.
               */

              arm_restorestate(rtcb->xcp.regs);
            }
#endif
        }
      /* No, then we will need to perform the user context switch */

      else
        {
          struct tcb_s *nexttcb = this_task();

          /* Update scheduler parameters */

          nxsched_resume_scheduler(nexttcb);

          /* Switch context to the context of the task at the head of the
           * ready to run list.
           */

#ifdef CONFIG_ARM_TRUSTRAM_COOPERATIVE_TEST
          if (board_trustram_coop_scheduler_active())
            {
              board_trustram_coop_scheduler_switch(&rtcb->xcp.regs,
                nexttcb->xcp.regs, TRUSTRAM_COOP_SCHED_UNBLOCK);
            }
          else
#endif
            {
              arm_switchcontext(&rtcb->xcp.regs, nexttcb->xcp.regs);
            }

#if defined(TRUSTRAM_BOOT_WORKER_CYCLE_PROBE) && defined(__arm__)
          __asm__ __volatile__(".global board_trustram_boot_worker_wake_native_return\n"
                               ".hidden board_trustram_boot_worker_wake_native_return\n"
                               "board_trustram_boot_worker_wake_native_return:\n" ::: "memory");
#endif
        }
    }
}
