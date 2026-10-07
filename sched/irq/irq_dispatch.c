/****************************************************************************
 * sched/irq/irq_dispatch.c
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

#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mm/mm.h>
#include <nuttx/random.h>
#include <nuttx/sched_note.h>

#include "irq/irq.h"
#include "clock/clock.h"
#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE
#  define NUSER_IRQS CONFIG_ARCH_NUSER_INTERRUPTS
#else
#  define NUSER_IRQS NR_IRQS
#endif

/* INCR_COUNT - Increment the count of interrupts taken on this IRQ number */

#ifndef CONFIG_SCHED_IRQMONITOR
#  define INCR_COUNT(ndx)
#elif defined(CONFIG_HAVE_LONG_LONG)
#  define INCR_COUNT(ndx) \
     do \
       { \
         g_irqvector[ndx].count++; \
       } \
     while (0)
#else
#  define INCR_COUNT(ndx) \
     do \
       { \
         if (++g_irqvector[ndx].lscount == 0) \
           { \
             g_irqvector[ndx].mscount++; \
           } \
       } \
     while (0)
#endif

/* CALL_VECTOR - Call the interrupt service routine attached to this
 * interrupt request
 */

#ifndef CONFIG_SCHED_CRITMONITOR_MAXTIME_IRQ
#  define CONFIG_SCHED_CRITMONITOR_MAXTIME_IRQ 0
#endif

#ifdef CONFIG_SCHED_IRQMONITOR
#  define CALL_VECTOR(ndx, vector, irq, context, arg) \
     do \
       { \
         struct timespec delta; \
         uint32_t start; \
         uint32_t elapsed; \
         start = up_perf_gettime(); \
         vector(irq, context, arg); \
         elapsed = up_perf_gettime() - start; \
         up_perf_convert(elapsed, &delta); \
         if (ndx < NUSER_IRQS) \
           { \
             INCR_COUNT(ndx); \
             if (delta.tv_nsec > g_irqvector[ndx].time) \
               { \
                 g_irqvector[ndx].time = delta.tv_nsec; \
               } \
           } \
         if (CONFIG_SCHED_CRITMONITOR_MAXTIME_IRQ > 0 && \
             elapsed > CONFIG_SCHED_CRITMONITOR_MAXTIME_IRQ) \
           { \
             serr("IRQ %d(%p), execute time too long %"PRIu32"\n", \
                  irq, vector, elapsed); \
           } \
       } \
     while (0)
#else
#  define CALL_VECTOR(ndx, vector, irq, context, arg) \
     vector(irq, context, arg)
#endif /* CONFIG_SCHED_IRQMONITOR */

#if defined(CONFIG_ARM_TRUSTRAM_AW_CPU) && defined(CONFIG_ARCH_CHIP_STM32H7)
/* These two handlers are exported by the STM32H7/PX4 CPU profile.  Keep the
 * registered vector as the selector so detach, rebind and IRQ chains still
 * take the existing indirect path.  SVC uses the protected checkpoint path
 * in this profile; arm_svcall is not registered or linked.
 */

int stm32_timerisr(int irq, FAR void *context, FAR void *arg);
int hrt_tim_isr(int irq, FAR void *context, FAR void *arg);
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: irq_dispatch
 *
 * Description:
 *   This function must be called from the architecture-specific logic in
 *   order to dispatch an interrupt to the appropriate, registered handling
 *   logic.
 *
 ****************************************************************************/

void irq_dispatch(int irq, FAR void *context)
{
  xcpt_t vector = irq_unexpected_isr;
  FAR void *arg = NULL;
  unsigned int ndx = irq;

#if NR_IRQS > 0
  if ((unsigned)irq < NR_IRQS)
    {
#ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE
      ndx = g_irqmap[irq];
      if (ndx < CONFIG_ARCH_NUSER_INTERRUPTS)
        {
          if (g_irqvector[ndx].handler)
            {
              vector = g_irqvector[ndx].handler;
              arg    = g_irqvector[ndx].arg;
            }
        }
#else
      if (g_irqvector[ndx].handler)
        {
          vector = g_irqvector[ndx].handler;
          arg    = g_irqvector[ndx].arg;
        }
#endif
    }
#endif

#ifdef CONFIG_CRYPTO_RANDOM_POOL_COLLECT_IRQ_RANDOMNESS
  /* Add interrupt timing randomness to entropy pool */

  add_irq_randomness(irq);
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
  /* Notify that we are entering into the interrupt handler */

  sched_note_irqhandler(irq, vector, true);
#endif

  /* Then dispatch to the interrupt handler */

#if defined(CONFIG_ARM_TRUSTRAM_AW_CPU) && defined(CONFIG_ARCH_CHIP_STM32H7)
  if (irq == STM32_IRQ_SYSTICK && vector == stm32_timerisr && arg == NULL)
    {
      CALL_VECTOR(ndx, stm32_timerisr, irq, context, arg);
    }
  else if (irq == STM32_IRQ_TIM8CC && vector == hrt_tim_isr && arg == NULL)
    {
      CALL_VECTOR(ndx, hrt_tim_isr, irq, context, arg);
    }
  else
#endif
    {
      CALL_VECTOR(ndx, vector, irq, context, arg);
    }
  UNUSED(ndx);

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
  /* Notify that we are leaving from the interrupt handler */

  sched_note_irqhandler(irq, vector, false);
#endif

#ifdef CONFIG_DEBUG_MM
  if ((g_running_tasks[this_cpu()]->flags & TCB_FLAG_HEAPCHECK) || \
       (this_task()->flags & TCB_FLAG_HEAPCHECK))
    {
      kmm_checkcorruption();
    }
#endif

  /* Record the new "running" task.  g_running_tasks[] is only used by
   * assertion logic for reporting crashes.
   */

  g_running_tasks[this_cpu()] = this_task();
}
