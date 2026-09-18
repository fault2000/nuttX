/****************************************************************************
 * include/nuttx/trustram_cooperative.h
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_TRUSTRAM_COOPERATIVE_H
#define __INCLUDE_NUTTX_TRUSTRAM_COOPERATIVE_H

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#ifdef CONFIG_ARM_TRUSTRAM_COOPERATIVE_TEST

/* These hooks serve only the explicitly armed board diagnostic. The board
 * freezes ordinary tasks and admits its closed scheduling domain before
 * making active() true. Native queue operations remain in NuttX; the bridge
 * checks their selected save/restore operands against protected admission.
 * Neither operand nor the operation number grants execution authority.
 */

enum trustram_coop_scheduler_operation
{
  TRUSTRAM_COOP_SCHED_BLOCK = 1,
  TRUSTRAM_COOP_SCHED_UNBLOCK = 2
};

#ifdef __cplusplus
extern "C"
{
#endif

bool board_trustram_coop_scheduler_active(void);
struct tcb_s;
/* IRQ admission precedes all native queue mutation. Selection is a checked
 * result of native scheduling, never permission to write/read either operand. */
void board_trustram_coop_scheduler_irq_admit(struct tcb_s *tcb);
void board_trustram_coop_scheduler_irq_select(uint32_t **save,
                                             uint32_t *restore);
void board_trustram_coop_scheduler_switch(uint32_t **save,
                                         uint32_t *restore,
                                         unsigned operation);
void board_trustram_boot_fault(void) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_ARM_TRUSTRAM_COOPERATIVE_TEST */
#endif /* __INCLUDE_NUTTX_TRUSTRAM_COOPERATIVE_H */
