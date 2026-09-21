/* SPDX-License-Identifier: Apache-2.0 */
#ifndef __INCLUDE_NUTTX_TRUSTRAM_OBSERVE_H
#define __INCLUDE_NUTTX_TRUSTRAM_OBSERVE_H

#include <nuttx/config.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef CONFIG_SCHED_TRUSTRAM_OBSERVE
#if !defined(CONFIG_BUILD_FLAT) || !defined(CONFIG_ARCH_ARMV7M) || \
    defined(CONFIG_SMP) || defined(CONFIG_ARCH_HIPRI_INTERRUPT) || \
    defined(CONFIG_ARCH_TRUSTRAM_CONTEXT_HOOKS) || \
    defined(CONFIG_ARM_TRUSTRAM_NATIVE_BOOT) || \
    defined(CONFIG_ARM_TRUSTRAM_COOPERATIVE_TEST)
#  error "TRUST-RAM observation requires ordinary Flat single-CPU ARMv7-M"
#endif

#define TR_OBSERVE_RECORDS 256
#define TR_OBSERVE_LIVE 64
#define TR_OBSERVE_IDLE 3 /* Other kinds use native TCB_FLAG_TTYPE_* values. */

enum tr_observe_event
{
  TR_OBS_BEGIN = 1,
  TR_OBS_INITIALIZED,
  TR_OBS_ACTIVATE,
  TR_OBS_DISPATCH,
  TR_OBS_WORKER_ENTRY,
  TR_OBS_IDLE_LOOP,
  TR_OBS_TEST_ENTRY,
  TR_OBS_TEST_RETURN,
  TR_OBS_EXIT_HOOK,
  TR_OBS_CREATE_FAILED,
  TR_OBS_RELEASE_BEGIN,
  TR_OBS_RELEASE_END
};

/* Ordinary RAM diagnostics only. Neither addresses nor IDs confer authority.
 * Context is sampled before the recorder raises BASEPRI. No target is ever
 * dereferenced by the recorder, including after native free().
 */

struct tr_observe_record
{
  uint32_t sequence;
  uint32_t instance;
  uintptr_t tcb;
  uintptr_t value;
  uint16_t ipsr;
  uint8_t basepri;
  uint8_t primask;
  uint16_t event;
  uint8_t kind;
  uint8_t reserved;
};

struct tr_observe_token
{
  uintptr_t tcb;
  uint32_t instance;
  uint8_t kind;
};

struct tr_observe_status
{
  uint32_t first;
  uint32_t last;
  uint32_t overwritten;
  uint32_t untracked;
  uint32_t exhausted;
};

#ifdef __cplusplus
extern "C" {
#endif
void tr_observe_begin(uintptr_t tcb, uint8_t kind, uintptr_t entry);
void tr_observe_note(uintptr_t tcb, enum tr_observe_event event,
                     uintptr_t value);
struct tr_observe_token tr_observe_releasing(uintptr_t tcb);
void tr_observe_released(struct tr_observe_token token, int result);
void tr_observe_status(struct tr_observe_status *out);
bool tr_observe_read(uint32_t sequence, struct tr_observe_record *out);
#ifdef __cplusplus
}
#endif
#endif /* CONFIG_SCHED_TRUSTRAM_OBSERVE */
#endif
