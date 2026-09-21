/* SPDX-License-Identifier: Apache-2.0 */
#include <nuttx/trustram_observe.h>
#include <nuttx/irq.h>

/* Append-only sequence numbers over a bounded ring; live lookup is bounded
 * too. All exhaustion is diagnostic loss, never a native failure or panic.
 * Single CPU, no high-priority IRQ producers: up_irq_save serializes writers
 * without scheduler/TLS/heap dependencies during early idle initialization.
 */

static struct tr_observe_record g_tr_observe_records[TR_OBSERVE_RECORDS];
static struct tr_observe_token g_tr_observe_live[TR_OBSERVE_LIVE];
static struct tr_observe_status g_tr_observe_status;
static uint32_t g_tr_observe_instance;

struct observe_context
{
  uint16_t ipsr;
  uint8_t basepri;
  uint8_t primask;
};

static struct observe_context context(void)
{
  struct observe_context value = {getipsr(), getbasepri(), getprimask()};
  return value;
}

static void increment(uint32_t *value)
{
  if (*value != UINT32_MAX)
    {
      ++*value;
    }
}

static void append(struct tr_observe_token token, enum tr_observe_event event,
                   uintptr_t value, struct observe_context ctx)
{
  struct tr_observe_record *record;
  uint32_t sequence;

  if (g_tr_observe_status.last == UINT32_MAX)
    {
      increment(&g_tr_observe_status.exhausted);
      return;
    }

  sequence = ++g_tr_observe_status.last;
  record = &g_tr_observe_records[(sequence - 1) % TR_OBSERVE_RECORDS];
  record->sequence = sequence;
  record->instance = token.instance;
  record->tcb = token.tcb;
  record->value = value;
  record->ipsr = ctx.ipsr;
  record->basepri = ctx.basepri;
  record->primask = ctx.primask;
  record->event = event;
  record->kind = token.kind;
  record->reserved = 0;
  if (sequence > TR_OBSERVE_RECORDS)
    {
      increment(&g_tr_observe_status.overwritten);
    }

  g_tr_observe_status.first = sequence > TR_OBSERVE_RECORDS ?
                   sequence - TR_OBSERVE_RECORDS + 1 : 1;
}

static int lookup(uintptr_t tcb)
{
  int i;
  for (i = 0; i < TR_OBSERVE_LIVE; i++)
    {
      if (g_tr_observe_live[i].instance != 0 && g_tr_observe_live[i].tcb == tcb)
        {
          return i;
        }
    }

  return -1;
}

void tr_observe_begin(uintptr_t tcb, uint8_t kind, uintptr_t entry)
{
  struct observe_context ctx = context();
  struct tr_observe_token token = {tcb, 0, kind};
  irqstate_t flags = up_irq_save();
  int slot = lookup(tcb);
  int i;

  if (slot >= 0)
    {
      /* A missing release or repeated init must not reuse an old identity. */
      increment(&g_tr_observe_status.untracked);
      g_tr_observe_live[slot].instance = 0;
    }
  else
    {
      for (i = 0; i < TR_OBSERVE_LIVE; i++)
        {
          if (g_tr_observe_live[i].instance == 0)
            {
              slot = i;
              break;
            }
        }
    }

  if (slot < 0 || tcb == 0 || g_tr_observe_instance == UINT32_MAX)
    {
      increment(&g_tr_observe_status.untracked);
    }
  else
    {
      token.instance = ++g_tr_observe_instance;
      g_tr_observe_live[slot] = token;
    }

  append(token, TR_OBS_BEGIN, entry, ctx);
  up_irq_restore(flags);
}

void tr_observe_note(uintptr_t tcb, enum tr_observe_event event,
                     uintptr_t value)
{
  struct observe_context ctx = context();
  struct tr_observe_token token = {tcb, 0, 0};
  irqstate_t flags = up_irq_save();
  int slot = lookup(tcb);

  if (slot >= 0)
    {
      token = g_tr_observe_live[slot];
    }
  else
    {
      increment(&g_tr_observe_status.untracked);
    }

  append(token, event, value, ctx);
  up_irq_restore(flags);
}

struct tr_observe_token tr_observe_releasing(uintptr_t tcb)
{
  struct observe_context ctx = context();
  struct tr_observe_token token = {tcb, 0, 0};
  irqstate_t flags = up_irq_save();
  int slot = lookup(tcb);

  if (slot >= 0)
    {
      token = g_tr_observe_live[slot];
      g_tr_observe_live[slot].instance = 0;
    }
  else
    {
      increment(&g_tr_observe_status.untracked);
    }

  append(token, TR_OBS_RELEASE_BEGIN, 0, ctx);
  up_irq_restore(flags);
  return token;
}

void tr_observe_released(struct tr_observe_token token, int result)
{
  struct observe_context ctx = context();
  irqstate_t flags = up_irq_save();

  /* The token was copied before native cleanup. No freed pointer evaluation
   * or lookup by a potentially reused address is needed here. */
  append(token, TR_OBS_RELEASE_END, (uintptr_t)result, ctx);
  up_irq_restore(flags);
}

void tr_observe_status(struct tr_observe_status *out)
{
  irqstate_t flags = up_irq_save();
  *out = g_tr_observe_status;
  up_irq_restore(flags);
}

bool tr_observe_read(uint32_t sequence, struct tr_observe_record *out)
{
  irqstate_t flags = up_irq_save();
  bool valid = sequence != 0 && sequence >= g_tr_observe_status.first &&
               sequence <= g_tr_observe_status.last;
  if (valid)
    {
      *out = g_tr_observe_records[(sequence - 1) % TR_OBSERVE_RECORDS];
    }

  up_irq_restore(flags);
  return valid;
}
