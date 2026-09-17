/****************************************************************************
 * include/nuttx/trustram_context.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_TRUSTRAM_CONTEXT_H
#define __INCLUDE_NUTTX_TRUSTRAM_CONTEXT_H

#include <nuttx/config.h>

#ifdef CONFIG_ARCH_TRUSTRAM_CONTEXT_HOOKS

/* Creation-transaction integration for host tests only. There is no runtime
 * implementation, Kconfig enablement, protected allocator, or CFI/MPU policy
 * behind these declarations. A test define must never enable them on ARM.
 */

#if !defined(TRUSTRAM_CONTEXT_HOST_TEST) || defined(__arm__) || defined(__thumb__)
#  error "TRUST-RAM context hooks are host-test only; runtime integration is unavailable"
#endif

#if !defined(CONFIG_BUILD_FLAT) || defined(CONFIG_BUILD_PROTECTED) || \
    defined(CONFIG_BUILD_KERNEL) || defined(CONFIG_SMP) || defined(CONFIG_PIC) || \
    defined(CONFIG_ARCH_ADDRENV) || defined(CONFIG_SCHED_HAVE_PARENT) || \
    defined(CONFIG_SCHED_CHILD_STATUS) || defined(CONFIG_SCHED_CPULOAD) || \
    defined(CONFIG_SCHED_SPORADIC) || defined(CONFIG_TLS_ALIGNED) || \
    defined(CONFIG_MM_KERNEL_HEAP) || defined(CONFIG_ARMV7M_STACKCHECK) || \
    defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SCHED_STARTHOOK) || \
    defined(CONFIG_HAVE_CXXINITIALIZE) || defined(CONFIG_ARCH_HIPRI_INTERRUPT) || \
    defined(CONFIG_MM_SHM) || defined(CONFIG_BINFMT_CONSTRUCTORS) || \
    (defined(CONFIG_ARCH_HAVE_VFORK) && defined(CONFIG_SCHED_WAITPID))
#  error "Unsupported TRUST-RAM context-hook rollback profile"
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct tcb_s;

/* Host-only TCB allocation/first-access boundaries. alloc claims one exact
 * boot-registered extent and retains its generation/type before the first
 * zero or write; success returns that complete extent zeroed. NULL denotes
 * ordinary exhaustion, not permission to fall back to the heap. Only fixed
 * concrete sizes and types from approved task/kernel/pthread callers are
 * accepted. No ordinary allocation, flags, PID or caller-supplied pointer can
 * establish provenance. Protected backing, allocator entry/CFI, serialization
 * and all-writer exclusion remain required and are not implemented here.
 *
 * create_type resolves the exact pointer against that retained claim and
 * generation in its pre-stack CLAIMED state before any ordinary field read.
 * It returns the retained TCB type or negative errno without changing state.
 * The pointer is an identifier, never caller authority. A newly zeroed
 * pthread TCB need not yet have its ordinary type flag mirror populated.
 */

void *up_trustram_tcb_alloc(size_t size, uint8_t ttype);
int up_trustram_tcb_create_type(struct tcb_s *tcb);

/* uninit validates or terminates before the inactive-list operation or any
 * TCB dereference. It requires the retained BOUND identity, sealed creation
 * and stack lease, and a RESERVED owner that has never run. Caller provenance
 * must authorize cancellation; ordinary inactive/PID fields are not proof.
 * Validation does not consume the binding. Abort/free retain it until every
 * subsequent cleanup writer has finished; replay/active/unknown is fatal.
 */

void up_trustram_context_check_uninit(struct tcb_s *tcb);

/* IRQ handoff boundaries for host integration only. enter validates or
 * terminates before ordinary LED/ack/dispatch callbacks. The interrupted
 * owner comes from protected active ownership, never the scheduler's possibly
 * already changed this_task()/CURRENT_REGS mirrors. A separately authorized,
 * immutable early capture must exist before any ordinary writer or callback;
 * copying a stack at this C boundary does NOT establish that provenance.
 * Nested entry is rejected by arm_doirq before the service is called.
 *
 * return validates the final candidate after every ordinary callback and
 * CURRENT_REGS clearing. Intermediate scheduler selections do not enter or
 * resume an owner. The service supplies only the approved ordinary frame,
 * restored from a protected initial image/continuation. Its protected source,
 * restore plan and every hardware-referenced frame remain live and immutable
 * until assembly has finished using them. A successful metadata handoff is
 * NOT hardware quiescence and never authorizes reclaiming an outgoing stack.
 * Failures terminate; there is no fallback to the candidate frame. Actual
 * assembly capture/restore, caller CFI and all-writer protection are absent.
 */

void up_trustram_irq_enter(int irq, uint32_t *regs);
uint32_t *up_trustram_irq_return(int irq, uint32_t *candidate);

/* Exact native function signatures for the supported Flat ABI. Keep this
 * header independent of scheduler/pthread structure definitions.
 */

typedef int (*trustram_main_t)(int argc, char **argv);
typedef void *(*trustram_pthread_entry_t)(void *arg);
typedef void (*trustram_pthread_trampoline_t)(trustram_pthread_entry_t entry,
                                            void *arg);

struct trustram_task_start_s
{
  trustram_main_t entry;
  int argc;
  char **argv;
  bool kernel;
};

struct trustram_pthread_start_s
{
  trustram_pthread_trampoline_t trampoline;
  trustram_pthread_entry_t entry;
  void *arg;
};

/* First-dispatch services validate or terminate. start validates the actual
 * registered TCB binding before ordinary field dereferences and supplies a
 * protected plan. It does not authorize execution from this_task() or PID.
 * A trusted argument-extent check and immutable snapshot/provenance are
 * required before any argv read. Pthread arg is opaque and not dereferenced.
 * Inputs, outputs, their lifetimes and callback entry require protection and
 * serialization; these raw C interfaces are not runtime security gates.
 * Final entry/dispatch checks return approved targets immediately before the
 * corresponding indirect call, including after a possible priority change.
 * Scheduler, group and signal services retain their own trust obligations.
 */

void up_trustram_task_start(struct tcb_s *tcb,
                           struct trustram_task_start_s *out);
trustram_main_t up_trustram_task_entry(trustram_main_t entry, int argc,
                                     char **argv, bool kernel);
void up_trustram_pthread_start(struct tcb_s *tcb,
                              struct trustram_pthread_start_s *out);
trustram_pthread_trampoline_t up_trustram_pthread_dispatch(
  trustram_pthread_trampoline_t trampoline, trustram_pthread_entry_t entry,
  void *arg);
trustram_pthread_entry_t up_trustram_pthread_entry(
  trustram_pthread_entry_t entry, void *arg);

struct trustram_stack_layout_s
{
  void *allocation;
  void *base;
  size_t size;
};

struct trustram_stack_frame_s
{
  void *frame;
  size_t size;
  struct trustram_stack_layout_s remaining;
};

struct trustram_initial_state_s
{
  struct trustram_stack_layout_s stack;
  uint32_t *regs;
  uint32_t words[53];
  bool idle;
};

/* initial validates or terminates before the first xcp/stack write. Its
 * approved plan comes from the protected creation or boot binding, never
 * from PID alone or ordinary stack/entry fields. A task plan contains the
 * complete fixed M7 initial frame; idle has no synthetic frame and no stack
 * coloration. idle_complete follows successful TLS setup and admits the
 * already-running boot owner without changing the machine stack pointer.
 */

void up_trustram_context_initial(struct tcb_s *tcb,
                                struct trustram_initial_state_s *out);
void up_trustram_idle_complete(struct tcb_s *tcb);

/* These host-fixture services authorize stack writes from a tracked lease,
 * before ordinary TCB updates, coloration or frame clearing. acquire rejects
 * existing bindings/nonfresh TCBs and validates the complete candidate span.
 * A NULL supplied pointer requests an allocation; otherwise storage remains
 * caller-owned. Errors acquire nothing and leave output/TCB/storage intact.
 * carve validates its output and remaining frame/headroom space before any
 * writes. FREE_STACK is only an ordinary descriptive mirror, not authority.
 */

int up_trustram_stack_acquire(struct tcb_s *tcb, void *supplied,
                             size_t requested, uint8_t ttype,
                             struct trustram_stack_layout_s *out);
int up_trustram_stack_carve(struct tcb_s *tcb, size_t requested,
                           struct trustram_stack_frame_s *out);

/* release validates or terminates; only the service decides allocator
 * ownership and whether storage must await handoff/quiescence. The final
 * TCB release follows the same deferral decision, never an ordinary flag.
 * Legitimate early failures can have no lease. Otherwise the service must
 * validate the bound lease even when the ordinary stack pointer is NULL.
 * Cancelled, never-running creation can reclaim its lease before release
 * returns but retains the TCB claim through all remaining cleanup writes.
 * nxtask_init callers use free_tcb only after that cleanup has finished.
 * Retiring execution defers both backing and TCB until handoff/quiescence.
 * free_tcb verifies detachment and retains a retiring TCB until reclaim;
 * there is no fallback to freeing from ordinary TCB pointers or flags.
 */

void up_trustram_stack_release(struct tcb_s *tcb);
void up_trustram_context_free_tcb(struct tcb_s *tcb);

/* prepare verifies an early stack lease and its already reserved owner. It
 * does not reserve a second owner. A negative errno leaves that early lease
 * for the caller's acquired-stack cleanup, without marking it prepared.
 * Original stack allocation extent is recorded before coloration/TLS/argv.
 * Ordinary TCB fields are not a trusted source in a runtime implementation;
 * allocation provenance remains a
 * required adapter contract. Callers track successful acquisition explicitly.
 */

int up_trustram_context_prepare(struct tcb_s *tcb, bool supplied_stack);

/* Common setup checks the reservation before PID/context publication. Seal
 * follows family-specific TLS/argument/trampoline setup, before activation.
 */

int up_trustram_context_check_create(struct tcb_s *tcb, uintptr_t start,
                                   uintptr_t entry, uint8_t ttype);
int up_trustram_context_seal(struct tcb_s *tcb);

/* Void callbacks must validate the transition or terminate. They must not
 * return after an invariant failure. abort is called only for an acquired
 * creation reservation. activate runs before making the task runnable.
 * release accepts an early failure with no acquired reservation; a LIVE
 * owner becomes RETIRING and retains capacity until a separate trusted
 * handoff/quiescence operation, not merely until the TCB has been freed.
 */

void up_trustram_context_abort(struct tcb_s *tcb);
void up_trustram_context_activate(struct tcb_s *tcb);
void up_trustram_context_release(struct tcb_s *tcb);

#endif /* CONFIG_ARCH_TRUSTRAM_CONTEXT_HOOKS */
#endif /* __INCLUDE_NUTTX_TRUSTRAM_CONTEXT_H */
