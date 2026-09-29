/* SPDX-License-Identifier: Apache-2.0 */
#ifndef __INCLUDE_NUTTX_TRUSTRAM_CPU_MASKS_H
#define __INCLUDE_NUTTX_TRUSTRAM_CPU_MASKS_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Fixed ordinary instrumented wrappers; the CPU compiler must lower their
 * typed intrinsics to the protected mask service. No raw-mask fallback. */
uint32_t aw_cpu_native_irq_save(void);
void aw_cpu_native_irq_restore(uint32_t);
void aw_cpu_native_irq_disable(void);
void aw_cpu_native_irq_enable(void);
#ifdef __cplusplus
}
#endif
#endif
