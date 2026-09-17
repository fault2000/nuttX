/****************************************************************************
 * include/nuttx/trustram_boot_layout.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_TRUSTRAM_BOOT_LAYOUT_H
#define __INCLUDE_NUTTX_TRUSTRAM_BOOT_LAYOUT_H

#include <stddef.h>
#include <stdint.h>

#ifndef TRUSTRAM_BOOT_LAYOUT_PROBE
#  error "TRUST-RAM boot layout export requires an explicit detached probe"
#endif

/* Immutable layout of the actual private idle TCB, emitted by nx_start.c
 * only for an explicit detached link probe.  No ordinary TCB member is read.
 * This descriptor is neither a registration ticket nor execution authority;
 * it does not establish SP provenance, protected storage or root ancestry.
 * The normal build does not export this descriptor or expose g_idletcb.
 */

struct trustram_boot_idle_object
{
  uintptr_t identity;
  size_t bytes;
  size_t alignment;
};

#ifdef __cplusplus
extern "C"
{
#endif

extern const struct trustram_boot_idle_object g_trustram_boot_idle_object;

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_TRUSTRAM_BOOT_LAYOUT_H */
