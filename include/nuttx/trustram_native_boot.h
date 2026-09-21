/* SPDX-License-Identifier: Apache-2.0 */
#ifndef __INCLUDE_NUTTX_TRUSTRAM_NATIVE_BOOT_H
#define __INCLUDE_NUTTX_TRUSTRAM_NATIVE_BOOT_H
#include <nuttx/config.h>
#ifdef CONFIG_ARM_TRUSTRAM_NATIVE_BOOT
struct tcb_s;
extern struct tcb_s *const g_trustram_native_boot_idle;
/* Exactly around nx_start's final scheduler release, under trusted boot
 * ownership. Not a runtime registration or general context interface. */
void board_trustram_native_boot_prepare(void);
void board_trustram_native_boot_complete(void);
#endif
#endif
