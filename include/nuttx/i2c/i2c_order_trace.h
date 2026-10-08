/* Experimental I2C completion-order observations, never protection authority.
 * Enabled only by the diagnostic image producer. No MMIO, cache maintenance,
 * logging calls or extra interrupt masking are introduced by this header.
 */
#ifndef __INCLUDE_NUTTX_I2C_I2C_ORDER_TRACE_H
#define __INCLUDE_NUTTX_I2C_I2C_ORDER_TRACE_H

#include <nuttx/config.h>

#if defined(CONFIG_ARM_TRUSTRAM_AW_CPU) && \
    defined(AW_CPU_I2C_ORDER_TRACE_EXPERIMENT)
#define AW_I2C_ORDER_ENABLED 1
#include <stdint.h>
#include <nuttx/clock.h>

#if defined(CONFIG_SMP) || defined(CONFIG_SYSTEM_TIME64) || \
    !defined(CONFIG_ARMV7M_USEBASEPRI) || defined(CONFIG_I2C_POLLED)
#error I2C order experiment requires the reviewed single-core BASEPRI tick build
#endif

enum aw_i2c_order_event
{
  AW_I2C_TRY_RETURN = 1,
  AW_I2C_ARM_BEFORE = 2,
  AW_I2C_ARM_AFTER = 3,
  AW_I2C_SEM_WAIT_RETURN = 4,
  AW_I2C_TIMEOUT_BEFORE = 5,
  AW_I2C_TIMEOUT_AFTER = 6,
  AW_I2C_POST_ENTER = 7,
  AW_I2C_POST_WAKE = 8,
  AW_I2C_POST_RETURN = 9,
  AW_I2C_DRIVER_WAITING = 10,
  AW_I2C_DRIVER_WAIT_RETURN = 11,
  AW_I2C_DRIVER_IDLE = 12
};

#define AW_I2C_ORDER_MAX 64
#define AW_I2C_ORDER_ISR_MAX 16
struct aw_i2c_order_row
{
  uint32_t event;
  uint32_t tick;
  uint32_t a;
  uint32_t b;
};

struct aw_i2c_order_trace
{
  uintptr_t active_sem;
  uint32_t taken;
  uint32_t used;
  uint32_t overflow;
  uint32_t start_tick;
  uint32_t end_tick;
  uint32_t isr_used;
  uint32_t isr_overflow;
  struct aw_i2c_order_row rows[AW_I2C_ORDER_MAX];
  struct aw_i2c_order_row isr[AW_I2C_ORDER_ISR_MAX];
};

extern volatile struct aw_i2c_order_trace g_aw_i2c_order;

/* Every caller is inside an EXISTING NuttX critical section. Only these
 * serialized decision points share the index. ISR progress has its own lane.
 * Volatile publishes the complete row before its count; CPU export after the
 * transfer avoids interpreting debugger reads of dirty cache as CPU values.
 */
static inline __attribute__((always_inline))
void aw_i2c_order_record(uintptr_t sem, uint32_t event,
                         uint32_t a, uint32_t b)
{
  if (sem != 0 && g_aw_i2c_order.active_sem == sem)
    {
      uint32_t i = g_aw_i2c_order.used;
      if (i < AW_I2C_ORDER_MAX)
        {
          g_aw_i2c_order.rows[i].event = event;
          g_aw_i2c_order.rows[i].tick = clock_systime_ticks();
          g_aw_i2c_order.rows[i].a = a;
          g_aw_i2c_order.rows[i].b = b;
          g_aw_i2c_order.used = i + 1;
        }
      else
        {
          g_aw_i2c_order.overflow = 1;
        }
    }
}

#define AW_I2C_ORDER(s, e, a, b) \
  aw_i2c_order_record((uintptr_t)(s), (e), (uint32_t)(a), (uint32_t)(b))
#else
#define AW_I2C_ORDER(s, e, a, b) do { } while (0)
#endif
#endif
