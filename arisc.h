#ifndef _ARISC_H
#define _ARISC_H

#include <stdint.h>



#define DEBUG 0




#define ARISC_FW_BASE           (0x00040000) // for ARM CPU it's 0x00040000
#define ARISC_FW_SIZE           ((8+8+32)*1024)
#define ARISC_SHM_SIZE          (4096)
#define ARISC_SHM_BASE          (ARISC_FW_BASE + ARISC_FW_SIZE - ARISC_SHM_SIZE)




// spinlock
#define SPINLOCK_CNT            (32)
#define SPINLOCK_BASE           (0x01C18000)
#define SPINLOCK_SYSTATUS_REG   (SPINLOCK_BASE + 0x0000)
#define SPINLOCK_STATUS_REG     (SPINLOCK_BASE + 0x0010)
#define SPINLOCK_LOCK_REG(N)    (SPINLOCK_BASE + 0x0100 + (N)*0x4)




#define GPIO_BASE               0x01c20800
#define GPIO_R_BASE             0x01f02c00
#define GPIO_BANK_SIZE          0x24

#define GPIO_PORTS_MAX_CNT      8
#define GPIO_PINS_MAX_CNT       24

enum
{
    GPIO_USED,
    GPIO_ARM_LOCK,
    GPIO_ARISC_LOCK,
    GPIO_PORTS_CNT,
    GPIO_DATA_CNT
};

#define GPIO_SPINLOCK_SOFT      0
#define GPIO_SPINLOCK_CHECK     0
#define GPIO_SPINLOCK_ID        (SPINLOCK_CNT - 1)
#define GPIO_SPINLOCK_MASK      (1UL << GPIO_SPINLOCK_ID)

#define GPIO_SHM_BASE           (ARISC_SHM_BASE)
#define GPIO_SHM_SET_BASE       (GPIO_SHM_BASE     + 0)
#define GPIO_SHM_CLR_BASE       (GPIO_SHM_SET_BASE + GPIO_PORTS_MAX_CNT*4)
#define GPIO_SHM_OUT_BASE       (GPIO_SHM_CLR_BASE + GPIO_PORTS_MAX_CNT*4)
#define GPIO_SHM_INP_BASE       (GPIO_SHM_OUT_BASE + GPIO_PORTS_MAX_CNT*4)
#define GPIO_SHM_DATA_BASE      (GPIO_SHM_INP_BASE + GPIO_PORTS_MAX_CNT*4)
#define GPIO_SHM_SIZE           (GPIO_SHM_DATA_BASE + GPIO_DATA_CNT*4 - GPIO_SHM_BASE)

enum { PA, PB, PC, PD, PE, PF, PG, PL };
enum { LOW, HIGH };

#define GPIO_PIN_SET(PORT,PIN_MASK) \
    *gpio[PORT] |= PIN_MASK

#define GPIO_PIN_CLR(PORT,PIN_MASK_NOT) \
    *gpio[PORT] &= PIN_MASK_NOT

#define GPIO_PIN_GET(PORT,PIN_MASK) \
    (*gpio[PORT] & PIN_MASK)




#define PG_CH_MAX_CNT 16

enum
{
    PG_PORT,
    PG_PIN_MSK,
    PG_PIN_MSKN,
    PG_TASK_TOGGLES,
    PG_TASK_T0,
    PG_TASK_T1,
    PG_TASK_TICK,
    PG_TASK_TIMEOUT,
    PG_PARAM_CNT
};

enum
{
    PG_USED,
    PG_ARM_LOCK,
    PG_ARISC_LOCK,
    PG_TIMER_FREQ,
    PG_TIMER_TICK,
    PG_CH_CNT,
    PG_DATA_CNT
};

#define PG_SPINLOCK_SOFT    0
#define PG_SPINLOCK_ID      (SPINLOCK_CNT - 2)
#define PG_SPINLOCK_MASK    (1UL << PG_SPINLOCK_ID)
#define PG_SPINLOCK_CHECK   0

#define PG_SHM_BASE         (ARISC_SHM_BASE + GPIO_SHM_SIZE)
#define PG_SHM_CH_BASE      (PG_SHM_BASE)
#define PG_SHM_DATA_BASE    (PG_SHM_CH_BASE + PG_CH_MAX_CNT*PG_PARAM_CNT*4)
#define PG_SHM_SIZE         (PG_SHM_DATA_BASE + PG_DATA_CNT*4)




typedef struct {
    int32_t  pos;
    uint32_t pg_ch[2];
    uint32_t port[2];
    uint32_t pin[2];
    uint32_t inv[2];
    uint32_t t0[2];
    uint32_t t1[2];
} _stepgen_ch_t;

enum { STEP, DIR };

#define STEPGEN_CH_MAX_CNT 8




#endif
