#ifndef _ARISC_H
#define _ARISC_H

#include <stdint.h>



#define ARISC_FW_BASE           (0x00040000) // for ARM CPU it's 0x00040000
#define ARISC_FW_SIZE           ((8+8+32)*1024)
#define ARISC_SHM_SIZE          (4096)
#define ARISC_SHM_BASE          (ARISC_FW_BASE + ARISC_FW_SIZE - ARISC_SHM_SIZE)




#define GPIO_BASE               0x01c20800
#define GPIO_R_BASE             0x01f02c00
#define GPIO_BANK_SIZE          0x24

#define GPIO_PORTS_MAX_CNT      8
#define GPIO_PINS_MAX_CNT       24

enum
{
    GPIO_FUNC_IN,
    GPIO_FUNC_OUT,
    GPIO_FUNC_F2,
    GPIO_FUNC_F3,
    GPIO_FUNC_F4,
    GPIO_FUNC_F5,
    GPIO_FUNC_EINT
};

enum { PA, PB, PC, PD, PE, PF, PG, PL };
enum { LOW, HIGH };

#define GPIO_PIN_SET(PORT,PIN_MASK) \
    _GPIO[PORT]->data |= PIN_MASK

#define GPIO_PIN_CLR(PORT,PIN_MASK_NOT) \
    _GPIO[PORT]->data &= PIN_MASK_NOT

#define GPIO_PIN_GET(PORT,PIN_MASK) \
    (_GPIO[PORT]->data & PIN_MASK)

typedef struct
{
    uint32_t config[4];
    uint32_t data;
    uint32_t drive[2];
    uint32_t pull[2];
} _GPIO_PORT_REG_t;




#define PG_CH_MAX_CNT       16
#define PG_CH_SLOT_MAX_CNT  4
#define PG_CH_SLOT_MAX      (PG_CH_SLOT_MAX_CNT - 1)

enum
{
    PG_PORT,
    PG_PIN_MSK,
    PG_PIN_MSKN,
    PG_TOGGLES,
    PG_T0,
    PG_T1,
    PG_TICK,
    PG_TIMEOUT,
    PG_CH_DATA_CNT
};

enum
{
    PG_TIMER_TICK,
    PG_ARM_LOCK,
    PG_ARISC_LOCK,
    PG_CH_CNT,
    PG_DATA_CNT
};

#define PG_SHM_BASE         (ARISC_SHM_BASE)
#define PG_SHM_CH_SLOT_BASE (PG_SHM_BASE)
#define PG_SHM_CH_DATA_BASE (PG_SHM_CH_SLOT_BASE + PG_CH_MAX_CNT*4)
#define PG_SHM_DATA_BASE    (PG_SHM_CH_DATA_BASE + PG_CH_MAX_CNT*PG_CH_DATA_CNT*PG_CH_SLOT_MAX_CNT*4)
#define PG_SHM_SIZE         (PG_SHM_DATA_BASE + PG_DATA_CNT*4)




typedef struct {
    uint32_t busy;
    int32_t  pos;
    int32_t  dir;
    uint32_t port[2];
    uint32_t pin_msk[2];
    uint32_t pin_mskn[2];
} _stepgen_ch_t;

enum { STEP, DIR };

#define STEPGEN_CH_MAX_CNT 16




#endif
