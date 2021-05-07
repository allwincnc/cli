#ifndef _ARISC_H
#define _ARISC_H

#include <stdint.h>



#define ARISC_CPU_FREQ          450000000 // Hz
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
    GPIO_FUNC_2,
    GPIO_FUNC_3,
    GPIO_FUNC_RESERVED4,
    GPIO_FUNC_RESERVED5,
    GPIO_FUNC_EINT,
    GPIO_FUNC_DISABLE,
    GPIO_FUNC_CNT
};

enum
{
    GPIO_MULTI_DRIVE_LEVEL0,
    GPIO_MULTI_DRIVE_LEVEL1,
    GPIO_MULTI_DRIVE_LEVEL2,
    GPIO_MULTI_DRIVE_LEVEL3,
    GPIO_MULTI_DRIVE_LEVEL_CNT
};

enum
{
    GPIO_PULL_DISABLE,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN,
    GPIO_PULL_RESERVED3,
    GPIO_PULL_CNT
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




#define PWM_CH_MAX_CNT 16
#define PWM_WASTED_TICKS (160/2) // number of ARISC ticks wasted for calculations

enum
{
    PWM_CH_POS,
    PWM_CH_TICK,
    PWM_CH_TIMEOUT,
    PWM_CH_STATE,
    PWM_CH_WATCHDOG,

    PWM_CH_P_PORT,
    PWM_CH_P_PIN_MSK,
    PWM_CH_P_PIN_MSKN,
    PWM_CH_P_INV,
    PWM_CH_P_T0,
    PWM_CH_P_T1,
    PWM_CH_P_STOP,
    PWM_CH_P_TICK,

    PWM_CH_D_PORT,
    PWM_CH_D_PIN_MSK,
    PWM_CH_D_PIN_MSKN,
    PWM_CH_D,
    PWM_CH_D_INV,
    PWM_CH_D_T0,
    PWM_CH_D_T1,
    PWM_CH_D_CHANGE,

    PWM_CH_DATA_CNT
};

enum
{
    PWM_TIMER_TICK,
    PWM_ARM_LOCK,
    PWM_ARISC_LOCK,
    PWM_CH_CNT,
    PWM_DATA_CNT
};

enum
{
    PWM_CH_STATE_IDLE,
    PWM_CH_STATE_P0,
    PWM_CH_STATE_P1,
    PWM_CH_STATE_D0,
    PWM_CH_STATE_D1
};

#define PWM_SHM_BASE         (ARISC_SHM_BASE)
#define PWM_SHM_DATA_BASE    (PWM_SHM_BASE)
#define PWM_SHM_CH_DATA_BASE (PWM_SHM_DATA_BASE + PWM_DATA_CNT*4)
#define PWM_SHM_SIZE         (PWM_SHM_CH_DATA_BASE + PWM_CH_MAX_CNT*PWM_CH_DATA_CNT*4)




#define ENC_CH_MAX_CNT 8

enum
{
    ENC_CH_BUSY,
    ENC_CH_POS,
    ENC_CH_AB_STATE,

    ENC_CH_A_PORT,
    ENC_CH_A_PIN_MSK,
    ENC_CH_A_INV,
    ENC_CH_A_ALL,
    ENC_CH_A_STATE,

    ENC_CH_B_USE,
    ENC_CH_B_PORT,
    ENC_CH_B_PIN_MSK,
    ENC_CH_B_STATE,

    ENC_CH_Z_USE,
    ENC_CH_Z_PORT,
    ENC_CH_Z_PIN_MSK,
    ENC_CH_Z_INV,
    ENC_CH_Z_ALL,
    ENC_CH_Z_STATE,

    ENC_CH_DATA_CNT
};

enum
{
    ENC_ARM_LOCK,
    ENC_ARISC_LOCK,
    ENC_CH_CNT,
    ENC_DATA_CNT
};

#define ENC_SHM_BASE         (PWM_SHM_BASE + PWM_SHM_SIZE)
#define ENC_SHM_DATA_BASE    (ENC_SHM_BASE)
#define ENC_SHM_CH_DATA_BASE (ENC_SHM_DATA_BASE + ENC_DATA_CNT*4)
#define ENC_SHM_SIZE         (ENC_SHM_CH_DATA_BASE + ENC_CH_MAX_CNT*ENC_CH_DATA_CNT*4 - ENC_SHM_BASE)




#endif
