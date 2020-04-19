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

#define GPIO_PORTS_CNT          8
#define GPIO_PINS_CNT           24

enum
{
    GPIO_USED,
    GPIO_ARM_LOCK,
    GPIO_ARISC_LOCK,
    GPIO_PORT_MAX_ID,
    GPIO_DATA_CNT
};

#define GPIO_SHM_BASE           (ARISC_SHM_BASE)
#define GPIO_SHM_SET_BASE       (GPIO_SHM_BASE     + 0)
#define GPIO_SHM_CLR_BASE       (GPIO_SHM_SET_BASE + GPIO_PORTS_CNT*4)
#define GPIO_SHM_OUT_BASE       (GPIO_SHM_CLR_BASE + GPIO_PORTS_CNT*4)
#define GPIO_SHM_INP_BASE       (GPIO_SHM_OUT_BASE + GPIO_PORTS_CNT*4)
#define GPIO_SHM_DATA_BASE      (GPIO_SHM_INP_BASE + GPIO_PORTS_CNT*4)
#define GPIO_SHM_SIZE           (GPIO_SHM_DATA_BASE + GPIO_DATA_CNT*4 - GPIO_SHM_SET_BASE)

enum { PA, PB, PC, PD, PE, PF, PG, PL };
enum { LOW, HIGH };




#endif
