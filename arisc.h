/**
 * @file    arisc.h
 *
 * @brief   ARISC firmware API header
 *
 * This test program implements an API to ARISC firmware
 */

#ifndef _ARISC_H
#define _ARISC_H

#include <stdint.h>




#define ARISC_FW_BASE           (0x00040000) // for ARM CPU it's 0x00040000
#define ARISC_FW_SIZE           ((8+8+32)*1024)
#define ARISC_SHM_SIZE          (4096)
#define ARISC_SHM_BASE          (ARISC_FW_BASE + ARISC_FW_SIZE - ARISC_SHM_SIZE)

#define GPIO_BASE               0x01C20800 // GPIO registers block start address
#define GPIO_R_BASE             0x01f02c00 // GPIO R registers block start address
#define GPIO_BANK_SIZE          0x24

#define GPIO_PORTS_CNT          8   // number of GPIO ports
#define GPIO_PINS_CNT           24  // number of GPIO port pins

#define GPIO_SHM_BASE           (ARISC_SHM_BASE)
#define GPIO_SHM_SET_BASE       (GPIO_SHM_BASE     + GPIO_PORTS_CNT*4)
#define GPIO_SHM_CLR_BASE       (GPIO_SHM_SET_BASE + GPIO_PORTS_CNT*4)
#define GPIO_SHM_OUT_BASE       (GPIO_SHM_CLR_BASE + GPIO_PORTS_CNT*4)
#define GPIO_SHM_INP_BASE       (GPIO_SHM_OUT_BASE + GPIO_PORTS_CNT*4)
#define GPIO_SHM_SIZE           (GPIO_SHM_INP_BASE + GPIO_PORTS_CNT*4)

// GPIO port names
enum { PA, PB, PC, PD, PE, PF, PG, PL };

// GPIO pin states
enum { LOW, HIGH };




#endif
