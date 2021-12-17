/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-09-22     Bernard      add board.h to this bsp
 */

// <<< Use Configuration Wizard in Context Menu >>>
#ifndef __BOARD_H__
#define __BOARD_H__

#include "r_typedefs.h"
#include "r_intc_lld_rza2m.h"

#ifdef __CC_ARM
extern int Image$$RTT_HEAP$$ZI$$Base;
extern int Image$$RTT_HEAP$$ZI$$Limit;
#define HEAP_BEGIN          (&Image$$RTT_HEAP$$ZI$$Base)
#define HEAP_END            (&Image$$RTT_HEAP$$ZI$$Limit)

#elif __ICCARM__
#pragma section="HEAP"
#define HEAP_BEGIN          (__segment_end("HEAP"))
extern void __RTT_HEAP_END;
#define HEAP_END            (&__RTT_HEAP_END)

#else
extern char   heap_start asm ("__rtthread_heap_start");  		/* Defined by the linker.  */
extern char   heap_end __asm ("__rtthread_heap_end");    		/* Defined by the linker.  */

#define HEAP_BEGIN          (&heap_start)
#define HEAP_END            (&heap_end)
#endif

#define HEAP_SIZE           (128 * 1024)

#define	RT_USING_GIC_V2
#define ARM_GIC_MAX_NR 		(1)
#define ARM_GIC_NR_IRQS		(INTC_GIC_ID_TOTAL)
#define MAX_HANDLERS		(INTC_GIC_ID_TOTAL)
#define GIC_IRQ_START   	(0)
#define GIC_ACK_INTID_MASK  (0x000003FF)

#define __REG32(x)  (*((volatile unsigned int *)(x)))

typedef void (*port_isr_handler_t)(uint32_t int_sense);

void rt_hw_board_init(void);
rt_uint32_t platform_get_gic_dist_base(void);
rt_uint32_t platform_get_gic_cpu_base(void);

#endif

