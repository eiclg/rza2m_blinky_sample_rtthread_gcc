/**********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO
 * THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2018 Renesas Electronics Corporation. All rights reserved.
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * File Name    : port.c
 * Device(s)    : RZ/A2M
 * Tool-Chain   : e2Studio Ver 7.6.0
 *              : GNU Arm Embedded Toolchain 6-2017-q2-update
 * OS           : None
 * H/W Platform : RZ/A2M Evaluation Board
 * Description  : RZ/A2M Sample Program - RT-Thread Port
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 *********************************************************************************************************************/
#include "rtdef.h"
#include "rtthread.h"
#include "board.h"
#include "stdio.h"

/**********************************************************************************************************************
 Typedef definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/


/**********************************************************************************************************************
 Imported global variables and functions (from other files)
 *********************************************************************************************************************/
extern void SysTick_Handler(void);
extern void rt_hw_console_output(const char *str);

/**********************************************************************************************************************
 Exported global variables and functions (to be accessed by other files)
 *********************************************************************************************************************/
/* The following constants describe the hardware, and are correct for the
Renesas RZ MPU. */
#define configINTERRUPT_CONTROLLER_BASE_ADDRESS 0xE8221000
#define configINTERRUPT_CONTROLLER_CPU_INTERFACE_OFFSET 0x1000


/**********************************************************************************************************************
 Private global variables and functions
 *********************************************************************************************************************/
/* This is the timer interrupt service routine. */
void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

/* the basic constants needed by gic */
rt_uint32_t platform_get_gic_dist_base(void)
{
	return (rt_uint32_t)(configINTERRUPT_CONTROLLER_BASE_ADDRESS);
}

rt_uint32_t platform_get_gic_cpu_base(void)
{
    return (rt_uint32_t)(configINTERRUPT_CONTROLLER_BASE_ADDRESS + configINTERRUPT_CONTROLLER_CPU_INTERFACE_OFFSET);
}

void rt_hw_console_output(const char *str)
{
	rt_tick_t tick = rt_tick_get();
	float t = (float)(tick) / 1000.0f;
	printf("[%5.3f]%s",t,str);
}
/* End of File */
