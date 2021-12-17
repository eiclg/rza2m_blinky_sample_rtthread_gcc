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
 * File Name    : main.c
 * Device(s)    : RZ/A2M
 * Tool-Chain   : e2Studio Ver 7.6.0
 *              : GNU Arm Embedded Toolchain 6-2017-q2-update
 * OS           : None
 * H/W Platform : RZ/A2M Evaluation Board
 * Description  : RZ/A2M Sample Program - Main
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 *********************************************************************************************************************/
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include "r_typedefs.h"
#include "iodefine.h"
#include "r_cpg_drv_api.h"
#include "r_ostm_drv_api.h"
#include "r_scifa_drv_api.h"
#include "r_gpio_drv_api.h"
#include "r_startup_config.h"
#include "compiler_settings.h"
#include "main.h"
#include "r_os_abstraction_api.h"
#include "r_task_priority.h"
#include "command.h"

#include "iodefine.h"
#include "rtdef.h"

/**********************************************************************************************************************
 Typedef definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/
#define MAIN_PRV_LED_ON     (1)
#define MAIN_PRV_LED_OFF    (0)

/**********************************************************************************************************************
 Imported global variables and functions (from other files)
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Exported global variables and functions (to be accessed by other files)
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Private global variables and functions
 *********************************************************************************************************************/
static uint32_t s_main_led_flg;      /* LED lighting/turning off */
static int_t s_my_gpio_handle;
static st_r_drv_gpio_pin_rw_t s_p60_hi =
{
    GPIO_PORT_6_PIN_0,
    GPIO_LEVEL_HIGH,
    GPIO_SUCCESS
};
static st_r_drv_gpio_pin_rw_t s_p60_lo =
{
    GPIO_PORT_6_PIN_0,
    GPIO_LEVEL_LOW,
    GPIO_SUCCESS
};
static const r_gpio_port_pin_t s_led_pin_list[] =
{
    GPIO_PORT_6_PIN_0,
};

/* Terminal window escape sequences */
static const char_t * const sp_clear_screen = "\x1b[2J";
static const char_t * const sp_cursor_home  = "\x1b[H";

extern int thread_delete_init();
extern rt_err_t thread_delay_init();
extern int thread_priority_init();
extern rt_err_t thread_same_priority_init();
extern int thread_resume_init();
extern int thread_yield_init();
extern int timer_timeout_init();
extern void timer_create_init();
extern void timer_control_init();
extern int semaphore_dynamic_init();
extern int semaphore_priority_init();
extern int semaphore_producer_consumer_init();
extern int mutex_simple_init();
extern int mbox_send_wait_init();
extern void heap_malloc_init();
extern int event_simple_init();

/**********************************************************************************************************************
 * Function Name: os_rt_test_task_t
 * Description  : rt thread test task
 * Arguments    : none
 * Return Value : 0
 *********************************************************************************************************************/
int_t os_rt_test_task_t(void)
{
    /* never exits */
    while (1)
    {
        /* ==== Receive command, activate sample software ==== */
        printf("==== thread_delete test start ====\n");
    	thread_delete_init();
    	R_OS_TaskSleep(5000);
    	printf("==== thread_delete test end ====\n");

    	printf("==== thread_delay test start ====\n");
    	thread_delay_init();
    	R_OS_TaskSleep(5000);
    	printf("==== thread_delay test end ====\n");

    	printf("==== thread_priority test start ====\n");
    	thread_priority_init();
    	R_OS_TaskSleep(5000);
    	printf("==== thread_priority test end ====\n");

    	printf("==== thread_same_priority test start ====\n");
    	thread_same_priority_init();
    	R_OS_TaskSleep(5000);
    	printf("==== thread_same_priority test end ====\n");

    	printf("==== thread_resume test start ====\n");
    	thread_resume_init();
    	R_OS_TaskSleep(5000);
    	printf("==== thread_resume test end ====\n");

    	printf("==== thread_yield test start ====\n");
    	thread_yield_init();
    	R_OS_TaskSleep(5000);
    	printf("==== thread_yield test end ====\n");

    	printf("==== timer_timeout test start ====\n");
    	timer_timeout_init();
    	R_OS_TaskSleep(5000);
    	printf("==== timer_timeout test end ====\n");

    	printf("==== timer_create test start ====\n");
    	timer_create_init();
    	printf("==== timer_create test end ====\n");
    	R_OS_TaskSleep(5000);

    	printf("==== timer_control test start ====\n");
    	timer_control_init();
    	printf("==== timer_control test end ====\n");
    	R_OS_TaskSleep(5000);

    	printf("==== semaphore_dynamic test start ====\n");
    	semaphore_dynamic_init();
    	printf("==== semaphore_dynamic test end ====\n");
    	R_OS_TaskSleep(5000);

    	printf("==== semaphore_priority test start ====\n");
    	semaphore_priority_init();
    	printf("==== semaphore_priority test end ====\n");
    	R_OS_TaskSleep(5000);

    	printf("==== semaphore_producer_consumer test start ====\n");
    	semaphore_producer_consumer_init();
    	printf("==== semaphore_producer_consumer test end ====\n");
    	R_OS_TaskSleep(5000);

    	printf("==== mutex_priority test start ====\n");
    	mutex_simple_init();
    	printf("==== mutex_priority test end ====\n");
    	R_OS_TaskSleep(5000);

    	printf("==== mbox_send_wait test start ====\n");
    	mbox_send_wait_init();
    	printf("==== mbox_send_wait test end ====\n");
    	R_OS_TaskSleep(5000);

    	printf("==== heap_malloc test start ====\n");
    	heap_malloc_init();
    	printf("==== heap_malloc test end ====\n");
    	R_OS_TaskSleep(5000);

    	printf("==== event_simple test start ====\n");
    	event_simple_init();
    	printf("==== event_simple test end ====\n");
    	R_OS_TaskSleep(5000);


    	while (1)
    	{
    		R_OS_TaskSleep(500);
    	}
    }

    return (0);
}
/**********************************************************************************************************************
 End of function os_rt_test_task_t
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: os_main_task_t
 * Description  : FreeRTOS main task called by R_OS_KernelInit()
 *              : FreeRTOS is now configured and R_OS_Abstraction calls
 *              : can be used.
 *              : From this point forward no longer use direct_xxx calls.
 *              : For example
 *              : in place of   direct_open("ostm2", O_RDWR);
 *              : use           open(DEVICE_INDENTIFIER "ostm2", O_RDWR);
 *              :
 * Arguments    : none
 * Return Value : 0
 *********************************************************************************************************************/
int_t os_main_task_t(void)
{
    int_t err;
    st_r_drv_gpio_pin_list_t pin_led;
    char_t data;

    /* For information only
     * Use stdio calls to open drivers once  the kernel is initialised
     *
     * i.e.
     * int_t ostm3_handle;
     * ostm3_handle = open (DEVICE_INDENTIFIER "ostm2", O_RDWR);
     * close (ostm3_handle);
     */

    s_my_gpio_handle = open(DEVICE_INDENTIFIER "gpio", O_RDWR);

    /* On error */
    if (s_my_gpio_handle < 0)
    {
        /* stop execute */
        while (1)
        {
            R_COMPILER_Nop();
        }
    }

    /**************************************************
     * Initialise P6_0 pin parameterised in GPIO_SC_TABLE_MANUAL
     **************************************************/
    pin_led.p_pin_list = s_led_pin_list;
    pin_led.count = (sizeof(s_led_pin_list)) / (sizeof(s_led_pin_list[0]));
    err = direct_control(s_my_gpio_handle, CTL_GPIO_INIT_BY_PIN_LIST, &pin_led);

    /* On error */
    if (err < 0)
    {
        /* stop execute */
        while (1)
        {
            R_COMPILER_Nop();
        }
    }

    /* ==== Output banner message ==== */
    printf("%s%s", sp_clear_screen, sp_cursor_home);
    show_welcome_msg(stdout, true);

    /* Create a task to run the RT-Thread Test cases */
    R_OS_TaskCreate("RT-Test", (os_task_code_t) os_rt_test_task_t, NULL,
                        R_OS_ABSTRACTION_DEFAULT_STACK_SIZE,
                        TASK_CONSOLE_TASK_PRI);

    while (1)
    {
        /* ==== LED blink ==== */
        s_main_led_flg ^= 1;

        if (MAIN_PRV_LED_ON == s_main_led_flg)
        {
            direct_control(s_my_gpio_handle, CTL_GPIO_PIN_WRITE, &s_p60_hi);
        }
        else
        {
            direct_control(s_my_gpio_handle, CTL_GPIO_PIN_WRITE, &s_p60_lo);
        }

        R_OS_TaskSleep(500);
    }

    return (err);
}
/**********************************************************************************************************************
 * End of function os_main_task_t
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: main
 * Description  : C Entry point
 *              : opens and configures cpg driver
 *              : starts the freertos kernel
 * Arguments    : none
 * Return Value : 0
 *********************************************************************************************************************/
int_t main(void)
{
    int_t cpg_handle;

    /* configure any drivers that are required before the Kernel initialises */

    /* Initialize the devlink layer */
    R_DEVLINK_Init();

    /* Initialize CPG */
    cpg_handle = direct_open("cpg", 0);

    if (cpg_handle < 0)
    {
        /* stop execute */
        while (1)
        {
            R_COMPILER_Nop();
        }
    }

    /* Can close handle if no need to change clock after here */
    direct_close(cpg_handle);

    /* Start FreeRTOS */
    /* R_OS_InitKernel should never return */
    R_OS_KernelInit();

    return (0);
}
/**********************************************************************************************************************
 * End of function main
 *********************************************************************************************************************/

/* End of File */
