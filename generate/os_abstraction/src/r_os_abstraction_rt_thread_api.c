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
 * Copyright (C) 2019 Renesas Electronics Corporation. All rights reserved.
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * File Name    : r_os_abstraction_rt_thread_api.c
 * Version      : see OS_LESS_RZ_HLD_VERSION_MAJOR.OS_LESS_RZ_HLD_VERSION_MINOR
 * Description  : To make efficient code re-use the identical high layer driver shall be used in both OS and
 *                OS Less applications.
 *                This file aims to abstract the Operating system (OS) awareness when creating an OS Less driver.
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History      : DD.MM.YYYY Version Description
 *              : 03.09.2019 3.03    Added R_COMPILER_WEAK to vApplicationStackOverflowHook () allowing user override 
 *********************************************************************************************************************/

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include "r_devlink_wrapper.h"

#include "r_typedefs.h"
#include "driver.h"

#include "rtthread.h"
#include "rtconfig.h"
#include "rtservice.h"
#include "rtm.h"
#include "rtdef.h"
#include "rthw.h"
#include "rtdebug.h"
#include "rtdbg.h"
#include "interrupt.h"
#include "board.h"
#include "r_ostm_drv_api.h"

/* compiler specific API header */
#include "r_compiler_abstraction_api.h"

/* OS abstraction specific API header */
#include "r_os_abstraction_api.h"

#include "r_task_priority.h"
#include "application_cfg.h"

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/

/* A minimum of 8 characters or "Unknown" will not fit! */
#define R_OS_PRV_TINY_STACK_SIZE        (RT_MAIN_THREAD_STACK_SIZE/4)
#define R_OS_PRV_SMALL_STACK_SIZE       (RT_MAIN_THREAD_STACK_SIZE/2)
#define R_OS_PRV_DEFAULT_STACK_SIZE     (RT_MAIN_THREAD_STACK_SIZE)
#define R_OS_PRV_LARGE_STACK_SIZE       (RT_MAIN_THREAD_STACK_SIZE*2)
#define R_OS_PRV_HUGE_STACK_SIZE        (RT_MAIN_THREAD_STACK_SIZE*4)

/* If there are other thread which have same priority with Main thread,
 * main thread and other ready thread will switch back and forth.
 * main thread will run R_OS_MAIN_THREAD_TIMESLICE ticks before switching
 */
#define R_OS_MAIN_THREAD_TIMESLICE		(1)
#define R_OS_THREAD_TIMESLICE_DEFAULT	(5)

#define R_RUNTIME_STATS_OSTM_RESOURCE ("\\\\.\\runtime_stats_timer")

#define portMAX_DELAY					(RT_TICK_MAX)

/* Note ucHeap is used in freeRTOS for fixed location fixed sized memory pool */
uint8_t ucHeap[HEAP_SIZE] __attribute__ ((section(".rtthread_heap")));


/**********************************************************************************************************************
 Private global variables
 *********************************************************************************************************************/
static int_t s_runtime_stats_timer_handle = (-1);

#ifdef OS_DEBUG_RECORD_MEM_ALLOCS
static volatile char gs_debug_task_malloc_str[24];
static volatile char gs_debug_task_free_str[24];
#endif /* OS_DEBUG_RECORD_MEM_ALLOCS */

static const st_os_abstraction_info_t s_os_version =
{
    /*Version Number*/
    { ((R_OS_ABSTRACTION_VERSION_MAJOR << 16) + R_OS_ABSTRACTION_VERSION_MINOR) },

    /*Build Number*/
	RT_VER_NUM,

    /*Name of OS Abstraction Layer*/
    ("RT-Thread OS Abstraction")
};

static rt_thread_t tid_main = RT_NULL;
static const char s_startup_task_name_str[] = "Main";

#define RT_THREAD_INIT                  0x00                /**< Initialized status */
#define RT_THREAD_READY                 0x01                /**< Ready status */
#define RT_THREAD_SUSPEND               0x02                /**< Suspend status */
#define RT_THREAD_RUNNING               0x03                /**< Running status */
#define RT_THREAD_BLOCK                 RT_THREAD_SUSPEND   /**< Blocked status */
#define RT_THREAD_CLOSE                 0x04                /**< Closed status */

static const char* sp_status_name[6] =
{
		"Initialized",
		"Ready",
		"Suspend",
		"Running",
		"Closed",
		"Error"
};
/*xSemaphoreHandle is defined as a void pointer*/
static p_mutex_t s_pstream_mutex = NULL;

/*Initialising as Null*/
static void *sp_task_handle = NULL;
static char s_file[200];
static uint32_t s_line;

static size_t s_event_count = 0UL;
static size_t s_event_max = 0UL;

static bool_t s_os_running = false;

/* LOG_TASK_INFO provides logging of information into a circular buffer. This is currently
 * configured to do so in mallocs and frees.
 * To use: Add text to log using the log_data function. The log_write_data_to_console will
 * output the contents of the buffer to the console (currently set to do this in the
 * OS_assert function).
 */
#ifdef LOG_TASK_INFO
static volatile char s_log_data[LOG_DATA_DUMP_SIZE_PRV_ + 1];
static volatile char *sp_log_head = s_log_data;
static volatile char *sp_log_tail = s_log_data;
static volatile uint8_t s_log_readback_running = 0u;
#endif /* LOG_TASK_INFO */

void os_abstraction_isr (void);

extern rt_err_t rt_thread_sleep(rt_tick_t tick);
extern rt_list_t rt_thread_priority_table[RT_THREAD_PRIORITY_MAX];

#ifdef LOG_TASK_INFO
/**********************************************************************************************************************
 * Function Name: log_data
 * Description  : Logs text to a circular buffer for debug purposes
 * Arguments    : char *p_data_to_log
 *                uint32_t len_data
 * Return Value : void
 *********************************************************************************************************************/
static void log_data (char *p_data_to_log, uint32_t len_data)
{
    uint32_t len = 0;

    /* determine length of data to log */
    if (0 == len_data)
    {
        /* using strlen */
        len = strlen(p_data_to_log);
    }
    else
    {
        len = len_data;
    }

    /* will head fit in buffer without overlap */
    if ((sp_log_head + len) < (s_log_data + LOG_DATA_DUMP_SIZE_PRV_))
    {
        /* push tail up if overlapped */
        if ((sp_log_tail > sp_log_head) && (sp_log_tail < (sp_log_head + len)))
        {
            sp_log_tail = (sp_log_head + len);
        }
        strncpy(sp_log_head, p_data_to_log, len);
        sp_log_head += len;
    }
    else
    {
        /* overlap */

        /* add to top of buffer */

        /* calc number of bytes to top */
        uint32_t num_chars_to_top = ((s_log_data + LOG_DATA_DUMP_SIZE_PRV_) - sp_log_head);

        /* fill buffer to the top */
        strncpy(sp_log_head, p_data_to_log, num_chars_to_top);

        /* add remaining to start of buffer */
        strncpy(s_log_data, (char *) (p_data_to_log + num_chars_to_top), (len - num_chars_to_top));

        /* update head */
        sp_log_head = (char *) (s_log_data + (len - num_chars_to_top));

        /* update tail if overlapped */
        if (sp_log_tail < sp_log_head)
        {
            sp_log_tail = sp_log_head;
        }
    }
}
/**********************************************************************************************************************
 End of function log_data
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: log_write_data_to_console
 * Description  : Outputs contents of circular debug buffer to console
 * Arguments    : void
 * Return Value : void
 *********************************************************************************************************************/
static void log_write_data_to_console (void)
{
    /* check if console reporting is already underway */
    /* sometimes assert runs in more than 1 task... */
    if (s_log_readback_running)
    {
        return;
    }

    /* prevent multiple access in assert */
    s_log_readback_running = 1u;

    if (sp_log_tail < sp_log_head)
    {
        /* not overlapped. can just printf */
        *(sp_log_head + 1) = 0;

        /* cast to remove volatile qualifier */
        printf((char *) sp_log_tail);
    }
    else
    {
        /* overlapped. print up to top of buffer */
        /* not overlapped. can just printf */
        s_log_data[LOG_DATA_DUMP_SIZE_PRV_] = 0;

        /* cast to remove volatile qualifier */
        printf((char *) sp_log_tail);

        /* not overlapped. can just printf */
        *(sp_log_head + 1) = 0;

        /* cast to remove volatile qualifier */
        printf((char *) s_log_data);
    }

    /* empty buffer */
    sp_log_head = s_log_data;
    sp_log_tail = s_log_data;

    /* reset multiple access */
    s_log_readback_running = 0u;
}
/**********************************************************************************************************************
 End of function log_write_data_to_console
 *********************************************************************************************************************/

#endif /* LOG_TASK_INFO */

static void ostm_start( void )
{
    if (( -1) == s_runtime_stats_timer_handle)
    {
        s_runtime_stats_timer_handle = open(R_OS_ABSTRACTION_OSTM_RESOURCE, O_RDWR);

        if (( -1) == s_runtime_stats_timer_handle)
        {
            R_OS_AssertCalled( __FILE__, __LINE__);
        }
        else
        {
            control(s_runtime_stats_timer_handle, CTRL_OSTM_START_TIMER, NULL);
        }
    }
}

/**********************************************************************************************************************
 * Function Name: main_task
 * Description  : The main task, performs any ramining initialisation and call
 *                the user level function main().
 * Arguments    : none
 * Return Value : none
 *********************************************************************************************************************/
static void main_task (void *param)
{
	s_os_running = true;

    /* Call application main function which shouldn't return */
    extern int_t os_main_task_t (void);
    os_main_task_t();

    /* Guard here to protect the system if main returns */
    while (1)
    {
        /* Stops program from running off */
        ;
    }
}
/**********************************************************************************************************************
 End of function  main_task
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_AbstractionLayerInit
 * Description  : Function to configure critical resources for the connected
 *                OS or scheduler.
 * Arguments    : void
 * Return Value : true if there were no errors when initialising the Layer.
 *                false if there errors when initialising the Layer.
 *****************************************************************************/
bool_t R_OS_AbstractionLayerInit (void)
{
    bool_t ready = true;

    R_OS_KernelInit();

    return (ready);
}
/**********************************************************************************************************************
 End of function R_OS_AbstractionLayerInit
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_AbstractionLayerShutdown
 * Description  : Function to release critical resources for the connected
 *                OS or scheduler.
 * Arguments    : void
 * Return Value : true if there were no errors when closing the Layer.
 *                false if there errors when closing the Layer.
 *****************************************************************************/
bool_t R_OS_AbstractionLayerShutdown (void)
{
    bool_t ready = true;

    R_OS_KernelStop();

    return (ready);
}
/**********************************************************************************************************************
 End of function R_OS_AbstractionLayerShutdown
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *                                  Kernel API
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_KernelInit
 * Description  : Initialise the kernel
 * Arguments    : None
 * Return Value : None
 *********************************************************************************************************************/
void R_OS_KernelInit (void)
{
    uint8_t test_task_created = 0;


    R_OS_InitMemManager();

    /* initialzie hardware interrupt */
    rt_hw_interrupt_init();


    /* scheduler system initialization */
    rt_system_scheduler_init();

    /* timer system initialization */
    rt_system_timer_init();


#ifdef RT_USING_SIGNALS
    /* signal system initialization */
    rt_system_signal_init();
#endif

    R_DEVLINK_Init();

#if R_USE_ANSI_STDIO_MODE_CFG
    extern void initialise_monitor_handles _PARAMS ((void));

    initialise_monitor_handles();
#endif

    /* Create the main task */
    /* Creat thread 1, Name is thread1，Entry is thread1_entry */
    tid_main = rt_thread_create(s_startup_task_name_str,
    						main_task,
							RT_NULL,
							R_OS_PRV_HUGE_STACK_SIZE,
							R_OS_TASK_MAIN_TASK_PRI,
							R_OS_MAIN_THREAD_TIMESLICE);

    /* Start this thread if you get the thread control block */
    if (tid_main != RT_NULL)
        rt_thread_startup(tid_main);

    /* timer thread initialization */
     rt_system_timer_thread_init();

     /* idle thread initialization */
     rt_thread_idle_init();

     ostm_start();

     /* start scheduler */
     rt_system_scheduler_start();

     /* never reach here */
    while (1)
    {
        /* This point should never be reached */
        R_COMPILER_Nop();
    }
}
/**********************************************************************************************************************
 End of function R_OS_KernelInit
 *********************************************************************************************************************/


/**********************************************************************************************************************
 * Function Name: R_OS_Running
 *          Used to determine if the OS has started
 * @retval OS running true or false
 *********************************************************************************************************************/
bool_t R_OS_Running(void)
{
    return (s_os_running);
}
/**********************************************************************************************************************
 End of function R_OS_IsOSRunning
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_KernelStart
 * Description  : Start the kernel
 * Arguments    : None
 * Return Value : None
 *********************************************************************************************************************/
void R_OS_KernelStart (void)
{
	R_COMPILER_Nop();
}
/**********************************************************************************************************************
 End of function R_OS_KernelStart
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_KernelStop
 * Description  : Stop the kernel
 * Arguments    : None
 * Return Value : None
 *********************************************************************************************************************/
void R_OS_KernelStop (void)
{
	R_COMPILER_Nop();
}
/**********************************************************************************************************************
 End of function R_OS_KernelStop
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_InitMemManager
 * Description  : Initialise the Memory manager needs to be called before RAM usage
 * Arguments    : None
 * Return Value : None
 *********************************************************************************************************************/
void R_OS_InitMemManager (void)
{
	rt_system_heap_init(&ucHeap[0], HEAP_END);
}
/**********************************************************************************************************************
 End of function R_OS_InitMemManager
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *                                  Task API
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * Function Name: R_OS_TaskCreate
 * Description  : Create a task
 * Arguments    :  p_name - task name string
 *                 task_code - task main function
 *                 p_params - task associated parameters (task defined)
 *                 stack_size - initial stack size (maximum)
 *                 priority - initial priority
 * Return Value : os_task_t - pointer to newly created task, NULL if failed
 *********************************************************************************************************************/
os_task_t *R_OS_TaskCreate (const char_t *p_name, os_task_code_t task_code, void *p_params, size_t stack_size,
        int_t priority)
{
    rt_uint32_t actual_stack = 0;

    switch (stack_size)
    {
        case R_OS_ABSTRACTION_TINY_STACK_SIZE:
        {
            /* Stack size is defined in FreeRTOS as unsigned short, convert to 32 bit */
            actual_stack = (uint32_t) R_OS_PRV_TINY_STACK_SIZE;
            break;
        }
        case R_OS_ABSTRACTION_SMALL_STACK_SIZE:
        {
            /* Stack size is defined in FreeRTOS as unsigned short, convert to 32 bit */
            actual_stack = (uint32_t) R_OS_PRV_SMALL_STACK_SIZE;
            break;
        }
        case R_OS_ABSTRACTION_DEFAULT_STACK_SIZE:
        {
            /* Stack size is defined in FreeRTOS as unsigned short, convert to 32 bit */
            actual_stack = (uint32_t) R_OS_PRV_DEFAULT_STACK_SIZE;
            break;
        }
        case R_OS_ABSTRACTION_LARGE_STACK_SIZE:
        {
            /* Stack size is defined in FreeRTOS as unsigned short, convert to 32 bit */
            actual_stack = (uint32_t) R_OS_PRV_LARGE_STACK_SIZE;
            break;
        }
        case R_OS_ABSTRACTION_HUGE_STACK_SIZE:
        {
            /* Stack size is defined in FreeRTOS as unsigned short, convert to 32 bit */
            actual_stack = (uint32_t) R_OS_PRV_HUGE_STACK_SIZE;
            break;
        }
        default:
        {
            actual_stack = (rt_uint32_t )stack_size;
            break;
        }
    }

    /* Initialised variable (task) is checked against NULL to determine if the new task can be initialised
     * see prvInitialiseNewTask in file freertos\tasks.c */
    rt_thread_t  tid = RT_NULL;

    /* Create a new task */
    tid = rt_thread_create(p_name,
    		task_code, RT_NULL,
			actual_stack,
			(rt_uint8_t)priority,
			R_OS_THREAD_TIMESLICE_DEFAULT);

    /* Check whether the task was successfully created */
    if (tid != RT_NULL)
        rt_thread_startup(tid);

    return (os_task_t *)tid;
}
/**********************************************************************************************************************
 End of function R_OS_TaskCreate
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_TaskDelete
 * Description  : Delete task
 * Arguments    : pp_task - pointer to task handle, NULL deletes the current task
 * Return Value : none
 *********************************************************************************************************************/
void R_OS_TaskDelete (os_task_t **pp_task)
{
    /* Delete the specified task */
    if (NULL == pp_task || NULL == *pp_task)
    {

    }
    else
    {
        /* delete task */
    	rt_thread_delete((rt_thread_t ) *pp_task);

        /* NULL the pointer */
        *pp_task = NULL;
    }
}
/**********************************************************************************************************************
 End of function R_OS_TaskDelete
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_TaskSleep
 * Description  : Suspend the task for the specified period of time
 * Arguments    : sleep_ms - sleep period in milliseconds
 * Return Value : none
 *********************************************************************************************************************/
void R_OS_TaskSleep (uint32_t sleep_ms)
{
    /* Delay the task for the specified duration */
    rt_thread_sleep(R_OS_MS_TO_SYSTICKS(sleep_ms));
}
/**********************************************************************************************************************
 End of function R_OS_TaskSleep
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_TaskYield
 * Description  : Force a context switch
 * Arguments    : none
 * Return Value : none
 *********************************************************************************************************************/
void R_OS_TaskYield (void)
{
    /* Force a context switch */
	rt_thread_yield();
}
/**********************************************************************************************************************
 End of function R_OS_TaskYield
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_TaskSuspend
 * Description  : Suspend the specified task
 * Arguments    : p_task - pointer to task, NULL deletes the current task
 * Return Value : none
 *********************************************************************************************************************/
bool_t R_OS_TaskSuspend (os_task_t *p_task)
{
	rt_err_t ret;

    ret = rt_thread_suspend((rt_thread_t)p_task);

    if (ret == RT_EOK)
    {
    	return (true);
    }
    else
    {
    	return (false);
    }
}
/**********************************************************************************************************************
 End of function R_OS_TaskSuspend
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_TaskResume
 * Description  : Resume the task
 * Arguments    : p_task - pointer to task, NULL deletes the current task
 * Return Value : none
 *********************************************************************************************************************/
bool_t R_OS_TaskResume (os_task_t *p_task)
{
	rt_err_t ret;

    ret = rt_thread_resume((rt_thread_t)p_task);

    if (ret == RT_EOK)
    {
    	return (true);
    }
    else
    {
    	return (false);
    }
}
/**********************************************************************************************************************
 End of function R_OS_TaskResume
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_TasksSuspendAll
 * Description  : Suspend all tasks, only attempted if the operating system is running
 * Arguments    : none
 * Return Value : none
 *********************************************************************************************************************/
void R_OS_TasksSuspendAll (void)
{
	R_COMPILER_Nop();
}
/**********************************************************************************************************************
 End of function R_OS_TasksSuspendAll
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_TasksResumeAll
 * Description  : Resume all tasks, only attempted if the operating system is running
 * Arguments    : none
 * Return Value : none
 *********************************************************************************************************************/
void R_OS_TasksResumeAll (void)
{
	R_COMPILER_Nop();
}
/**********************************************************************************************************************
 End of function R_OS_TasksResumeAll
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_TasksGetNumber
 * Description  : Obtain total number of active tasks defined in the system, only attempted if the operating system is
 *                running
 * Arguments    : none
 * Return Value : none
 *********************************************************************************************************************/
uint32_t R_OS_TasksGetNumber (void)
{
	uint32_t i, ret;

	ret = 0;
	for (i=0; i<RT_THREAD_PRIORITY_MAX; i++)
	{
		ret += rt_slist_len((const rt_slist_t *)&(rt_thread_priority_table[i]));
	}

	return ret;
}
/**********************************************************************************************************************
 End of function R_OS_TasksGetNumber
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_TaskUsesFloatingPoint
 * Description  : Enables tasks to use floating point variables.
 * Arguments    : None
 * Return Value : None
 *********************************************************************************************************************/
void R_OS_TaskUsesFloatingPoint (void)
{
	R_COMPILER_Nop();
}
/**********************************************************************************************************************
 End of function R_OS_TaskUsesFloatingPoint
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Function Name: R_OS_TaskGetPriority
 Description:   Function to get the task priority
 Arguments:     IN task_id - The ID of the task to get the priority of
 Return value:  The priority of the task or -1U if not found
 *********************************************************************************************************************/
int32_t R_OS_TaskGetPriority (uint32_t task_id)
{
	rt_thread_t tid =  (rt_thread_t)task_id;

	return (int32_t)(tid->current_priority);
}
/**********************************************************************************************************************
 End of function  R_OS_TaskGetPriority
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Function Name: R_OS_TaskSetPriority
 Description:   Function to set the priority of a task
 Arguments:     IN  task_id - The ID of the task to get the priority for
 IN  uiPriority - The priority of the task
 Return value:  true if the priority was set
 *********************************************************************************************************************/
bool_t R_OS_TaskSetPriority (uint32_t task_id, uint32_t priority)
{
    bool_t ret_val = false;
    rt_err_t ret;

    ret = rt_thread_control((rt_thread_t)task_id, RT_THREAD_CTRL_CHANGE_PRIORITY, (rt_uint8_t *)&priority);

    /*Cast to task handle for FreeRTOS*/
    if (ret == RT_EOK)
    {
    	ret_val = true;
    }
    else
    {
        ret_val = false;
    }

    return (ret_val);
}
/**********************************************************************************************************************
 End of function  R_OS_TaskSetPriority
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_TaskGetCurrentHandle
 * Description  : Get the ID of a task
 * Arguments    : task - pointer to task
 * Return Value : Task id
 *********************************************************************************************************************/
os_task_t *R_OS_TaskGetCurrentHandle (void)
{
    if(false == s_os_running)
    {
    	return NULL;
    }

    return (os_task_t *)rt_thread_self();
}
/**********************************************************************************************************************
 End of function R_OS_TaskGetCurrentHandle
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_TaskGetCurrentName
 * Description  : return a pointer to the current task name
 * Arguments    : none
 * Return Value : char * pointer to task name
 *********************************************************************************************************************/
const char *R_OS_TaskGetCurrentName (void)
{
	rt_thread_t tid;

    if(false == s_os_running)
    {
    	return NULL;
    }

	tid = rt_thread_self();

    return (tid->name);
}
/**********************************************************************************************************************
 End of function R_OS_TaskGetCurrentName
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_TaskGetState
 * Description  : Get the state of a task
 * Arguments    : p_task - pointer to task
 * Return Value : String describing the status of the task
 *********************************************************************************************************************/
const char *R_OS_TaskGetState (const char *p_task)
{
	rt_thread_t tid = (rt_thread_t)p_task;
	register rt_base_t stat;

    if(false == s_os_running)
    {
    	return NULL;
    }

	stat = tid->stat & RT_THREAD_STAT_MASK;
	if (stat >= 5)
	{
		return sp_status_name[5];
	}
	return sp_status_name[stat];

}
/**********************************************************************************************************************
 End of function R_OS_TaskGetState
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *                                  System API
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_SysLock
 * Description  : Function to lock a critical section.
 * Arguments    : None.
 * Return Value : 1 - Critical section entered outside ISR
 *                0 - Critical section entered inside ISR
 *********************************************************************************************************************/
int_t R_OS_SysLock (void)
{
    int_t return_value = 0;

    rt_enter_critical();

    return (return_value);
}
/**********************************************************************************************************************
 End of function R_OS_SysLock
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_SysUnlock
 * Description  : Function to unlock system, or OS object.
 * Arguments    : None.
 * Return Value : None
 *********************************************************************************************************************/
void R_OS_SysUnlock (void)
{
	rt_exit_critical();
}
/**********************************************************************************************************************
 End of function R_OS_SysUnlock
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_SysWaitAccess
 * Description  : The OS Abstraction layer contains a system mutex. This function allows a user to obtain the
 *                mutex for system critical usage.
 *
 *               sp_task_handle - keeps record of which task owns the mutex, to allow for re-entrant access.
 *               s_pstream_mutex - system mutex
 *
 * Arguments    : none
 * Return Value : none
 *********************************************************************************************************************/
void R_OS_SysWaitAccess (void)
{
	rt_thread_t p_task_handle = rt_thread_self();

    if(false == s_os_running)
    {
    	return;
    }

    if (sp_task_handle == p_task_handle)
    {
        ;/* allow re-entrant access */
    }
    else
    {
        /* Wait for access to the list */
        if (NULL == s_pstream_mutex)
        {
            /*Get handle to mutex*/
            s_pstream_mutex = rt_mutex_create("sys", RT_IPC_FLAG_PRIO);
        }

        /* Wait mutex */
        rt_mutex_take((rt_mutex_t)s_pstream_mutex, (rt_int32_t)portMAX_DELAY);

        /* Assign the task ID for future reentrant access */
        sp_task_handle = p_task_handle;
    }
}
/**********************************************************************************************************************
 End of function R_OS_SysWaitAccess
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_SysReleaseAccess
 * Description  :  The OS Abstraction layer contains a system mutex, this function allows a user to release the
 *                 mutex from system critical usage.
 *
 *               sp_task_handle - keeps record of which task owns the mutex, to allow for re-entrant access.
 *               s_pstream_mutex - system mutex
 *
 * Arguments    : none
 * Return Value : none
 *********************************************************************************************************************/
void R_OS_SysReleaseAccess (void)
{
    if(false == s_os_running)
    {
    	return;
    }

    /*Release current task ownership of mutex*/
    sp_task_handle = NULL;

    /* Release mutex */
    rt_mutex_release((rt_mutex_t)s_pstream_mutex);
}
/**********************************************************************************************************************
 End of function R_OS_SysReleaseAccess
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_AssertCalled
 * Description  : Generic error Handler
 * Arguments    : file - Source code file where the error occurred.
 *                line - Source code file where the error occurred.
 * Return Value : none
 *********************************************************************************************************************/
void R_OS_AssertCalled (const char *p_file, uint32_t line)

{
	volatile uint32_t ul = 0;
	char local_string_buffer[32];

	memset(local_string_buffer, 0, 32);

	/* Variable assigned to a volatile static variable to avoid being optimised away */
	strncpy(s_file, p_file, 199);

	/* Variable assigned to a volatile static variable to avoid being optimised away */
	s_line = line;

	/* print to console */
	printf("\r\nR_OS_Assert. File :%s", s_file);
	printf("\r\n Line %d\r\n:(\r\n", ((uint16_t) s_line));

	strncpy(local_string_buffer, R_OS_TaskGetCurrentName(), 31);
	strcat(local_string_buffer, "\0");
	printf("Current task: %s\r\n", local_string_buffer);

	fflush(stdout);

#ifdef LOG_TASK_INFO
	printf("\r\nLog of Memory transactions:\r\n");
	log_write_data_to_console();
#endif /* LOG_TASK_INFO */

	/*Software Loop to ensure error detail printed to console*/
	while (0xFFFFFF >= ul)
	{
		uint32_t ll;
		for (ll = 0; ll < 0xFFFF; ll++)
		{
			;
		}
		ul++;
	}

	ul = 0;

	/*RT-Thread Error Handling Loop*/
	rt_enter_critical();

	/* Set ul to a non-zero value using the debugger to step out of this function */
	while (0 == ul)
	{
		;
	}

	rt_exit_critical();
}
/**********************************************************************************************************************
 End of function R_OS_AssertCalled
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *                                  Memory Management API
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_Malloc
 * Description  : Allocate a block of memory
 * Arguments    : size - request size
 *              : region - region of memory to allocate from
 * Return Value : ptr to memory block
 *********************************************************************************************************************/
void *R_OS_Malloc (size_t size, e_memory_region_t region)
{
    void *p_mem_to_alloc;

    p_mem_to_alloc = rt_malloc(size);

    /* Return a pointer to the newly allocated memory block */
    return (p_mem_to_alloc);
}
/**********************************************************************************************************************
 End of function R_OS_Malloc
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_Free
 * Description  : Free memory block
 * Arguments    : p_memory_to_free - ptr to memory block that is to be freed
 * Return Value : none
 *********************************************************************************************************************/
void R_OS_Free (void **pp_memory_to_free)
{
    /* Make sure the pointer, and the pointer that it points to are valid */
    if ((NULL != pp_memory_to_free) && (NULL != (*pp_memory_to_free)))
    {
        /* Free memory block */
    	rt_free(*pp_memory_to_free);

        /* clear pointer */
        *pp_memory_to_free = NULL;
    }
}
/**********************************************************************************************************************
 End of function R_OS_Free
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *                                  Semaphore API
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_SemaphoreCreate
 * Description  : Create a semaphore
 * Arguments    : p_semaphore - Pointer to a associated semaphore
 *              : count         - The maximum count for the semaphore object. This value must be greater than zero
 * Return Value : The function returns TRUE if the semaphore object was successfully created
 *                Otherwise, FALSE is returned
 *********************************************************************************************************************/
bool_t R_OS_SemaphoreCreate (p_semaphore_t p_semaphore, uint32_t count)
{
	p_semaphore = (p_semaphore_t)rt_sem_create("sem", (rt_uint32_t)count, RT_IPC_FLAG_PRIO);

	return (p_semaphore == RT_NULL) ? false : true;
}
/**********************************************************************************************************************
 End of function R_OS_SemaphoreCreate
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_SemaphoreDelete
 * Description  : Delete a semaphore, freeing any associated resources
 * Arguments    : semaphore_ptr - Pointer to a associated semaphore
 * Return Value : none
 *********************************************************************************************************************/
void R_OS_SemaphoreDelete (p_semaphore_t p_semaphore)
{
	rt_sem_delete((rt_sem_t)p_semaphore);
}
/**********************************************************************************************************************
 End of function R_OS_SemaphoreDelete
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_SemaphoreWait
 * Description  : Blocks operation until one of the following occurs:
 *              :        A timeout occurs
 *              :     The associated semaphore has been set
 * Arguments    : p_semaphore - Pointer to a associated semaphore
 *              : timeout       - Maximum time to wait for associated event to occur
 * Return Value : The function returns TRUE if the semaphore object was successfully set. Otherwise, FALSE is returned
 *********************************************************************************************************************/
bool_t R_OS_SemaphoreWait (p_semaphore_t p_semaphore, systime_t timeout)
{
	rt_err_t ret;

	ret = rt_sem_take((rt_sem_t)p_semaphore, (rt_int32_t)rt_tick_from_millisecond((rt_int32_t)timeout));

	return (ret == RT_EOK) ? true : false;
}
/**********************************************************************************************************************
 End of function R_OS_SemaphoreWait
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_SemaphoreRelease
 * Description  : Release a semaphore, freeing freeing it to be used by another task
 * Arguments    : semaphore_ptr - Pointer to a associated semaphore
 * Return Value : none
 *********************************************************************************************************************/
void R_OS_SemaphoreRelease (p_semaphore_t p_semaphore)
{
	rt_sem_release((rt_sem_t)p_semaphore);
}
/**********************************************************************************************************************
 End of function R_OS_SemaphoreRelease
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *                                  Mutex API
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_MutexCreate
 * Description  : Create a mutex
 * Arguments    : None
 * Return Value : mutex - ptr to object
 *********************************************************************************************************************/
void *R_OS_MutexCreate (void)
{
    /* cast SemaphoreHandle_t used by FreeRTOS to void * used by OS abstraction */
    return ((void *) rt_mutex_create("mutex", RT_IPC_FLAG_PRIO));
}
/**********************************************************************************************************************
 End of function R_OS_MutexCreate
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_MutexDelete
 * Description  : Delete mutex
 * Arguments    : pp_mutex - ptr to mutex object
 * Return Value : none
 *********************************************************************************************************************/
void R_OS_MutexDelete (pp_mutex_t pp_mutex)
{
    /* cast void * used by OS abstraction to SemaphoreHandle_t used by FreeRTOS */
	rt_mutex_delete ((rt_mutex_t ) *pp_mutex);

    /* Set NULL as Deleted */
    *pp_mutex = NULL;
}
/**********************************************************************************************************************
 End of function R_OS_MutexDelete
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_MutexAcquire
 * Description  : Acquire the mutex, waiting indefinitely if need be
 * Arguments    : p_mutex - pointer to object
 * Return Value : none
 *********************************************************************************************************************/
void R_OS_MutexAcquire (p_mutex_t p_mutex)
{
	if (rt_thread_self() == RT_NULL)
		return;
    /* Obtain ownership of the mutex object */
    /* cast void * used by OS abstraction to SemaphoreHandle_t used by FreeRTOS */
	rt_mutex_take((rt_mutex_t ) p_mutex, (rt_int32_t)portMAX_DELAY);
}
/**********************************************************************************************************************
 End of function R_OS_MutexAcquire
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_MutexRelease
 * Description  : This primitive function is specific for os_less applications
 * Arguments    : p_mutex - ptr to object
 * Return Value : none
 *********************************************************************************************************************/
void R_OS_MutexRelease (p_mutex_t p_mutex)
{
	if (rt_thread_self() == RT_NULL)
		return;
    /* Release ownership of the mutex object */
    /* cast void * used by OS abstraction to SemaphoreHandle_t used by FreeRTOS */
	rt_mutex_release((rt_mutex_t ) p_mutex);
}
/**********************************************************************************************************************
 End of function R_OS_MutexRelease
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_MutexWait
 * Description  : Creates a mutex if NULL object handle given.
 *                If a valid mutex object given, try acquire mutex for time of length timeout.
 *                If mutex acquired, return true, otherwise timeout is reached and returns false.
 * Arguments    : pp_mutex - object to lock
 *                time_out - wait time, maximum use R_OS_ABSTRACTION_EV_WAIT_INFINITE
 * Return Value : true if the mutex was acquired, false if not
 *********************************************************************************************************************/
bool_t R_OS_MutexWait (pp_mutex_t pp_mutex, uint32_t time_out)
{
    bool_t ret_val = false;

	if (rt_thread_self() == RT_NULL)
		return true;

    /* Check to see if it is pointing to NULL */
    if (NULL == ( *pp_mutex))
    {
        /* Create the mutex */
        *pp_mutex = (p_mutex_t )rt_mutex_create("mutex", RT_IPC_FLAG_PRIO);
    }

    /* Check that the mutex was created */
    if (NULL != ( *pp_mutex))
    {
        /* Try Get mutex */
        if (rt_mutex_take((p_mutex_t )( *pp_mutex), (rt_int32_t)time_out))
        {
            ret_val = true;
        }
    }

    return (ret_val);
}
/**********************************************************************************************************************
 End of function R_OS_MutexWait
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_EnterCritical
 * Description  : Enter critical section
 * Arguments    : None
 * Return Value : None
 *********************************************************************************************************************/
void R_OS_EnterCritical (void)
{
	rt_enter_critical();
}
/**********************************************************************************************************************
 End of function R_OS_EnterCritical
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_ExitCritical
 * Description  : Exit critical section
 * Arguments    : None
 * Return Value : None
 *********************************************************************************************************************/
void R_OS_ExitCritical (void)
{
	rt_exit_critical();
}
/**********************************************************************************************************************
 End of function R_OS_ExitCritical
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *                                  Message Queue API
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_MessageQueueCreate
 * Description  : Create an Message Queue.
 * Arguments    : queue_sz - Maximum number of elements in queue.
 *                pp_queue_handle - pointer to queue handle pointer.
 * Return Value : The function returns TRUE if the message queue was successfully created.
 *                Otherwise, FALSE is returned.
 *********************************************************************************************************************/
bool_t R_OS_MessageQueueCreate (p_os_msg_queue_handle_t *pp_queue_handle, uint32_t queue_sz)
{
    bool_t ret_value = false;

    p_os_msg_queue_handle_t *p_queue_temp;

    /* Create queue with size of os_msg_t struct which points to message */
    p_queue_temp = (p_os_msg_queue_handle_t *) rt_mb_create("mb", queue_sz, RT_IPC_FLAG_PRIO);

    /* test for valid response before confirming correct create */
    if (RT_NULL != p_queue_temp)
    {
        *pp_queue_handle = p_queue_temp;
        ret_value = true;
    }

    return (ret_value);
}
/**********************************************************************************************************************
 End of function R_OS_MessageQueueCreate
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_MessageQueuePut
 * Description  : Put a message onto a queue.
 * Arguments    : p_queue_handle - pointer to queue handle.
 *                p_message - pointer to message.
 * Return Value : The function returns TRUE if the event object was successfully added to the queue. Otherwise,
 *                FALSE is returned
 *********************************************************************************************************************/
bool_t R_OS_MessageQueuePut (p_os_msg_queue_handle_t p_queue_handle, p_os_msg_t p_message)
{
	rt_err_t ret;
    bool_t ret_value = false;

    /* return failed if queue handle pointer is NULL */
    if (NULL != p_queue_handle)
    {
    	ret = rt_mb_send((rt_mailbox_t)p_queue_handle, (rt_ubase_t)p_message);
    	if (ret == RT_EOK)
    		ret_value = true;
    }

    /* return status */
    return (ret_value);
}
/**********************************************************************************************************************
 End of function R_OS_MessageQueuePut
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_MessageQueueGet
 * Description  : Get a message from a queue.
 * Arguments    : p_queue_handle - pointer to queue handle
 *                pp_msg pointer - Pointer will point to NULL if no message and times out.
 *                timeout - in system ticks.
 *                blocking true = block thread/task until message received.False = not blocking
 * Return Value : The function returns TRUE if the event object was successfully retrieved from the queue.
 *                Otherwise, FALSE is returned
 *********************************************************************************************************************/
bool_t R_OS_MessageQueueGet (p_os_msg_queue_handle_t p_queue, p_os_msg_t *pp_msg, uint32_t timeout, bool_t blocking)
{
    /* set default timeout to be OS max */
	rt_tick_t ticks_to_wait;
    bool_t ret_value = false;

    /* ensure that pointers are not NULL before proceeding */
    if ((NULL != p_queue) && (NULL != pp_msg))
    {
        /* handle max timeout */
        ticks_to_wait = rt_tick_from_millisecond((rt_int32_t)timeout);

        if (false == blocking)
        {
            /* Receive check is not task-blocking */
            if (RT_EOK == rt_mb_recv((rt_mailbox_t) p_queue, (rt_ubase_t *) (pp_msg), (rt_int32_t)ticks_to_wait))
            {
                ret_value = true;
            }
            else
            {
                /* timed out - report failure */
                *pp_msg = NULL;
            }
        }
        else
        {
            /* Receive check is task-blocking */
            while (RT_EOK !=
            		rt_mb_recv((rt_mailbox_t) p_queue, (rt_ubase_t *) (pp_msg), (rt_int32_t)ticks_to_wait))
            {
            	rt_thread_yield();
            }

            ret_value = true;
        }
    }

    /* return status */
    return (ret_value);
}
/**********************************************************************************************************************
 End of function R_OS_MessageQueueGet
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_MessageQueueClear
 * Description  : Clear a message queue.
 * Arguments    : p_queue_handle - pointer to queue handle.
 * Return Value : The function returns TRUE if the event object was successfully cleared. Otherwise, FALSE is returned
 *********************************************************************************************************************/
bool_t R_OS_MessageQueueClear (p_os_msg_queue_handle_t p_queue_handle)
{
    /* ensure that queue handle is valid */
    if (NULL != p_queue_handle)
    {
    	if (RT_EOK == rt_mb_control((rt_mailbox_t) p_queue_handle, RT_IPC_CMD_RESET, RT_NULL))
            return (true);
    }

    /* NULL queue pointer as argument */
    return (false);
}
/**********************************************************************************************************************
 End of function R_OS_MessageQueueClear
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_MessageQueueDelete
 * Description  : Delete a message queue. The message queue pointer argument will be set to NULL.
 * Arguments    : pp_queue_handle - pointer to queue handle pointer.
 * Return Value : The function returns TRUE if the event object was successfully deleted. Otherwise, FALSE is returned
 *********************************************************************************************************************/
bool_t R_OS_MessageQueueDelete (p_os_msg_queue_handle_t *pp_queue_handle)
{
    /* ensure that queue handle is valid */
    if (NULL != pp_queue_handle)
    {
        /* delete queue structure */
    	rt_mb_delete((rt_mailbox_t) *pp_queue_handle);

        /* set queue pointer to NULL */
        *pp_queue_handle = NULL;

        /* freeRTOS always returns true */
        return (true);
    }

    /* queue pointer is NULL to start with so nothing deleted */
    return (false);
}
/**********************************************************************************************************************
 End of function R_OS_MessageQueueDelete
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *                                      Event API
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * Function Name: R_OS_EventCreate
 * Description  : Create an event
 * Arguments    : pp_event - pointer to a associated event
 * Return Value : The function returns TRUE if the event object was successfully created. Otherwise, FALSE is returned
 *********************************************************************************************************************/
bool_t R_OS_EventCreate (pp_event_t pp_event)
{
	rt_event_t p_temp_event = rt_event_create("event", RT_IPC_FLAG_PRIO);

    /*Only return event if created*/
    if (NULL != p_temp_event)
    {
        /* return the event */
        *pp_event = (pp_event_t)p_temp_event;
    }

    /*Return boolean true if event created*/
    return (NULL != p_temp_event);
}
/**********************************************************************************************************************
 End of function R_OS_EventCreate
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_EventDelete
 * Description  : Delete an event, freeing any associated resources
 * Arguments    : pp_event - pointer to a associated event
 * Return Value : None
 *********************************************************************************************************************/
void R_OS_EventDelete (pp_event_t pp_event)
{
    /* Make sure the handle is valid */
    if (NULL != pp_event)
    {
        /* Properly dispose the event object */
    	rt_event_delete((rt_event_t)( *pp_event));
    }

    /* The expectation is that the handle to the event should now be NULL
     * so that any future attempt to get the event will not be valid */
    *pp_event = NULL;
}
/**********************************************************************************************************************
 End of function R_OS_EventDelete
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_EventSet
 * Description  : Sets the state on the associated event
 * Arguments    : pp_event - pointer to a associated event
 * Return Value : None
 *********************************************************************************************************************/
void R_OS_EventSet (pp_event_t pp_event)
{


}
/**********************************************************************************************************************
 End of function R_OS_EventSet
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_EventReset
 * Description  : Clears the state on the associated event
 * Arguments    : pp_event - pointer to a associated event
 * Return Value : None
 *********************************************************************************************************************/
void R_OS_EventReset (pp_event_t pp_event)
{

}
/**********************************************************************************************************************
 End of function R_OS_EventReset
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Function Name: R_OS_EventGet
 Description:   Function to return the current state of an event
 Arguments:     IN pp_event - pointer to the event
 Return value:  The state of the event
 *********************************************************************************************************************/
e_event_state_t R_OS_EventGet (pp_event_t pp_event)
{
    e_event_state_t event_state = EV_RESET;

    return event_state;
}
/**********************************************************************************************************************
 End of function R_OS_EventGet
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_EventWait
 * Description  : Blocks operation until one of the following occurs
 *                A timeout occurs
 *                The associated event has been set
 * Arguments    : pp_event - pointer to a associated event
 *                timeout   - maximum time to wait for associated event to occur
 * Return Value : The function returns TRUE if the event object was successfully set. Otherwise, FALSE is returned
 *********************************************************************************************************************/
bool_t R_OS_EventWait (pp_event_t pp_event, systime_t timeout)
{

    return 0;
}
/**********************************************************************************************************************
 End of function R_OS_EventWait
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_EventSetFromIsr
 * Description  : Sets the state on the associated event
 *                Warning : Function shall only be caller from within an ISR routine
 * Arguments    : pp_event  - Pointer to a associated event
 * Return Value : The function returns TRUE if the event object was successfully set. Otherwise, FALSE is returned
 *********************************************************************************************************************/
bool_t R_OS_EventSetFromIsr (pp_event_t pp_event)
{

    return 0;
}
/**********************************************************************************************************************
 End of function R_OS_EventSetFromIsr
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_GetTickCount
 * Description  : Get the number of task ticks.
 * Arguments    : None
 * Return Value : Number of Ticks counted for a task
 *********************************************************************************************************************/
uint32_t R_OS_GetTickCount (void)
{
    return (rt_tick_get());
}
/**********************************************************************************************************************
 End of function R_OS_GetTickCount
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: R_OS_GetVersion
 * Description  :
 * Arguments    : pinfo - detailed driver information structure
 * Return Value : 0 can not fail
 *********************************************************************************************************************/
int32_t R_OS_GetVersion (st_os_abstraction_info_t *p_info)
{
    p_info->version.sub.major = s_os_version.version.sub.major;
    p_info->version.sub.minor = s_os_version.version.sub.minor;
    p_info->build = s_os_version.build;
    p_info->p_szdriver_name = s_os_version.p_szdriver_name;
    return (0);
}
/**********************************************************************************************************************
 End of function R_OS_GetVersion
 *********************************************************************************************************************/


/**********************************************************************************************************************
 * Function Name: os_abstraction_isr
 *          Used to perform timing operations (for function like R_OS_TaskSleep()
 * @retval test status pass true or false
 *********************************************************************************************************************/
void os_abstraction_isr (void)
{
    R_COMPILER_Nop();
}
/**********************************************************************************************************************
 End of function os_abstraction_isr
 *********************************************************************************************************************/

