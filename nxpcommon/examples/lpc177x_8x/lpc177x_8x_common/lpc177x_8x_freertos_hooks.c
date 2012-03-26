/**********************************************************************
* $Id$		lpc177x_8x_freertos_hooks.c			2011-11-20
*//**
* @file		lpc177x_8x_freertos_hooks.c
* @brief	FreeRTOS support hooks for the LPC177x_8x
* @version	1.0
* @date		20. Nov. 2011
* @author	NXP MCU SW Application Team
* 
* Copyright(C) 2011, NXP Semiconductor
* All rights reserved.
*
***********************************************************************
* Software that is described herein is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
* NXP Semiconductors assumes no responsibility or liability for the
* use of the software, conveys no license or title under any patent,
* copyright, or mask work right to the product. NXP Semiconductors
* reserves the right to make changes in the software without
* notification. NXP Semiconductors also make no representation or
* warranty that such application will be suitable for the specified
* use without further testing or modification.
**********************************************************************/

#include "FreeRTOS.h"
#include "task.h"

#include "lwip/opt.h"
#include "lwip/mem.h"
#include "lwip/sys.h"

#include "lpc177x_8x.h"
#include "lpc_arch.h"

#if 0 // FIXME - cannot redirect to lwip mem
// Via LWIP memory manager
void vPortFree(void *mem)
{
	mem_free(mem);
}

// Via LWIP memory manager
void *pvPortMalloc(size_t bytes)
{
	return mem_malloc((mem_size_t) bytes);
}
#endif

void msDelay(uint32_t ms)
{
	portTickType xDelayTime;

	xDelayTime = xTaskGetTickCount();
	vTaskDelayUntil( &xDelayTime, ms );
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	LWIP_DEBUGF(LWIP_DBG_ON, ("MALLOC failed\n"));
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* Best to sleep here until next systick */
	__WFI;
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	LWIP_DEBUGF(LWIP_DBG_ON, ("App stack overflow\n"));
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	;
}
/*-----------------------------------------------------------*/

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
