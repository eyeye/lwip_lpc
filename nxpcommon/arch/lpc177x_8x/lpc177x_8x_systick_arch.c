/**********************************************************************
* $Id$		lpc177x_8x_systick_arch.c			2011-11-20
*//**
* @file		lpc177x_8x_systick_arch.c
* @brief	Setups up the system tick to generate a reference timebase
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

#include "lwip/opt.h"

#if NO_SYS == 1

#include "lpc177x_8x_systick.h"
#include "lpc_arch.h"

/** @defgroup LPC177x_8x_TimerSysTick	LPC177x_8x LWIP timer base
 * @ingroup LPC177x_8x
 * @{
 */

/* Saved reference period */
 static uint32_t saved_period;
 
/* Saved total time in mS since timer was enabled */
static volatile u32_t systick_timems;

/** \brief  Enable the system tick timer with a reference period.

    This function sets the Systick at a specific rate.

    \param[in]     period      Rate in mS for the system tick
 */
void SysTick_Enable(uint32_t period)
{
	/* Initialize System Tick with time interval */
	SYSTICK_InternalInit(period);
	saved_period = period;
	systick_timems = 0;

	/* Enable System Tick interrupt */
	SYSTICK_IntCmd(ENABLE);

	/* Enable System Tick Counter */
	SYSTICK_Cmd(ENABLE);
}

/** \brief  Disable the system tick timer.

    This function disables the Systick timer.
 */
void SysTick_Disable(void)
{
	/* Disable System Tick Counter */
	SYSTICK_Cmd(DISABLE);

	/* Disable System Tick interrupt */
	SYSTICK_IntCmd(DISABLE);
}

/** \brief  SysTick IRQ handler and timebase management

    This function keeps a timebase for the sysTick that can be
	used for other functions. It also calls an external function
	(SysTick_User) that must be defined outside this handler.
 */
void SysTick_Handler(void)
{
	/* Clear System Tick counter flag */
	SYSTICK_ClearCounterFlag();

	/* Increment tick count */
	systick_timems += saved_period;

	/* Call user function */
	SysTick_User(systick_timems);
}

/** \brief  Returns the number of mS since the timer was started

    This function will return the number of milliSeconds since the
	System tick timer was started. It overflows at about 49 days.

	\return    Number of milliSeconds since timer was started
 */
uint32_t SysTick_GetMS(void)
{
	return systick_timems;
}

/** \brief  Delay the specified number of milliSeconds.

    This function will delay the passed number of milliSeconds.

	\param[in]    ms   Number of milliSeconds to delay
 */
void msDelay(uint32_t ms)
{
	uint32_t to = ms + systick_timems;

	while (to > systick_timems);
}
#endif

/**
 * @}
 */

 /* --------------------------------- End Of File ------------------------------ */
