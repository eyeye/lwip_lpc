/**********************************************************************
* $Id$		lpc_arch.h			2011-11-20
*//**
* @file		lpc_arch.h
* @brief	Architecture specific functions used with the LWIP examples
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

#ifndef __LPC_ARCH_H
#define __LPC_ARCH_H

#include "lwip/opt.h"

#ifdef __cplusplus
extern "C"
{
#endif

#if NO_SYS == 1
/* These are used in standalone mode only */
void SysTick_Enable(uint32_t period);
void SysTick_Disable(void);
uint32_t SysTick_GetMS(void);
void SysTick_User(u32_t ms);
#endif

void msDelay(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __LPC_ARCH_H */

/* --------------------------------- End Of File ------------------------------ */
