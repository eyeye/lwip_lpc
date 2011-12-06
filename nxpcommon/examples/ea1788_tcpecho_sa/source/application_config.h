/**********************************************************************
* $Id$		application_config.h			2011-11-20
*//**
* @file		application_config.h
* @brief	Application specific configuration
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

#ifndef __APPLICATION_CONFIG_H__
#define __APPLICATION_CONFIG_H__

/** @ingroup tcpecho_sa_app
 * @{
 */

/** \brief  This application uses UART0 for output with printf and
            LWIP debug statements.
 */
#define UART_REDIRECT -1 /**< Set to UART number for UART re-direction, or -1 for NULL output */

/**
 * @}
 */

#endif /* __APPLICATION_CONFIG_H__ */

/* --------------------------------- End Of File ------------------------------ */
