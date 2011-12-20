/**********************************************************************
* $Id$		lpc_lwip_doxygen.h			2011-11-20
*//**
* @file		lpc_lwip_doxygen.h
* @brief	Doxygen structure file
* @version	1.0
* @date		20 Nov. 2011
* @author	NXP MCU SW Application Team
*
* @note     This file is not used in the code. It only provides a
*           high level group architecture for Doxygen that is used a
*           a basis for all other in-code documentation.
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

#ifndef __LPC_LWIP_DOXYGEN_H
#define __LPC_LWIP_DOXYGEN_H

/* Documentation grouping structure
 * lwip_lpc
 *  lwip_emac
 *  lwip_phy
 *  lwip_applications
 *  lpc_board
 *   EA1788
 *  lpc_arch
 *   LPC177x_8x
 */

 /** @mainpage lwip_lpc	NXP LPC LWIP library port
 *
 * Welcome to the NXP LPC LWIP port for NXP devices. This NXP supported
 * port of LWIP provides support for various NXP devices with built-in
 * MAC controllers. The port is maintained by NXP and will be updated
 * and improved over time. The port is based on LWIP v1.4.0.
 * @{
 */
 
/** @defgroup lwip_lpc	NXP LPC LWIP library port.
 *
 * This package provides a LWIP port to NXP LPC devices. The port
 * supports both standalone and RTOS based applications as well as
 * link monitoring, zero-copy buffers, and more.
 * @{
 */

/**		  
 * @}
 */

/** @defgroup lwip_emac	LPC EMAC driver.
 * @ingroup lwip_lpc
 *
 * The NXP LPC EMAC driver supoprts the ethernet MAC controller in
 * some of NXP's devices. This same controller is used in the
 * LPC2000, LPC32xx, and LPC177x_8x devices. The driver supports
 * standalone and RTOS based applications and both copied and
 * zero-copy buffers for both receive and transmit. The driver
 * depends on an external PHY driver that handles link status via
 * the MII link interface.
 * @{
 */

/**		  
 * @}
 */

/** @defgroup lwip_phy	LPC PHY driver.
 * @ingroup lwip_lpc
 *
 * The PHY driver is needed by the EMAC driver and handles the basic
 * PHY management functions such as link status, speed, and duplex.
 * @{
 */

/**		  
 * @}
 */

/** @defgroup lwip_applications	LWIP applications
 * @ingroup lwip_lpc
 *
 * LWIP applications consist of echo servers, HTTP servers, etc. and
 * can usually run in standalone mode or with an RTOS. Applications
 * may support multiple toolchains such as Keil, IAR, or XPresso.
 * @{
 */

/**		  
 * @}
 */

/** @defgroup lpc_board  NXP LPC board specific port files for LWIP
 * @ingroup lwip_lpc
 *
 * Some functions for the NXP LPC port are board specific. Each board
 * has a specific set of functions that is used to setup or control
 * a capability for that board. The functions may setup up board muxing
 * or select a GPIO used to toggle an LED. Common functions are used in
 * the lwip applications to setup board specific capabilities.
 * @{
 */

/** @defgroup EA1788	Embedded Artists LPC1788 (EA1788) board
 * @ingroup lpc_board
 * @{
 */

/**		  
 * @}
 */

/**		  
 * @}
 */

/** @defgroup lpc_arch  NXP LPC architecture specific port files for LWIP
 * @ingroup lwip_lpc
 *
 * Some functions for the NXP LPC port are architecture specific. For
 * example, the timebase on one architecture may be different than
 * another. Common functions are used in the lwip applications to setup
 * architecture specific capabilities.
 * @{
 */

/** @defgroup LPC177x_8x	NXP LPC177x_8x devices
 * @ingroup lpc_arch
 * @{
 */

/**		  
 * @}
 */

/**		  
 * @}
 */

#endif /* __LPC_LWIP_DOXYGEN_H */

/* --------------------------------- End Of File ------------------------------ */
