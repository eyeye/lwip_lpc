/**********************************************************************
* $Id$		lpc_phy.c			2011-11-20
*//**
* @file		lpc_phy.c
* @version	1.0
* @date		20 Nov. 2011
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

#include "lpc_emac_config.h"
#include "lpc_phy.h"

#ifdef LPC_PHY_DP83848
/* Build the DP83848 variant of the PHY support driver */
#include "lpc_phy_dp83848.c"
#endif

#ifdef LPC_PHY_LAN8720
/* Build the DP83848 variant of the PHY support driver */
#include "lpc_phy_lan8720.c"
#endif

/* --------------------------------- End Of File ------------------------------ */
