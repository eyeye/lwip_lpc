/******************************************************************************/
/* RETARGET.C: 'Retarget' layer for target-dependent low level functions      */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2006 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

#include <stdio.h>


int _read (int file, char *ptr, int len)
{
	/* Can be implemented to getc(), but not used */
	return 1;
}
	
int _write (int file, char *ptr, int len)
{
	int i;

	/* Send characters on the serial port */
	for (i = 0; i < len; i++)
	{
		sendchar((int) *ptr);
		ptr++;
	}

	return len;
}
