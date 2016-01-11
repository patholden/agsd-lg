#include <stdint.h>
/*   $Id: AutoCertCmds.h,v 1.3 1997/05/11 23:20:53 ags-sw Exp $  */

#ifndef __unix__
#pragma once
#endif

#define kNoResult				0x00000000UL

extern	uint32_t	CheckSearch ( void );
extern	uint32_t	CheckStop ( void );
extern	uint32_t	PostSearch ( void );
extern	uint32_t	PostStop ( void );
extern	void			InitAutoCertCmds ( void );
extern	void			AutoCertResponse ( char * theResponse );
