#include <stdint.h>
/*   $Id: InfoManager.h,v 1.3 1997/05/11 23:19:56 ags-sw Exp $  */

#ifndef __unix__
#pragma once
#endif

extern	void	InformCommand ( uint32_t theCommand );
extern	void	InformParameters
					( uint32_t theCommand, char * theParamBuffer );
extern	void	InformSerialNumber ( uint32_t theNumber );
