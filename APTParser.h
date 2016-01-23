#include <stdint.h>
/*   $Id: APTParser.h,v 1.3 1999/07/29 19:57:44 ags-sw Exp $  */

#ifndef __unix__
#pragma once
#endif

extern	void			InitAPTParser ( void );

extern	void			CloseAPTParser ( void );

extern	uint32_t ProcessPatternData(struct lg_master *pLgMaster, unsigned char *theData,	uint32_t lengthOfData );

