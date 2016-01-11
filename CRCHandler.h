#include <stdint.h>
/*   $Id: CRCHandler.h,v 1.2 1996/12/25 18:38:04 ags-sw Exp $  */

#ifndef __unix__
#pragma once
#endif

#define	kCRC_OK		0
#define	kCRC_Bad	1

extern	void	AppendCRC	( char *theBuffer,
								int32_t lengthOfBufferWithoutCRC );
					
extern	short	CheckCRC	( char *theBuffer,
								int32_t lengthOfBufferWithoutCRC );


/* These two functions are platform dependent */
extern	void	InitCRCHandler		( void );

extern	void	CloseCRCHandler		( void );
