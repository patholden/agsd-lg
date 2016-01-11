#include <stdint.h>
/*   $Id: AppResponses.h,v 1.3 1997/05/11 23:21:13 ags-sw Exp $  */

#ifndef __unix__
#pragma once
#endif

#define	kRespondExtern		0U
#define	kRespondLocal		1U
#define	kRespondAutoCert	2U
#define	kRespond_HAL_CELL	3U
#define kRespondAE		4U
#define	kDoNotRespond		5U

void	AppendToResponseDisplay ( short length, char * text );
