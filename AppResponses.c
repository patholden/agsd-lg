#include <stdint.h>
static char rcsid[] = "$Id: AppResponses.c,v 1.3 1997/06/14 21:03:16 ags-sw Exp $";

#include "AppCommon.h"
/* #include "AEServer.h" */
#include "AppResponses.h"
#include "AppStrListIDs.h"
/* #include "AppWindows.h" */
#include "AutoCertCmds.h"
#include "ExternalComm.h"
/* #include "LocalComm.h" */
/* #include "HAL_CELL_Handler.h" */
#include "LaserCmds.h"
#include <stdio.h>

static	uint32_t	gShownResponse = 0UL;
static	short			gResponseStrLength = 0;

enum
{
	 kResponseOK = 1,
	 kResponseStopOK,
	 kResponseFail,
	 kResponseQCFail,
	 kResponseStopFail,
	 kResponseUnknown,
	 kResponseQCOK,
	 kResponseQCPlyFail,
	 kResponseItsGone,
	 kResponseItsGoneNOT
};

/*
** static	void	ShowResponse ( uint32_t theResponse );
** 
** void ShowResponse ( uint32_t theResponse )
** {
** }
*/

void HandleResponse ( char * theResponseBuffer,
	int32_t lengthOfResponse, uint32_t gRespondToWhom )
{
	uint32_t temp;

	switch ( gRespondToWhom )
	{
		case kRespondExtern:
			temp = *(uint32_t *)theResponseBuffer;
			LongConv ( theResponseBuffer );
#ifdef ZDEBUG
fprintf( stderr, "HandleR about to SendResponse\n" );
#endif
			SendResponse ( theResponseBuffer, lengthOfResponse );
#ifdef ZDEBUG
fprintf( stderr, "HandleR finished to SendResponse\n" );
#endif
			  /* ShowResponse ( temp ); */
			break;
/*  there is NO Local, AutoCert, or HAL_CELL mode */
		case kDoNotRespond:
			break;
	}
}

/*
** void AppendToResponseDisplay ( short length, char * text )
** {
** 	
** }
*/
