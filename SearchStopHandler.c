#include <stdint.h>
/*
static char rcsid[] = "$Id: SearchStopHandler.c,v 1.1 1997/01/10 14:34:03 ags-sw Exp $";
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include "BoardComm.h"
#include "comm_loop.h"

extern int IfStopThenStopAndNeg1Else0 (struct lg_master *pLgMaster);
extern void ClearStopFlag ( void );
extern int	CheckStopFlag ( void );

int IfStopThenStopAndNeg1Else0 (struct lg_master *pLgMaster)
{
        if ( CheckInput(pLgMaster) ) {
#if ZDEBUG
fprintf( stderr, "stop requested\n" );
#endif
             return -1;
        } else {
             return 0;
        }
}

/*
	Somewhere in BoardComm.c declare
	
	static int gStopFlag;
	extern void ClearStopFlag ( void ) { gStopFlag = 0; }
	extern int CheckStopFlag ( void ) { return gStopFlag; }
	
	When you see PostCommand ( kStop, whatever, whatever )
	in addition to what you do now
	you should set gStopFlag = -1;
	
	Any variations on this scheme are OK as int32_t as I get either 0 or -1
*/
