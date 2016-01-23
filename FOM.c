/*
 *  static char rcsid[] = "$Id: FOM.c,v 1.2 1999/05/04 15:32:35 ags-sw Exp $";
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "FOM.h"
#include "Protocol.h"
#include "comm_loop.h"
#include "SensorRegistration.h"

void DosuperFOM (struct lg_master *pLgMaster, uint32_t respondToWhom )
{
  uint32_t    resp_len=(sizeof ( uint32_t ) 
			+ sizeof ( int32_t )
			+ sizeof ( double )
			+ (64 * (sizeof(int32_t) + 2*sizeof(double)))
			+ kCRCSize);
        int i;

	memset(pLgMaster->theResponseBuffer, 0, resp_len);
        *(uint32_t *)pLgMaster->theResponseBuffer = kOK;
        *(int32_t *)&(pLgMaster->theResponseBuffer[4]) = gNumberOfPoints;
        *(double *)&(pLgMaster->theResponseBuffer[8]) = gChisqr;

#ifdef ZDEBUG
fprintf( stderr, "superFOM  gNumberOfPoints %d\n", gNumberOfPoints );
fprintf( stderr, "superFOM  ggChisqr %lf\n", gChisqr );
#endif

        for ( i=0; i<gNumberOfPoints; i++ ) {
            if ( foundTarget[i] == 1 ) {
                *(int32_t *)&(pLgMaster->theResponseBuffer[16+(i)*20]) = i;
                *(double *)&(pLgMaster->theResponseBuffer[16+(i)*20+ 4]) = gDiffX[i];
                *(double *)&(pLgMaster->theResponseBuffer[16+(i)*20+12]) = gDiffY[i];
#ifdef ZDEBUG
fprintf( stderr, "superFOM  diffXY %d  %lf %lf\n", i, gDiffX[i], gDiffY[i] );
#endif
            } else {
                *(int32_t *)&(pLgMaster->theResponseBuffer[16+(i)*20]) = -1;
                *(double *)&(pLgMaster->theResponseBuffer[16+(i)*20+ 4]) = 0;
                *(double *)&(pLgMaster->theResponseBuffer[16+(i)*20+12]) = 0;
            }
        }
        HandleResponse (pLgMaster,
			(
			 sizeof ( uint32_t )
			 + sizeof ( int32_t )
			 + sizeof ( double )
			 + gNumberOfPoints * ( sizeof(int32_t) + 2*sizeof(double) )
			 ),
			respondToWhom );
       return;
}
