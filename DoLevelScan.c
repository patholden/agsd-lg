#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>

#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "DoLevelScan.h"
#include "SensorSearch.h"
#include "Protocol.h"
#include "AppResponses.h"
#include "LaserInterface.h"

uint32_t * scandata;
uint32_t * gScan;

void
DoLevelScan(struct lg_master *pLgMaster, double dX, double dY )
{
      uint32_t xoff, yoff;
      uint32_t xout, yout;
      uint32_t delX, delY;
      uint32_t x, y;
      uint32_t nSteps;
      int index;

      uint32_t xmid;
      uint32_t ymid;
      uint32_t return_val;
      struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
      
#ifdef ZZZDEBUG
fprintf( stderr, "entering DoLevelScan\n" );
fprintf( stderr, "DoLevelScan x,y double %10.6lf %10.6lf\n", dX, dY );
#endif

      return_val =
           ConvertExternalAnglesToBinary( dX, dY, &xmid, &ymid );

#ifdef ZZZDEBUG
fprintf( stderr, "DoLevelScan x,y mid %8x %8x\n", xmid, ymid );
#endif

      memset( (void *)&(scandata[0]), 0, 67*67*sizeof(uint32_t) );
      
      if (return_val)
      {
	pResp->hdr.status1 = RESPFAIL;
	pResp->hdr.errtype1 = RESPE1INANGLEOUTOFRANGE;
	HandleResponse(pLgMaster,
		       (sizeof(struct parse_basic_resp)-kCRCSize),
		       kRespondExtern);
	return;
      }


      xoff = xmid - ((LSCNTSTEP * LSSTEPSIZE) / 2 );
      yoff = ymid - ((LSCNTSTEP * LSSTEPSIZE) / 2 );
   
#ifdef ZZZDEBUG
fprintf( stderr, "DoLevelScan x,y off %8x %8x\n", xoff, yoff );
#endif

      for ( y = 0x1U; y <= LSCNTSTEP+1; y += 0x2U ) {
          xout   =     LSSTEPSIZE + xoff;
          yout   = y * LSSTEPSIZE + yoff;
          delX   = LSSTEPSIZE;
          delY   = 0;
          nSteps = LSCNTSTEP;
          if ( DoLevelSearch(pLgMaster, xout, yout, delX, delY, nSteps, gScan ) ) {
                   return;    // kStopWasDone should be the reason
          }
          for ( x = 0x1U; x <= LSCNTSTEP; x += 0x1U ) {
                 index = y * LSCNTSTEP + x;
                 scandata[index] = gScan[x-1];
          }
          xout   = LSCNTSTEP *  LSSTEPSIZE + xoff;
          yout   =   (y + 1) * LSSTEPSIZE + yoff;
          delX   = -LSSTEPSIZE;
          delY   = 0;
          nSteps = LSCNTSTEP;
          if ( DoLevelSearch(pLgMaster, xout, yout, delX, delY, nSteps, gScan ) ) {
                   return;    // kStopWasDone should be the reason
          }
          for ( x = LSCNTSTEP; x > 0 ; x -= 0x1U ) {
                 index = (y + 1) * LSCNTSTEP + (LSCNTSTEP - x);
                 scandata[index] = gScan[x];
          }


      }

      SlowDownAndStop(pLgMaster);
      pResp->hdr.status = RESPGOOD;
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), kRespondExtern);
      return;
}

void
InitLevelScan( void )
{
      scandata = (uint32_t *)calloc(67*67, sizeof(uint32_t));
      gScan = (uint32_t *)calloc(1024, sizeof(uint32_t));
      
}
