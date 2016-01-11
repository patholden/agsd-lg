#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "DoSuperLevelScan.h"
#include "SensorSearch.h"
#include "Protocol.h"
#include "AppResponses.h"

uint32_t * scandata;
uint32_t * gScan;

void
DoSuperLevelScan( double dX, double dY )
{
      uint32_t xoff, yoff;
      uint32_t xout, yout;
      uint32_t delX, delY;
      uint32_t x, y;
      uint32_t nSteps;
      int index;

      uint32_t xmid;
      uint32_t ymid;
      char theResponseBuffer[sizeof ( uint32_t ) + kCRCSize];     

#ifdef ZZZDEBUG
fprintf( stderr, "entering DoLevelScan\n" );
fprintf( stderr, "DoLevelScan x,y double %10.6lf %10.6lf\n", dX, dY );
#endif

      *(uint32_t *)theResponseBuffer =
           ConvertExternalAnglesToBinary( dX, dY, &xmid, &ymid );

#ifdef ZZZDEBUG
fprintf( stderr, "DoLevelScan x,y mid %8x %8x\n", xmid, ymid );
#endif

      memset( (void *)&(scandata[0]), 0, 66*66*sizeof(uint32_t) );
      
      if ( *(uint32_t *)theResponseBuffer )
      {
                *(uint32_t *)theResponseBuffer +=
                        kFail + kInputAngleOutOfRange;
                HandleResponse ( (char *)theResponseBuffer,
                        sizeof ( uint32_t ), kRespondExtern );
                return;
      }


      xoff = xmid - ((LSCNTSTEP * LSSTEPSIZE) / 2 );
      yoff = ymid - ((LSCNTSTEP * LSSTEPSIZE) / 2 );
   
#ifdef ZZZDEBUG
fprintf( stderr, "DoLevelScan x,y off %8x %8x\n", xoff, yoff );
#endif

      for ( y = 0x1U; y <= LSCNTSTEP; y += 0x2U ) {
          xout   =     LSSTEPSIZE + xoff;
          yout   = y * LSSTEPSIZE + yoff;
          delX   = LSSTEPSIZE;
          delY   = 0;
          nSteps = LSCNTSTEP;
          if ( DoLevelSearch( xout, yout, delX, delY, nSteps, gScan ) ) {
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
          if ( DoLevelSearch( xout, yout, delX, delY, nSteps, gScan ) ) {
                   return;    // kStopWasDone should be the reason
          }
          for ( x = LSCNTSTEP; x > 0 ; x -= 0x1U ) {
                 index = (y + 1) * LSCNTSTEP + (LSCNTSTEP - x);
                 scandata[index] = gScan[x];
          }


      }

      SlowDownAndStop( );

#ifdef ZZZDEBUG
fprintf( stderr, "DoLevelScan about to HandleResponse\n" );
#endif
      *(uint32_t *)theResponseBuffer = kOK;
      HandleResponse ( (char *)theResponseBuffer,
                        sizeof ( uint32_t ), kRespondExtern );
      return;
}

void
InitLevelScan( void )
{
      scandata = (uint32_t *)calloc(66*66, sizeof(uint32_t));
      gScan = (uint32_t *)calloc(1024, sizeof(uint32_t));
      
}
