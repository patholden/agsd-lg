#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "DoCoarseScan.h"
#include "SensorSearch.h"
#include "Protocol.h"

extern uint32_t * coarsedata;

uint32_t * coarsedata;

uint32_t * gScan;
uint32_t DCSminlevel = 70;

#define DELLEV 30

extern
uint32_t * gLsort;

extern
uint32_t * gScan;

extern
int
cint32_tsort( const void *elem1, const void *elem2 );

int
DoCoarseScan(struct lg_master *pLgMaster,
	     uint32_t dX
            , uint32_t dY
            , uint32_t lsstep
            , int lscount
            , uint32_t *xfound
            , uint32_t *yfound
            )
{
      struct lg_xydata  xydata;
      struct lg_xydata  xydelta;
      uint32_t xoff, yoff;
      uint32_t x, y;
      uint32_t nSteps;
      int firstCoarse = 1;
      int index;
      int target_cnt;
      int halfStep;
      int notarget;
      int min_target = 6;
      int theResult;
      uint32_t testlevel;
      uint32_t xmid;
      uint32_t ymid;
      double Dxsum, Dysum;
      double Dcount;
      int Icount;
      int Ihalf;
      int32_t Xarray[65536];
      int32_t Yarray[65536];
      int32_t Ixavg;
      int32_t Iyavg;

      xmid = dX;
      ymid = dY;
      target_cnt = 0;

      memset((char *)&xydata, 0, sizeof(struct lg_xydata));
      memset((char *)&xydelta, 0, sizeof(struct lg_xydata));
      memset( (void *)&(coarsedata[0]), 0, 512*512*sizeof(uint32_t) );
      

      xoff = xmid - ((lscount * lsstep) / 2 );
      yoff = ymid - ((lscount * lsstep) / 2 );
   
      testlevel = DELLEV;
      for ( y = 0x1U; y <= lscount+1; y += 0x2U ) {
          notarget = 0;
          xydata.xdata = ((0 * lsstep) + xoff) & kMaxUnsigned;
          xydata.ydata = ((y * lsstep) + yoff) & kMaxUnsigned;
          xydelta.xdata = lsstep & kMaxUnsigned;
          xydelta.ydata = 0;
          nSteps = lscount;
	  
          theResult = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
				    (struct lg_xydata *)&xydelta, nSteps, gScan );
          if ( theResult == kStopWasDone ) {
                   SearchBeamOff(pLgMaster);
                   return theResult;
          }
          for ( x = 0x1U; x <= lscount; x += 0x1U ) {
                 index = y * lscount + x;
                 if ( y > 1 ) {
                     coarsedata[index] = gScan[x-1];
                 }
                 gLsort[x-1] = gScan[x-1];
          }
          qsort( gLsort, lscount, sizeof(uint32_t), cint32_tsort );
          halfStep = lsstep / 2;
          testlevel = gLsort[halfStep];
          if ( firstCoarse == 1 ) {
               firstCoarse = 0;
               DCSminlevel = testlevel + DELLEV;
          } 
          if ( y > 1 ) {
              for ( x = 0x1U; x <= lscount; x += 0x1U ) {
                   if ( gScan[x] >= DCSminlevel ) {
                       target_cnt++;
                       notarget = 0;
                       break;
                   } else {
                       notarget = 1;
                   }
              }
          }
          if ( notarget == 1 && target_cnt >= 1 ) break;
          if ( target_cnt >= min_target ) break;

          xydata.xdata = ((lscount * lsstep) + xoff) & kMaxUnsigned;
          xydata.ydata = (((y + 1) * lsstep) + yoff) & kMaxUnsigned;
          xydelta.xdata = -lsstep;
          xydelta.ydata = 0;
          nSteps = lscount;
          theResult = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
				    (struct lg_xydata *)&xydelta, nSteps, gScan);
          if ( theResult == kStopWasDone ) {
	    SearchBeamOff(pLgMaster);
	    return theResult;
          }
          for ( x = lscount; x > 0 ; x -= 0x1U ) {
                 index = (y + 1) * lscount + (lscount - x);
                 coarsedata[index] = gScan[x];
          }
          for ( x = lscount; x > 0 ; x -= 0x1U ) {
                 if ( gScan[x] >= DCSminlevel ) {
                       target_cnt++;
                       notarget = 0;
                       break;
                 } else {
                       notarget = 1;
                 }
          }
          if ( notarget == 1 && target_cnt >= 1 ) break;
          if ( target_cnt >= min_target ) break;


      }

      SlowDownAndStop(pLgMaster);
      if ( target_cnt == 0 ) {
          return kCoarseNotFound;
      }

      Dxsum = 0.0;
      Dysum = 0.0;
      Dcount = 0.0;
      Icount = 0;
      for ( y = 0x1U; y <= lscount; y += 0x1U ) {
          for ( x = 0x1U; x <= lscount; x += 0x1U ) {
               index = y * lscount + x;
               if ( coarsedata[index] > DCSminlevel ) {
                    Dxsum += (double)((int32_t)x);
                    Dysum += (double)((int32_t)y);
                    Dcount += 1.0;
                    Xarray[Icount] = (int32_t)x;
                    Yarray[Icount] = (int32_t)y;
                    Icount++;
               }
          }
      }
      if ( Dcount < 1.0 ) return kCoarseNotFound;
      qsort( Xarray, Icount, sizeof(int32_t), cint32_tsort );
      qsort( Yarray, Icount, sizeof(int32_t), cint32_tsort );
      Ihalf = Icount / 2;
      Ixavg = Xarray[Ihalf];
      Iyavg = Yarray[Ihalf];
      *xfound = ((uint32_t)(Ixavg)) * lsstep + xoff;
      *yfound = ((uint32_t)(Iyavg)) * lsstep + yoff;

      return 0;
}

void
InitCoarseScan( void )
{
      coarsedata = (uint32_t *)calloc(512*512, sizeof(uint32_t));
      
}

int
cint32_tsort( const void *elem1, const void *elem2 )
{
    uint32_t numone, numtwo;

    numone = *(const uint32_t *)elem1;
    numtwo = *(const uint32_t *)elem2;

    if ( numone < numtwo ) return -1;
    if ( numone > numtwo ) return  1;

    return 0;
}

