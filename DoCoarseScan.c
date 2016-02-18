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
#include "LaserInterface.h"

extern int32_t * gLsort;
int32_t * coarsedata;
extern int32_t * coarsedata;
int32_t * gScan;
extern int32_t * gScan;
uint32_t DCSminlevel = 70;
#define DELLEV 30

static int cintsort( const void *elem1, const void *elem2 );

int
DoCoarseScan(struct lg_master *pLgMaster, int32_t dX, int32_t dY,
            uint32_t lsstep, int lscount, int32_t *xfound, int32_t *yfound)
{
      struct lg_xydata  xydata;
      struct lg_xydata  xydelta;
      int32_t Xarray[65536];
      int32_t Yarray[65536];
      double Dxsum, Dysum;
      double Dcount;
      int32_t xoff, yoff;
      int32_t x, y;
      uint32_t nSteps;
      int firstCoarse = 1;
      uint32_t index;
      uint32_t target_cnt;
      uint32_t min_target = 6;
      uint32_t halfStep;
      uint32_t testlevel;
      int notarget;
      int theResult;
      int32_t xmid;
      int32_t ymid;
      int Icount;
      int Ihalf;
      int32_t Ixavg;
      int32_t Iyavg;

      xmid = dX;
      ymid = dY;
      target_cnt = 0;

      memset((char *)&xydata, 0, sizeof(struct lg_xydata));
      memset((char *)&xydelta, 0, sizeof(struct lg_xydata));
      memset( (void *)&(coarsedata[0]), 0, 512*512*sizeof(int32_t) );
      memset((char *)&Xarray, 0, sizeof(Xarray));
      memset((char *)&Yarray, 0, sizeof(Yarray));

      xoff = xmid - ((lscount * lsstep) / 2 );
      yoff = ymid - ((lscount * lsstep) / 2 );
   
      testlevel = DELLEV;
      for ( y = 0x1U; y <= lscount+1; y += 0x2U ) {
          notarget = 0;
          xydata.xdata = ((0 * lsstep) + xoff) & kMaxSigned;
          xydata.ydata = ((y * lsstep) + yoff) & kMaxSigned;
          xydelta.xdata = lsstep & kMaxSigned;
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
          qsort( gLsort, lscount, sizeof(int32_t), cintsort);
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

          xydata.xdata = ((lscount * lsstep) + xoff) & kMaxSigned;
          xydata.ydata = (((y + 1) * lsstep) + yoff) & kMaxSigned;
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
      qsort( Xarray, Icount, sizeof(int32_t), cintsort);
      qsort( Yarray, Icount, sizeof(int32_t), cintsort);
      Ihalf = Icount / 2;
      Ixavg = Xarray[Ihalf];
      Iyavg = Yarray[Ihalf];
      *xfound = ((int32_t)Ixavg * lsstep) + xoff;
      *yfound = ((int32_t)Iyavg * lsstep) + yoff;

      return 0;
}

void
InitCoarseScan( void )
{
      coarsedata = (int32_t *)calloc(512*512, sizeof(int32_t));
      
}

static int cintsort( const void *elem1, const void *elem2 )
{
    int32_t numone, numtwo;

    numone = *(const int32_t *)elem1;
    numtwo = *(const int32_t *)elem2;

    if ( numone < numtwo ) return -1;
    if ( numone > numtwo ) return  1;

    return 0;
}

