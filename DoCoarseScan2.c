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
#include "DoCoarseScan2.h"
#include "SensorSearch.h"
#include "Protocol.h"
#include "LaserInterface.h"

extern int32_t * gLsort;
extern int32_t * coarsedata;
int32_t * gScan;
extern int32_t * gScan;
uint32_t DCS2minlevel = 70;

#define DELLEV 30

static int c2intsort( const void *elem1, const void *elem2 );

int DoCoarseScan2(struct lg_master *pLgMaster,
		  int32_t dX, int32_t dY,
		  uint32_t lsstep, int lscount,
		  int32_t *xfound, int32_t *yfound)
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
      int index;
      int target_cnt;
      int halfStep;
      int notarget;
      int min_target = 6;
      int theResult;
      uint32_t testlevel;
      uint32_t xmid;
      uint32_t ymid;
      int Icount;
      int Ihalf;
      int32_t Ixavg;
      int32_t Iyavg;


      xmid = dX;
      ymid = dY;
      target_cnt = 0;

      memset((char *)&xydata, 0, sizeof(struct lg_xydata));
      memset((char *)&xydelta, 0, sizeof(struct lg_xydata));
      memset((char *)&Xarray[0], 0, sizeof(Xarray));
      memset((char *)&Yarray[0], 0, sizeof(Yarray));
      memset( (void *)&(coarsedata[0]), 0, 512*512*sizeof(uint32_t) );

      xoff = xmid - ((lscount * lsstep) / 2 );
      yoff = ymid - ((lscount * lsstep) / 2 );
   
      testlevel = DELLEV;
      for ( x = 0x1U; x <= lscount+1; x += 0x2U ) {
          notarget = 0;
          xydata.xdata = ((x * lsstep) + xoff) & kMaxSigned;
          xydata.ydata = (lsstep + yoff) & kMaxSigned;
          xydelta.xdata   = 0;
          xydelta.ydata = lsstep;
          nSteps = lscount;
          theResult = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
				    (struct lg_xydata *)&xydelta, nSteps, gScan );
          if ( theResult == kStopWasDone ) {
	    SearchBeamOff(pLgMaster);
                   return theResult;
          }
          for ( y = 0x1U; y <= lscount; y += 0x1U ) {
                 index = y * lscount + x;
                 if ( x > 1 ) {
                     coarsedata[index] = gScan[y-1];
                 }
                 gLsort[y-1] = gScan[y-1];
          }
          qsort( gLsort, lscount, sizeof(uint32_t), c2intsort);
          halfStep = lsstep / 2;
          testlevel = gLsort[halfStep];
          if ( firstCoarse == 1 ) {
               firstCoarse = 0;
               DCS2minlevel = testlevel + DELLEV;
          } 
          if ( x > 1 ) {
              for ( y = 0x1U; y <= lscount; y += 0x1U ) {
                   if ( gScan[y] >= DCS2minlevel ) {
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

          xydata.xdata = (((x + 1) * lsstep) + xoff) & kMaxSigned;
          xydata.ydata = ((lscount * lsstep) + yoff) & kMaxSigned;
          xydelta.xdata = 0;
          xydelta.ydata = -lsstep;
          nSteps = lscount;
          theResult = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
				    (struct lg_xydata *)&xydelta,
				    nSteps, gScan );
          if ( theResult == kStopWasDone ) {
	    SearchBeamOff(pLgMaster);
                   return theResult;
          }
          for ( y = lscount; y > 0 ; y -= 0x1U ) {
                 index = (lscount - y) * lscount + (x + 1);
                 coarsedata[index] = gScan[y];
          }
          for ( y = lscount; y > 0 ; y -= 0x1U ) {
                 if ( gScan[y] >= DCS2minlevel ) {
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
               if ( coarsedata[index] > DCS2minlevel ) {
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
      qsort( Xarray, Icount, sizeof(int32_t), c2intsort);
      qsort( Yarray, Icount, sizeof(int32_t), c2intsort);
      Ihalf = Icount / 2;
      Ixavg = Xarray[Ihalf];
      Iyavg = Yarray[Ihalf];
      *xfound = ((uint32_t)(Ixavg)) * lsstep + xoff;
      *yfound = ((uint32_t)(Iyavg)) * lsstep + yoff;

      return 0;
}


static int c2intsort(const void *elem1, const void *elem2)
{
    uint32_t numone, numtwo;

    numone = *(const uint32_t *)elem1;
    numtwo = *(const uint32_t *)elem2;

    if ( numone < numtwo ) return -1;
    if ( numone > numtwo ) return  1;

    return 0;
}

