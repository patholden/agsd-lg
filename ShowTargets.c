/*
static char rcsid[] = "$Id$";
*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stddef.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "SensorSearch.h"
#include "Protocol.h"
#include "ShowTargets.h"
#include "LaserInterface.h"

#define kMaxNumberX                                        0x7FFFC000U
#define kMinNumberX                                        0x80000000U
#define kMaxNumberY                                        0x7FFFC000U
#define kMinNumberY                                        0x80000000U

#define kMaxUnsigned  0xFFFFC000U



#define NPOINTS 2048

   /*
    *  size based on default values of gNumberOfSpiral
    *  and gCoarseSearchStep
    */
static uint32_t gNumberStep =  64 * 0x00080000U;

static int32_t gCoarse3SearchStep;

static void DoCross(uint32_t currentX, uint32_t currentY,
		    char *tmpPtr, uint32_t *pIndex);
static void DoSquare(uint32_t currentX, uint32_t currentY,
		     char *tmpPtr, uint32_t *pIndex);
static void draw_line(uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1,
		      char *tmpPtr, uint32_t *pIndex);
static void draw_dark(uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1,
		      char *tmpPtr, uint32_t *pIndex);
static void off_pause(uint32_t x1, uint32_t y1, char *tmpPtr, uint32_t *pIndex);
static void Draw0(uint32_t xmin, uint32_t xmax, uint32_t ymin,
		  uint32_t ymax, char *tmpPtr, uint32_t *pIndex);
static void Draw1(uint32_t x0, uint32_t ymin, uint32_t ymax,
		  char *tmpPtr, uint32_t *pIndex);
static void Do1(uint32_t currentX, uint32_t currentY, char *tmpPtr, uint32_t * pIndex);
static void Do2(uint32_t currentX, uint32_t currentY, char *tmpPtr, uint32_t * pIndex);
static void Do3( uint32_t currentX, uint32_t currentY,
                char * tmpPtr, uint32_t *pIndex);
static void Do4( uint32_t currentX, uint32_t currentY,
                char * tmpPtr, uint32_t *pIndex);
static void Do5( uint32_t currentX, uint32_t currentY,
                char * tmpPtr, uint32_t *pIndex);

static void Do6( uint32_t currentX, uint32_t currentY,
                char * tmpPtr, uint32_t *pIndex);

static void Do7( uint32_t currentX, uint32_t currentY,
                char * tmpPtr, uint32_t *pIndex);

static void Do8( uint32_t currentX, uint32_t currentY,
                char * tmpPtr, uint32_t *pIndex);

static void Do9( uint32_t currentX, uint32_t currentY,
                char * tmpPtr, uint32_t *pIndex);

static void Do10( uint32_t currentX, uint32_t currentY,
                  char * tmpPtr, uint32_t *pIndex);

static void Do11( uint32_t currentX, uint32_t currentY,
                  char * tmpPtr, uint32_t *pIndex);

static void Do12( uint32_t currentX, uint32_t currentY,
                  char * tmpPtr, uint32_t *pIndex);

static void Do13( uint32_t currentX, uint32_t currentY,
                  char * tmpPtr, uint32_t *pIndex);

static void Do14( uint32_t currentX, uint32_t currentY,
                  char * tmpPtr, uint32_t *pIndex);

static void Do15( uint32_t currentX, uint32_t currentY,
                  char * tmpPtr, uint32_t *pIndex);

static void Do16( uint32_t currentX, uint32_t currentY,
                  char * tmpPtr, uint32_t *pIndex);

static void Do17( uint32_t currentX, uint32_t currentY,
                  char * tmpPtr, uint32_t *pIndex);

static void Do18( uint32_t currentX, uint32_t currentY,
                  char * tmpPtr, uint32_t *pIndex);

static void Do19( uint32_t currentX, uint32_t currentY,
                  char * tmpPtr, uint32_t *pIndex);

static void Do20( uint32_t currentX, uint32_t currentY,
                  char * tmpPtr, uint32_t *pIndex);

static void Do21( uint32_t currentX, uint32_t currentY,
                  char * tmpPtr, uint32_t *pIndex);

static void Do22( uint32_t currentX, uint32_t currentY,
                  char * tmpPtr, uint32_t *pIndex);

static void Do23( uint32_t currentX, uint32_t currentY,
                  char * tmpPtr, uint32_t *pIndex);
static void Do24( uint32_t currentX, uint32_t currentY,
                  char * tmpPtr, uint32_t *pIndex);
static void Draw2(uint32_t xmin, uint32_t xmax, uint32_t ymin,
		  uint32_t y0, uint32_t ymax, char *tmpPtr,
		  uint32_t *pIndex);
static void Draw3(uint32_t xmin, uint32_t xmax, uint32_t ymin,
		  uint32_t y0, uint32_t ymax, char *tmpPtr,
		  uint32_t *pIndex);
static void Draw4(uint32_t xmin, uint32_t xmax, uint32_t ymin,
		  uint32_t y0, uint32_t ymax, char *tmpPtr,
		  uint32_t *pIndex);
static void Draw5(uint32_t xmin, uint32_t xmax, uint32_t ymin,
		  uint32_t y0, uint32_t ymax, char *tmpPtr,
		  uint32_t *pIndex);
static void Draw6(uint32_t xmin, uint32_t xmax, uint32_t ymin,
		  uint32_t y0, uint32_t ymax, char *tmpPtr,
		  uint32_t *pIndex);
static void Draw7(uint32_t xmin, uint32_t xmax, uint32_t ymin,
		  uint32_t ymax, char *tmpPtr, uint32_t *pIndex);
static void Draw8(uint32_t xmin, uint32_t xmax, uint32_t ymin,
		  uint32_t y0, uint32_t ymax, char *tmpPtr,
		  uint32_t *pIndex);
static void Draw9(uint32_t xmin, uint32_t xmax, uint32_t ymin,
		  uint32_t y0, uint32_t ymax, char *tmpPtr,
		  uint32_t *pIndex);
static void show_move( uint32_t lastX, uint32_t lastY,
                uint32_t currentX, uint32_t currentY,
                char * tmpPtr, uint32_t *pIndex);
static void limit2C(uint32_t currentX, uint32_t currentY,
		    uint32_t *eolXNeg, uint32_t *eolX001,
		    uint32_t *eolX002, uint32_t *eolXPos,
		    uint32_t *eolYNeg, uint32_t * eolYPos);
static void limitXY(uint32_t currentX, uint32_t currentY,
		    uint32_t *eolXNeg, uint32_t *eolXPos,
		    uint32_t *eolYNeg, uint32_t * eolYPos);

//Start of local functions
void DoShowTargets(struct lg_master *pLgMaster, char * Parameters, uint32_t respondToWhom )
{
    char * wr_buf;
    char * tmpPtr;
    int maxSize;
    uint32_t index;
    int offset;
    double * pExternalAngles;
    int nTargets; 
    double Xtemp;
    double Ytemp;
    uint32_t Xi;
    uint32_t Yi;
    uint32_t Xarr[32];
    uint32_t Yarr[32];
    uint32_t flag[32];
    uint32_t currentX;
    uint32_t currentY;
    uint32_t lastX;
    uint32_t lastY;
    int i;
    uint32_t resp_len=(sizeof(uint32_t) + kCRCSize);
    struct parse_basic_resp *pResp = (struct parse_basic_resp *)pLgMaster->theResponseBuffer;

    memset((char *)pResp, 0, resp_len);
    maxSize = NPOINTS * 24;
    wr_buf = (char *)calloc( NPOINTS * 24, 4 );

    tmpPtr = wr_buf;

    offset = 0;
    nTargets = *(int32_t *)(&(Parameters)[offset]);

    gCoarse3SearchStep = 0x00080000;

#if ZDEBUG
    fprintf( stderr
           , "ShowTarget gCoarseSearchStep %x  (2) %x  (3)  %x\n"
           , gCoarseSearchStep
           , gCoarse2SearchStep
           , gCoarse3SearchStep
           );
    fprintf( stderr
           , "ShowTarget gNumberOfSpirals  %d\n"
           , gNumberOfSpirals
           );
#endif

    for ( i = 0; i < nTargets; i++ ) {
        offset = sizeof(int32_t) + i * (kSizeOf1TargetData);
        pExternalAngles = (double *)(&(Parameters)[offset]);
        Xtemp   = pExternalAngles[ 0 ];
        Ytemp   = pExternalAngles[ 1 ];
        ConvertExternalAnglesToBinary(pLgMaster, Xtemp, Ytemp, &Xi, &Yi );
        Xarr[i] = Xi;
        Yarr[i] = Yi;
        offset = sizeof(int32_t) + i * (kSizeOf1TargetData) + 2 * sizeof(double);
        flag[i] = *((uint32_t *)(&(Parameters)[offset]));
    }

    /*
     * draw marker
     */
    index = 0;
    lastX = Xarr[0];
    lastY = Yarr[0];
    for ( i = 0; i < nTargets; i++ ) {
#ifdef ZDEBUG
fprintf(stderr,"ShowTargets i %3d  ptr %08x  dex %5d\n", i, tmpPtr, index );
#endif
        if ( flag[i] == 1 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do1( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 2 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do2( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 3 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do3( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 4 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do4( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 5 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do5( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 6 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do6( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 7 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do7( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 8 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do8( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 9 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do9( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 10 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do10( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 11 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do11( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 12 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do12( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 13 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do13( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 14 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do14( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 15 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do15( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 16 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do16( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 17 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do17( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 18 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do18( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 19 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do19( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 20 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do20( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 21 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do21( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 22 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do22( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 23 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do23( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 24 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           Do24( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 31 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           DoCross( currentX, currentY, tmpPtr, &index );
        }
        if ( flag[i] == 32 ) {
           currentX = Xarr[i];
           currentY = Yarr[i];
           show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
           lastX = currentX;
           lastY = currentY;
           DoSquare( currentX, currentY, tmpPtr, &index );
        }
    }
    currentX = Xarr[0];
    currentY = Yarr[0];
    show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );

#ifdef ZDEBUG
    for ( i=0; i<index; i+= 8 ) {
      x = *(int32_t *)(&(tmpPtr[i+0]));
      y = *(int32_t *)(&(tmpPtr[i+4]));
      printf( "%9d %9d\n", x, y );
    }
#endif

    if ( index < maxSize ) {
      JustDoDisplay(pLgMaster, wr_buf, index );

       free( wr_buf );
       pResp->hdr.status = RESPGOOD;
       HandleResponse ( pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
       return;
    } else {
       free( wr_buf );
       pResp->hdr.status = RESPFAIL;
       HandleResponse ( pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
       return;
    }
}

static void DoCross(uint32_t currentX, uint32_t currentY,
		    char *tmpPtr, uint32_t*pIndex)
{
  struct lg_xydata *pXYdata;
  uint32_t index;
  uint32_t eolXNeg;
  uint32_t eolXPos;
  uint32_t eolYNeg;
  uint32_t eolYPos;
  uint32_t x;
  uint32_t y;
  int j;

  index = *pIndex;

  if (currentX >= (kMaxNumberX - gNumberStep))
    eolXPos   =  kMaxNumberX;
  else
    eolXPos =  currentX + gNumberStep; 

  if (currentX <= (kMinNumberX + gNumberStep))
    eolXNeg = kMinNumberX;
  else
    eolXNeg = currentX - gNumberStep; 
  if (currentY >= (kMaxNumberY - gNumberStep))
    eolYPos   =  kMaxNumberY;
  else
    eolYPos = currentY + gNumberStep; 
  if (currentY <= (kMinNumberY + gNumberStep))
    eolYNeg = kMinNumberY;
  else
    eolYNeg = currentY - gNumberStep; 

  for (j = 0; j < 10; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(currentX & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(currentY & kMaxUnsigned);
      SetLowBeam(pXYdata);
      index += sizeof(struct lg_xydata);
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(currentX & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(currentY & kMaxUnsigned);
      SetLowBeam(pXYdata);
      index += sizeof(struct lg_xydata);
    }
  for (x=currentX; ((x<=eolXPos) && (x>=currentX)); (x+= 64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(x & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(currentY & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata);
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(eolXPos & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(currentY & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata);
    }
  for (x=eolXPos; ((x>=eolXNeg) && (x<=eolXPos)); (x-=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(x & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(currentY & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata);
    }
  for (j=0; j<2; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(eolXNeg & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(currentY & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata);
    }
  for (x=eolXNeg; ((x<=currentX) && (x>=eolXNeg)); (x+=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(x & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(currentY & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata);
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(currentX & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(currentY & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata);
    }
  for (y=currentY; ((y>=eolYNeg) && (y<=currentY)); (y-=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(currentX & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(y & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata);
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(currentX & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(eolYNeg & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata);
    }
  for (y=eolYNeg; ((y<=eolYPos) && (y>=eolYNeg)); (y+=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(currentX & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(y & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata);
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(currentX & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(eolYPos & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata);
    }
  for (y=eolYPos; ((y>=currentY) && (y<=eolYPos)); (y-=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(currentX & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(y & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata);
    }
  x = (int32_t)currentX;
  y = (int32_t)currentY;
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(currentX & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(currentY & kMaxUnsigned);
      SetLowBeam(pXYdata);
      index += sizeof(struct lg_xydata);
    }
    *pIndex = index;
    return;
}

static void DoSquare(uint32_t currentX, uint32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
  struct lg_xydata *pXYdata;
  int index;
  uint32_t eolXNeg;
  uint32_t eolXPos;
  uint32_t eolYNeg;
  uint32_t eolYPos;
  uint32_t x;
  uint32_t y;
  int j;

  index = *pIndex;

  if (currentX >= (kMaxNumberX - gNumberStep))
    eolXPos   =  kMaxNumberX;
  else
    eolXPos =  currentX + gNumberStep; 
  if (currentX <= (kMinNumberX + gNumberStep))
    eolXNeg   =  kMinNumberX;
  else
    eolXNeg =  currentX - gNumberStep; 
  if (currentY >= (kMaxNumberY - gNumberStep))
    eolYPos   =  kMaxNumberY;
  else
    eolYPos = currentY + gNumberStep;
  if (currentY <= (kMinNumberY + gNumberStep))
    eolYNeg   =  kMinNumberY;
  else
    eolYNeg = currentY - gNumberStep; 
  for (j = 0; j < 10; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(eolXNeg & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(eolYNeg & kMaxUnsigned);
      SetLowBeam(pXYdata);
      index += sizeof(struct lg_xydata); 
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(eolXNeg & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(eolYNeg & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata); 
    }
  for (x=currentX; ((x<=eolXPos) && (x>=currentX)); (x+=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(x & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(eolYNeg & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata); 
    }
  for (j = 0; j < 5; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(eolXPos & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(eolYNeg & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata); 
    }
  for (y=eolYNeg; ((y<=eolYPos) && (y>=eolYNeg)); (y+=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(eolXPos & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(y & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata); 
    }
  for (j = 0; j < 5; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(eolXPos & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(eolYPos & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata); 
    }
  for (x = eolXPos; ((x>=eolXNeg) && (x<=eolXPos)); (x-=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(x & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(eolYPos & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata); 
    }
  for (j = 0; j < 5; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(eolXNeg & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(eolYPos & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata); 
    }
  for (y=eolYPos; ((y>=eolYNeg) && (y<=eolYPos)); (y-=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(eolXNeg & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(y & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata); 
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(eolXNeg & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(eolYNeg & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata); 
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(eolXNeg & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(eolYNeg & kMaxUnsigned);
      SetLowBeam(pXYdata);
      index += sizeof(struct lg_xydata); 
    }
  *pIndex = index;
  return;
}

void show_move(
                uint32_t lastX,
                uint32_t lastY,
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            )
{
    uint32_t new_xpos;
    uint32_t new_ypos;
    uint32_t last_xpos;
    uint32_t last_ypos;

    last_xpos = (int32_t)lastX;
    last_ypos = (int32_t)lastY;
    new_xpos  = (int32_t)currentX;
    new_ypos  = (int32_t)currentY;

    draw_dark( last_xpos, last_ypos, new_xpos, new_ypos, tmpPtr, pIndex);

}

static void Do1(uint32_t currentX, uint32_t currentY, char *tmpPtr,uint32_t * pIndex)
{
  uint32_t XNeg;
  uint32_t XPos;
  uint32_t YNeg;
  uint32_t YPos;

  limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);

  Draw1( currentX, YNeg, YPos, tmpPtr, pIndex );
}
static void Do2(uint32_t currentX, uint32_t currentY, char *tmpPtr,uint32_t * pIndex)
{
  uint32_t XNeg;
  uint32_t XPos;
  uint32_t YNeg;
  uint32_t YPos;

  limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);
  
  Draw2( XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex );
}


static void Do3(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);

    Draw3( XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex );
}


static void Do4(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);

    Draw4( XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex );
}


static void Do5(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);

    Draw5( XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex );
}

static void Do6(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);

    Draw6( XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex );

}
static void Do7(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);

    Draw7( XNeg,XPos,YNeg,YPos, tmpPtr, pIndex );

}
static void Do8(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);

    Draw8( XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex );

}
static void Do9(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);

    Draw9( XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex );

}



static void Do10(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t X001;
    uint32_t X002;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw0(X002,XPos,YNeg,YPos,tmpPtr,pIndex);

}


static void Do11(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t X001;
    uint32_t X002;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw1(XPos,YNeg,YPos,tmpPtr,pIndex);

}


static void Do12(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t X001;
    uint32_t X002;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw2(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);

}


static void Do13(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t X001;
    uint32_t X002;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw3(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);

}


static void Do14(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t X001;
    uint32_t X002;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw4(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);

}


static void Do15(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t X001;
    uint32_t X002;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw5(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);
    return;
}

static void Do16(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t X001;
    uint32_t X002;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw6(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);
    return;
}

static void Do17(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t X001;
    uint32_t X002;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw7(X002,XPos,YNeg,YPos,tmpPtr,pIndex);
    return;
}


static void Do18(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t X001;
    uint32_t X002;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw8(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);
    return;
}

static void Do19(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t X001;
    uint32_t X002;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw9(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);
}

static void Do20(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t X001;
    uint32_t X002;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw2(XNeg,X001,YNeg,currentY,YPos,tmpPtr,pIndex);
    Draw0(X002,XPos,YNeg,YPos,tmpPtr,pIndex);
    return;
}


static void Do21(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t X001;
    uint32_t X002;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw2(XNeg,X001,YNeg,currentY,YPos,tmpPtr,pIndex);
    Draw1(XPos,YNeg,YPos,tmpPtr,pIndex);
    return;
}


static void Do22(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t X001;
    uint32_t X002;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw2(XNeg,X001,YNeg,currentY,YPos,tmpPtr,pIndex);
    Draw2(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);

}


static void Do23(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t X001;
    uint32_t X002;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw2(XNeg,X001,YNeg,currentY,YPos,tmpPtr,pIndex);
    Draw3(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);

}

static void Do24(
                uint32_t currentX,
                uint32_t currentY,
                char * tmpPtr,
                uint32_t *pIndex
            ) {
    uint32_t XNeg;
    uint32_t X001;
    uint32_t X002;
    uint32_t XPos;
    uint32_t YNeg;
    uint32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw2(XNeg,X001,YNeg,currentY,YPos,tmpPtr,pIndex);
    Draw4(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);

}

static void Draw0(uint32_t xmin, uint32_t xmax, uint32_t ymin,
		  uint32_t ymax, char *tmpPtr, uint32_t *pIndex)
{
    off_pause(  xmin, ymin, tmpPtr, pIndex );
    draw_line(  xmin, ymin, xmin, ymax, tmpPtr, pIndex );
    draw_line(  xmin, ymax, xmax, ymax, tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmax, ymin, tmpPtr, pIndex );
    draw_line(  xmax, ymin, xmin, ymin, tmpPtr, pIndex );
    off_pause(  xmin, ymin, tmpPtr, pIndex );
    return;
}

static void Draw1(uint32_t x0, uint32_t ymin, uint32_t ymax,
		  char *tmpPtr, uint32_t *pIndex)
{
    off_pause(  x0, ymin,             tmpPtr, pIndex );
    draw_line(  x0, ymin, x0, ymax, tmpPtr, pIndex );
    off_pause(  x0, ymin,             tmpPtr, pIndex );
}

static void Draw2(uint32_t xmin, uint32_t xmax, uint32_t ymin,
		  uint32_t y0, uint32_t ymax, char *tmpPtr,
		  uint32_t *pIndex)
{

    off_pause(  xmin, ymax,             tmpPtr, pIndex );
    draw_line(  xmin, ymax, xmax, ymax, tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmax, y0,   tmpPtr, pIndex );
    draw_line(  xmax, y0,   xmin, y0,   tmpPtr, pIndex );
    draw_line(  xmin, y0,   xmin, ymin, tmpPtr, pIndex );
    draw_line(  xmin, ymin, xmax, ymin, tmpPtr, pIndex );
    off_pause(  xmax, ymin,             tmpPtr, pIndex );

}

static void Draw3(uint32_t xmin, uint32_t xmax, uint32_t ymin,
		  uint32_t y0, uint32_t ymax, char *tmpPtr,
		  uint32_t *pIndex)
{
    off_pause(  xmin, ymax,             tmpPtr, pIndex );
    draw_line(  xmin, ymax, xmax, ymax, tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmax, y0,   tmpPtr, pIndex );
    draw_line(  xmax, y0,   xmin, y0,   tmpPtr, pIndex );
    draw_line(  xmin, y0,   xmax, y0,   tmpPtr, pIndex );
    draw_line(  xmax, y0,   xmax, ymin, tmpPtr, pIndex );
    draw_line(  xmax, ymin, xmin, ymin, tmpPtr, pIndex );
    off_pause(  xmin, ymin,             tmpPtr, pIndex );
}

static void Draw4(uint32_t xmin, uint32_t xmax, uint32_t ymin,
		  uint32_t y0, uint32_t ymax, char *tmpPtr,
		  uint32_t *pIndex)
{
    off_pause(  xmin, ymax,             tmpPtr, pIndex );
    draw_line(  xmin, ymax, xmin, y0,   tmpPtr, pIndex );
    draw_line(  xmin, y0,   xmax, y0,   tmpPtr, pIndex );
    draw_line(  xmax, y0,   xmax, ymax, tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmax, ymin, tmpPtr, pIndex );
    off_pause(  xmax, ymin,             tmpPtr, pIndex );
}

static void Draw5(uint32_t xmin, uint32_t xmax, uint32_t ymin,
		  uint32_t y0, uint32_t ymax, char *tmpPtr,
		  uint32_t *pIndex)
{
    off_pause(  xmax, ymax,             tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmin, ymax, tmpPtr, pIndex );
    draw_line(  xmin, ymax, xmin, y0,   tmpPtr, pIndex );
    draw_line(  xmin, y0,   xmax, y0,   tmpPtr, pIndex );
    draw_line(  xmax, y0,   xmax, ymin, tmpPtr, pIndex );
    draw_line(  xmax, ymin, xmin, ymin, tmpPtr, pIndex );
    off_pause(  xmin, ymin,             tmpPtr, pIndex );
}

static void Draw6(uint32_t xmin, uint32_t xmax, uint32_t ymin,
		  uint32_t y0, uint32_t ymax, char *tmpPtr,
		  uint32_t *pIndex)
{
    off_pause(  xmax, ymax,             tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmin, ymax, tmpPtr, pIndex );
    draw_line(  xmin, ymax, xmin, ymin, tmpPtr, pIndex );
    draw_line(  xmin, ymin, xmax, ymin, tmpPtr, pIndex );
    draw_line(  xmax, ymin, xmax, y0,   tmpPtr, pIndex );
    draw_line(  xmax, y0,   xmin, y0,   tmpPtr, pIndex );
    off_pause(  xmin, y0,               tmpPtr, pIndex );
}

static void Draw7(uint32_t xmin, uint32_t xmax, uint32_t ymin,
		  uint32_t ymax, char *tmpPtr, uint32_t *pIndex)
{
    off_pause(  xmin, ymin,             tmpPtr, pIndex );
    draw_line(  xmin, ymin, xmax, ymax, tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmin, ymax, tmpPtr, pIndex );
    off_pause(  xmin, ymax,             tmpPtr, pIndex );
}
static void Draw8(uint32_t xmin, uint32_t xmax, uint32_t ymin,
		  uint32_t y0, uint32_t ymax, char *tmpPtr,
		  uint32_t *pIndex)
{
    off_pause(  xmin, y0,               tmpPtr, pIndex );
    draw_line(  xmin, y0,   xmin, ymax, tmpPtr, pIndex );
    draw_line(  xmin, ymax, xmax, ymax, tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmax, y0,   tmpPtr, pIndex );
    draw_line(  xmax, y0,   xmin, y0,   tmpPtr, pIndex );
    draw_line(  xmin, y0,   xmin, ymin, tmpPtr, pIndex );
    draw_line(  xmin, ymin, xmax, ymin, tmpPtr, pIndex );
    draw_line(  xmax, ymin, xmax, y0,   tmpPtr, pIndex );
    off_pause(  xmax, y0,               tmpPtr, pIndex );
}
static void Draw9(uint32_t xmin, uint32_t xmax, uint32_t ymin,
		  uint32_t y0, uint32_t ymax, char *tmpPtr,
		  uint32_t *pIndex)
{
    off_pause(  xmax, ymin,             tmpPtr, pIndex );
    draw_line(  xmax, ymin, xmax, ymax, tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmin, ymax, tmpPtr, pIndex );
    draw_line(  xmin, ymax, xmin, y0,   tmpPtr, pIndex );
    draw_line(  xmin, y0,   xmax, y0,   tmpPtr, pIndex );
    off_pause(  xmax, y0,               tmpPtr, pIndex );
}

void draw_line(uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1,
	       char *tmpPtr, uint32_t *pIndex)
{
  struct lg_xydata *pXYdata;
  int index;
  uint32_t delX;
  uint32_t delY;
  uint32_t j;

  index = *pIndex;

  delX = (x1 - x0) / 16;
  delY = (y1 - y0) / 16;

  // pause at the start of line
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(x0 & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(y0 & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata); 
    }
  for (j=0; j <= 16; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)((x0 + (j * delX)) & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)((y0 + (j * delY)) & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata); 
    }
  // pause at the end of line
  for ( j = 0; j < 2; j++ )
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(x1 & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(y1 & kMaxUnsigned);
      SetHighBeam(pXYdata);
      index += sizeof(struct lg_xydata); 
    }
  *pIndex = index;
  return;
}

static void draw_dark(uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1,
		      char *tmpPtr, uint32_t *pIndex)
{
  struct lg_xydata *pXYdata;
  int index;
  uint32_t delX;
  uint32_t delY;
  uint32_t j;

  index = *pIndex;

  delX = (x1 - x0) / 60;
  delY = (y1 - y0) / 60;

  // pause at the start of line
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(x0 & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(y0 & kMaxUnsigned);
      index += sizeof(struct lg_xydata); 
    }
  for (j=0; j <= 60; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)((x0 + (j*delX)) & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)((y0 + (j*delY)) & kMaxUnsigned);
      index += sizeof(struct lg_xydata); 
    }
  // pause at the end of line
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(x1 & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(y1 & kMaxUnsigned);
      index += sizeof(struct lg_xydata); 
    }
  *pIndex = index;
  return;
}

static void off_pause(uint32_t x1, uint32_t y1, char *tmpPtr, uint32_t *pIndex)
{
  struct lg_xydata *pXYdata;
  uint32_t index;
  uint32_t j;

  index = *pIndex;

  for (j = 0; j < 10; j++)
    {
      pXYdata = (struct lg_xydata *)&tmpPtr[j+index];
      pXYdata->xdata = (uint16_t)(x1 & kMaxUnsigned);
      pXYdata->ydata = (uint16_t)(y1 & kMaxUnsigned);
      index += sizeof(struct lg_xydata); 
    }
  *pIndex = index;
  return;
}

static void limitXY(uint32_t currentX, uint32_t currentY,
		    uint32_t *eolXNeg, uint32_t *eolXPos,
		    uint32_t *eolYNeg, uint32_t * eolYPos)
{
    
    if ( (int32_t)currentX >= (int32_t)(kMaxNumberX - gNumberStep) ) {
        *eolXPos   =  kMaxNumberX;
    } else {
        *eolXPos =  (int32_t)currentX            + gNumberStep; 
    }
    if ( (int32_t)currentX <= (int32_t)(kMinNumberX + gNumberStep) ) {
        *eolXNeg   =  kMinNumberX;
    } else {
        *eolXNeg =  (int32_t)currentX            - gNumberStep; 
    }
    if ( (int32_t)currentY >= (int32_t)(kMaxNumberY - gNumberStep) ) {
        *eolYPos   =  kMaxNumberY;
    } else {
        *eolYPos =  (int32_t)currentY            + gNumberStep; 
    }
    if ( (int32_t)currentY <= (int32_t)(kMinNumberY + gNumberStep) ) {
        *eolYNeg   =  kMinNumberY;
    } else {
        *eolYNeg =  (int32_t)currentY            - gNumberStep; 
    }

}

static void limit2C(uint32_t currentX, uint32_t currentY,
		    uint32_t *eolXNeg, uint32_t *eolX001,
		    uint32_t *eolX002, uint32_t *eolXPos,
		    uint32_t *eolYNeg, uint32_t * eolYPos)
{
    
    if ( (int32_t)currentX >= (int32_t)(kMaxNumberX - gNumberStep) ) {
        *eolXPos   =  kMaxNumberX;
    } else {
        *eolXPos =  (int32_t)currentX            + gNumberStep; 
    }
    if ( (int32_t)currentX >= (int32_t)(kMaxNumberX - gNumberStep/3) ) {
        *eolX002   =  kMaxNumberX;
    } else {
        *eolX002 =  (int32_t)currentX            + gNumberStep/3; 
    }
    if ( (int32_t)currentX <= (int32_t)(kMinNumberX + gNumberStep/3) ) {
        *eolX001   =  kMinNumberX;
    } else {
        *eolX001 =  (int32_t)currentX            - gNumberStep/3; 
    }
    if ( (int32_t)currentX <= (int32_t)(kMinNumberX + gNumberStep) ) {
        *eolXNeg   =  kMinNumberX;
    } else {
        *eolXNeg =  (int32_t)currentX            - gNumberStep; 
    }
    if ( (int32_t)currentY >= (int32_t)(kMaxNumberY - gNumberStep) ) {
        *eolYPos   =  kMaxNumberY;
    } else {
        *eolYPos =  (int32_t)currentY            + gNumberStep; 
    }
    if ( (int32_t)currentY <= (int32_t)(kMinNumberY + gNumberStep) ) {
        *eolYNeg   =  kMinNumberY;
    } else {
        *eolYNeg =  (int32_t)currentY            - gNumberStep; 
    }

}
