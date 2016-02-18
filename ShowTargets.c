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


#define NPOINTS 2048

   /*
    *  size based on default values of gNumberOfSpiral
    *  and gCoarseSearchStep
    */
static uint32_t gNumberStep =  64 * 0x0008;

static int32_t gCoarse3SearchStep;

static void DoCross(int32_t currentX, int32_t currentY,
		    char *tmpPtr, uint32_t *pIndex);
static void DoSquare(int32_t currentX, int32_t currentY,
		     char *tmpPtr, uint32_t *pIndex);
static void draw_line(int32_t x0, int32_t y0, int32_t x1, int32_t y1,
		      char *tmpPtr, uint32_t *pIndex);
static void draw_dark(int32_t x0, int32_t y0, int32_t x1, int32_t y1,
		      char *tmpPtr, uint32_t *pIndex);
static void off_pause(int32_t x1, int32_t y1, char *tmpPtr, uint32_t *pIndex);
static void Draw0(int32_t xmin, int32_t xmax, int32_t ymin,
		  int32_t ymax, char *tmpPtr, uint32_t *pIndex);
static void Draw1(int32_t x0, int32_t ymin, int32_t ymax,
		  char *tmpPtr, uint32_t *pIndex);
static void Do1(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do2(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do3(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do4(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do5(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do6(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do7(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do8(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do9(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do10(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do11(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do12(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do13(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do14(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do15(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do16(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do17(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do18(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do19(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do20(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do21(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do22(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do23(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Do24(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex);
static void Draw2(int32_t xmin, int32_t xmax, int32_t ymin,
		  int32_t y0, int32_t ymax, char *tmpPtr,
		  uint32_t *pIndex);
static void Draw3(int32_t xmin, int32_t xmax, int32_t ymin,
		  int32_t y0, int32_t ymax, char *tmpPtr,
		  uint32_t *pIndex);
static void Draw4(int32_t xmin, int32_t xmax, int32_t ymin,
		  int32_t y0, int32_t ymax, char *tmpPtr,
		  uint32_t *pIndex);
static void Draw5(int32_t xmin, int32_t xmax, int32_t ymin,
		  int32_t y0, int32_t ymax, char *tmpPtr,
		  uint32_t *pIndex);
static void Draw6(int32_t xmin, int32_t xmax, int32_t ymin,
		  int32_t y0, int32_t ymax, char *tmpPtr,
		  uint32_t *pIndex);
static void Draw7(int32_t xmin, int32_t xmax, int32_t ymin,
		  int32_t ymax, char *tmpPtr, uint32_t *pIndex);
static void Draw8(int32_t xmin, int32_t xmax, int32_t ymin,
		  int32_t y0, int32_t ymax, char *tmpPtr,
		  uint32_t *pIndex);
static void Draw9(int32_t xmin, int32_t xmax, int32_t ymin,
		  int32_t y0, int32_t ymax, char *tmpPtr,
		  uint32_t *pIndex);
static void show_move(int32_t lastX, int32_t lastY,
                int32_t currentX, int32_t currentY,
                char * tmpPtr, uint32_t *pIndex);
static void limit2C(int32_t currentX, int32_t currentY,
		    int32_t *eolXNeg, int32_t *eolX001,
		    int32_t *eolX002, int32_t *eolXPos,
		    int32_t *eolYNeg, int32_t * eolYPos);
static void limitXY(int32_t currentX, int32_t currentY,
		    int32_t *eolXNeg, int32_t *eolXPos,
		    int32_t *eolYNeg, int32_t * eolYPos);

//Start of local functions
void DoShowTargets(struct lg_master *pLgMaster, struct parse_showtgt_parms *Parameters, uint32_t respondToWhom )
{
    char *wr_buf;
    char *tmpPtr;
    int maxSize;
    uint32_t index;
    int nTargets; 
    double Xtemp;
    double Ytemp;
    int32_t Xi;
    int32_t Yi;
    int32_t Xarr[32];
    int32_t Yarr[32];
    int32_t flag[32];
    int32_t currentX;
    int32_t currentY;
    int32_t lastX;
    int32_t lastY;
    int i;
    struct parse_basic_resp *pResp = (struct parse_basic_resp *)pLgMaster->theResponseBuffer;

    memset((char *)pResp, 0, sizeof(struct parse_basic_resp));
    memset((char *)&Xarr, 0, sizeof(Xarr));
    memset((char *)&Yarr, 0, sizeof(Yarr));
    memset((char *)&flag, 0, sizeof(flag));
    maxSize = NPOINTS * 24;
    wr_buf = (char *)calloc( NPOINTS * 24, 4 );
    memset(wr_buf, 0, (NPOINTS * 24 * 4));
    
    tmpPtr = wr_buf;

    nTargets = Parameters->inp_numpairs;
    if(nTargets > kNumberOfFlexPoints)
      {
	pResp->hdr.status = RESPFAIL;
	HandleResponse ( pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
	return;
      }
    gCoarse3SearchStep = 0x0008;
#ifdef PATDEBUG
    fprintf(stderr,"\nSHOWTGT: num_pairs %d",nTargets);
#endif
    for (i = 0; i < nTargets; i++) {
        Xtemp = Parameters->inp_targetpairs[i].Xangle;
        Ytemp = Parameters->inp_targetpairs[i].Yangle;
        ConvertExternalAnglesToBinary(pLgMaster, Xtemp, Ytemp, &Xi, &Yi );
        Xarr[i] = Xi;
        Yarr[i] = Yi;
        flag[i] = Parameters->inp_targetpairs[i].flag;
#ifdef PATDEBUG
	fprintf(stderr,"\nSHOWTGT: Xangle %fx,Yangle %fx",Xtemp,Ytemp);
#endif
    }

    /*
     * draw marker
     */
    index = 0;
    lastX = Xarr[0];
    lastY = Yarr[0];
    for ( i = 0; i < nTargets; i++ ) {
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

static void DoCross(int32_t currentX, int32_t currentY,
		    char *tmpPtr, uint32_t *pIndex)
{
  struct lg_xydata *pXYdata;
  int32_t eolXNeg;
  int32_t eolXPos;
  int32_t eolYNeg;
  int32_t eolYPos;
  int32_t x;
  int32_t y;
  int j;

  if (currentX >= (int32_t)(kMaxSigned - gNumberStep))
    eolXPos = kMaxSigned;
  else
    eolXPos = currentX + gNumberStep; 

  if (currentX <= (int32_t)(kMinSigned + gNumberStep))
    eolXNeg = kMinSigned;
  else
    eolXNeg = currentX - gNumberStep; 
  if (currentY >= (int32_t)(kMaxSigned - gNumberStep))
    eolYPos = kMaxSigned;
  else
    eolYPos = currentY + gNumberStep; 
  if (currentY <= (int32_t)(kMinSigned + gNumberStep))
    eolYNeg = kMinSigned;
  else
    eolYNeg = currentY - gNumberStep; 

  pXYdata = (struct lg_xydata *)(tmpPtr + *pIndex);
  for (j = 0; j < 10; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(currentX & kMaxSigned);
      pXYdata->ydata = (int16_t)(currentY & kMaxSigned);
      SetLowBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(currentX & kMaxSigned);
      pXYdata->ydata = (int16_t)(currentY & kMaxSigned);
      SetLowBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (x=currentX; ((x<=eolXPos) && (x>=currentX)); (x+= 64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(x & kMaxSigned);
      pXYdata->ydata = (int16_t)(currentY & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(eolXPos & kMaxSigned);
      pXYdata->ydata = (int16_t)(currentY & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (x=eolXPos; ((x>=eolXNeg) && (x<=eolXPos)); (x-=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(x & kMaxSigned);
      pXYdata->ydata = (int16_t)(currentY & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (j=0; j<2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(eolXNeg & kMaxSigned);
      pXYdata->ydata = (int16_t)(currentY & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (x=eolXNeg; ((x<=currentX) && (x>=eolXNeg)); (x+=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(x & kMaxSigned);
      pXYdata->ydata = (int16_t)(currentY & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(currentX & kMaxSigned);
      pXYdata->ydata = (int16_t)(currentY & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (y=currentY; ((y>=eolYNeg) && (y<=currentY)); (y-=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(currentX & kMaxSigned);
      pXYdata->ydata = (int16_t)(y & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(currentX & kMaxSigned);
      pXYdata->ydata = (int16_t)(eolYNeg & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (y=eolYNeg; ((y<=eolYPos) && (y>=eolYNeg)); (y+=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(currentX & kMaxSigned);
      pXYdata->ydata = (int16_t)(y & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(currentX & kMaxSigned);
      pXYdata->ydata = (int16_t)(eolYPos & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (y=eolYPos; ((y>=currentY) && (y<=eolYPos)); (y-=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(currentX & kMaxSigned);
      pXYdata->ydata = (int16_t)(y & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(currentX & kMaxSigned);
      pXYdata->ydata = (int16_t)(currentY & kMaxSigned);
      SetLowBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
    return;
}

static void DoSquare(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
  struct lg_xydata *pXYdata;
  int32_t eolXNeg;
  int32_t eolXPos;
  int32_t eolYNeg;
  int32_t eolYPos;
  int32_t x;
  int32_t y;
  int j;

  if (currentX >= (kMaxSigned - gNumberStep))
    eolXPos   =  kMaxSigned;
  else
    eolXPos =  currentX + gNumberStep; 
  if (currentX <= (kMinSigned + gNumberStep))
    eolXNeg   =  kMinSigned;
  else
    eolXNeg =  currentX - gNumberStep; 
  if (currentY >= (kMaxSigned - gNumberStep))
    eolYPos   =  kMaxSigned;
  else
    eolYPos = currentY + gNumberStep;
  if (currentY <= (kMinSigned + gNumberStep))
    eolYNeg   =  kMinSigned;
  else
    eolYNeg = currentY - gNumberStep; 

  pXYdata = (struct lg_xydata *)(tmpPtr + *pIndex);
  for (j = 0; j < 10; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(eolXNeg & kMaxSigned);
      pXYdata->ydata = (int16_t)(eolYNeg & kMaxSigned);
      SetLowBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(eolXNeg & kMaxSigned);
      pXYdata->ydata = (int16_t)(eolYNeg & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (x=currentX; ((x<=eolXPos) && (x>=currentX)); (x+=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(x & kMaxSigned);
      pXYdata->ydata = (int16_t)(eolYNeg & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (j = 0; j < 5; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(eolXPos & kMaxSigned);
      pXYdata->ydata = (int16_t)(eolYNeg & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (y=eolYNeg; ((y<=eolYPos) && (y>=eolYNeg)); (y+=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(eolXPos & kMaxSigned);
      pXYdata->ydata = (int16_t)(y & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (j = 0; j < 5; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(eolXPos & kMaxSigned);
      pXYdata->ydata = (int16_t)(eolYPos & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (x = eolXPos; ((x>=eolXNeg) && (x<=eolXPos)); (x-=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(x & kMaxSigned);
      pXYdata->ydata = (int16_t)(eolYPos & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (j = 0; j < 5; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(eolXNeg & kMaxSigned);
      pXYdata->ydata = (int16_t)(eolYPos & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (y=eolYPos; ((y>=eolYNeg) && (y<=eolYPos)); (y-=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(eolXNeg & kMaxSigned);
      pXYdata->ydata = (int16_t)(y & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(eolXNeg & kMaxSigned);
      pXYdata->ydata = (uint16_t)(eolYNeg & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (uint16_t)(eolXNeg & kMaxSigned);
      pXYdata->ydata = (uint16_t)(eolYNeg & kMaxSigned);
      SetLowBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  return;
}

void show_move(int32_t lastX, int32_t lastY,
                int32_t currentX, int32_t currentY,
                char * tmpPtr, uint32_t *pIndex)
{
    int32_t new_xpos;
    int32_t new_ypos;
    int32_t last_xpos;
    int32_t last_ypos;

    last_xpos = lastX;
    last_ypos = lastY;
    new_xpos  = currentX;
    new_ypos  = currentY;
    draw_dark(last_xpos, last_ypos, new_xpos, new_ypos, tmpPtr, pIndex);
    return;
}

static void Do1(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex)
{
  int32_t XNeg;
  int32_t XPos;
  int32_t YNeg;
  int32_t YPos;

  limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);
  Draw1( currentX, YNeg, YPos, tmpPtr, pIndex );
  return;
}
static void Do2(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex)
{
  int32_t XNeg;
  int32_t XPos;
  int32_t YNeg;
  int32_t YPos;

  limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);
  Draw2(XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex);
  return;
}

static void Do3(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);
    Draw3(XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex);
    return;
}


static void Do4(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);
    Draw4(XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex);
    return;
}


static void Do5(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);
    Draw5(XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex);
    return;
}

static void Do6(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);
    Draw6( XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex );
    return;
}
static void Do7(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);
    Draw7( XNeg,XPos,YNeg,YPos, tmpPtr, pIndex );
    return;
}
static void Do8(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);
    Draw8( XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex );
    return;
}
static void Do9(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);
    Draw9(XNeg,XPos,YNeg,currentY,YPos, tmpPtr, pIndex);
    return;
}



static void Do10(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw0(X002,XPos,YNeg,YPos,tmpPtr,pIndex);
    return;
}


static void Do11(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw1(XPos,YNeg,YPos,tmpPtr,pIndex);
    return;
}


static void Do12(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw2(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);
    return;
}


static void Do13(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw3(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);
    return;
}


static void Do14(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw4(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);
    return;
}


static void Do15(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw5(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);
    return;
}

static void Do16(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw6(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);
    return;
}

static void Do17(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw7(X002,XPos,YNeg,YPos,tmpPtr,pIndex);
    return;
}


static void Do18(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw8(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);
    return;
}

static void Do19(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw9(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);
    return;
}

static void Do20(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw2(XNeg,X001,YNeg,currentY,YPos,tmpPtr,pIndex);
    Draw0(X002,XPos,YNeg,YPos,tmpPtr,pIndex);
    return;
}


static void Do21(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw2(XNeg,X001,YNeg,currentY,YPos,tmpPtr,pIndex);
    Draw1(XPos,YNeg,YPos,tmpPtr,pIndex);
    return;
}


static void Do22(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw2(XNeg,X001,YNeg,currentY,YPos,tmpPtr,pIndex);
    Draw2(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);
    return;
}


static void Do23(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw2(XNeg,X001,YNeg,currentY,YPos,tmpPtr,pIndex);
    Draw3(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);
    return;
}

static void Do24(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int32_t XNeg;
    int32_t X001;
    int32_t X002;
    int32_t XPos;
    int32_t YNeg;
    int32_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw2(XNeg,X001,YNeg,currentY,YPos,tmpPtr,pIndex);
    Draw4(X002,XPos,YNeg,currentY,YPos,tmpPtr,pIndex);
    return;
}

static void Draw0(int32_t xmin, int32_t xmax, int32_t ymin,
		  int32_t ymax, char *tmpPtr, uint32_t *pIndex)
{
    off_pause(xmin, ymin, tmpPtr, pIndex );
    draw_line(xmin, ymin, xmin, ymax, tmpPtr, pIndex );
    draw_line(xmin, ymax, xmax, ymax, tmpPtr, pIndex );
    draw_line(xmax, ymax, xmax, ymin, tmpPtr, pIndex );
    draw_line(xmax, ymin, xmin, ymin, tmpPtr, pIndex );
    off_pause(xmin, ymin, tmpPtr, pIndex );
    return;
}

static void Draw1(int32_t x0, int32_t ymin, int32_t ymax,
		  char *tmpPtr, uint32_t *pIndex)
{
    off_pause(x0, ymin, tmpPtr, pIndex);
    draw_line(x0, ymin, x0, ymax, tmpPtr, pIndex );
    off_pause(x0, ymin, tmpPtr, pIndex);
    return;
}

static void Draw2(int32_t xmin, int32_t xmax, int32_t ymin,
		  int32_t y0, int32_t ymax, char *tmpPtr,
		  uint32_t *pIndex)
{

    off_pause(xmin, ymax, tmpPtr, pIndex);
    draw_line(xmin, ymax, xmax, ymax, tmpPtr, pIndex );
    draw_line(xmax, ymax, xmax, y0,   tmpPtr, pIndex );
    draw_line(xmax, y0,   xmin, y0,   tmpPtr, pIndex );
    draw_line(xmin, y0,   xmin, ymin, tmpPtr, pIndex );
    draw_line(xmin, ymin, xmax, ymin, tmpPtr, pIndex );
    off_pause(xmax, ymin,             tmpPtr, pIndex );
    return;
}

static void Draw3(int32_t xmin, int32_t xmax, int32_t ymin,
		  int32_t y0, int32_t ymax, char *tmpPtr,
		  uint32_t *pIndex)
{
    off_pause(xmin, ymax,             tmpPtr, pIndex );
    draw_line(xmin, ymax, xmax, ymax, tmpPtr, pIndex );
    draw_line(xmax, ymax, xmax, y0,   tmpPtr, pIndex );
    draw_line(xmax, y0,   xmin, y0,   tmpPtr, pIndex );
    draw_line(xmin, y0,   xmax, y0,   tmpPtr, pIndex );
    draw_line(xmax, y0,   xmax, ymin, tmpPtr, pIndex );
    draw_line(xmax, ymin, xmin, ymin, tmpPtr, pIndex );
    off_pause(xmin, ymin,             tmpPtr, pIndex );
    return;
}

static void Draw4(int32_t xmin, int32_t xmax, int32_t ymin,
		  int32_t y0, int32_t ymax, char *tmpPtr,
		  uint32_t *pIndex)
{
    off_pause(xmin, ymax,             tmpPtr, pIndex );
    draw_line(xmin, ymax, xmin, y0,   tmpPtr, pIndex );
    draw_line(xmin, y0,   xmax, y0,   tmpPtr, pIndex );
    draw_line(xmax, y0,   xmax, ymax, tmpPtr, pIndex );
    draw_line(xmax, ymax, xmax, ymin, tmpPtr, pIndex );
    off_pause(xmax, ymin,             tmpPtr, pIndex );
    return;
}

static void Draw5(int32_t xmin, int32_t xmax, int32_t ymin,
		  int32_t y0, int32_t ymax, char *tmpPtr,
		  uint32_t *pIndex)
{
    off_pause(  xmax, ymax,             tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmin, ymax, tmpPtr, pIndex );
    draw_line(  xmin, ymax, xmin, y0,   tmpPtr, pIndex );
    draw_line(  xmin, y0,   xmax, y0,   tmpPtr, pIndex );
    draw_line(  xmax, y0,   xmax, ymin, tmpPtr, pIndex );
    draw_line(  xmax, ymin, xmin, ymin, tmpPtr, pIndex );
    off_pause(  xmin, ymin,             tmpPtr, pIndex );
    return;
}

static void Draw6(int32_t xmin, int32_t xmax, int32_t ymin,
		  int32_t y0, int32_t ymax, char *tmpPtr,
		  uint32_t *pIndex)
{
    off_pause(  xmax, ymax,             tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmin, ymax, tmpPtr, pIndex );
    draw_line(  xmin, ymax, xmin, ymin, tmpPtr, pIndex );
    draw_line(  xmin, ymin, xmax, ymin, tmpPtr, pIndex );
    draw_line(  xmax, ymin, xmax, y0,   tmpPtr, pIndex );
    draw_line(  xmax, y0,   xmin, y0,   tmpPtr, pIndex );
    off_pause(  xmin, y0,               tmpPtr, pIndex );
    return;
}

static void Draw7(int32_t xmin, int32_t xmax, int32_t ymin,
		  int32_t ymax, char *tmpPtr, uint32_t *pIndex)
{
    off_pause(  xmin, ymin,             tmpPtr, pIndex );
    draw_line(  xmin, ymin, xmax, ymax, tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmin, ymax, tmpPtr, pIndex );
    off_pause(  xmin, ymax,             tmpPtr, pIndex );
    return;
}
static void Draw8(int32_t xmin, int32_t xmax, int32_t ymin,
		  int32_t y0, int32_t ymax, char *tmpPtr,
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
    return;
}
static void Draw9(int32_t xmin, int32_t xmax, int32_t ymin,
		  int32_t y0, int32_t ymax, char *tmpPtr,
		  uint32_t *pIndex)
{
    off_pause(  xmax, ymin,             tmpPtr, pIndex );
    draw_line(  xmax, ymin, xmax, ymax, tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmin, ymax, tmpPtr, pIndex );
    draw_line(  xmin, ymax, xmin, y0,   tmpPtr, pIndex );
    draw_line(  xmin, y0,   xmax, y0,   tmpPtr, pIndex );
    off_pause(  xmax, y0,               tmpPtr, pIndex );
    return;
}

void draw_line(int32_t x0, int32_t y0, int32_t x1, int32_t y1,
	       char *tmpPtr, uint32_t *pIndex)
{
  struct lg_xydata *pXYdata;
  int32_t delX;
  int32_t delY;
  uint32_t j;

  delX = (x1 - x0) / 16;
  delY = (y1 - y0) / 16;

  // pause at the start of line
  pXYdata = (struct lg_xydata *)(tmpPtr + *pIndex);
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(x0 & kMaxSigned);
      pXYdata->ydata = (int16_t)(y0 & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (j=0; j <= 16; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)((x0 + (j * delX)) & kMaxSigned);
      pXYdata->ydata = (int16_t)((y0 + (j * delY)) & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  // pause at the end of line
  for ( j = 0; j < 2; j++ )
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(x1 & kMaxSigned);
      pXYdata->ydata = (int16_t)(y1 & kMaxSigned);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  return;
}

static void draw_dark(int32_t x0, int32_t y0, int32_t x1, int32_t y1,
		      char *tmpPtr, uint32_t *pIndex)
{
  struct lg_xydata *pXYdata;
  int32_t delX;
  int32_t delY;
  uint32_t j;

  delX = (x1 - x0) / 60;
  delY = (y1 - y0) / 60;

  // pause at the start of line
  pXYdata = (struct lg_xydata *)(tmpPtr + *pIndex);
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(x0 & kMaxSigned);
      pXYdata->ydata = (int16_t)(y0 & kMaxSigned);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (j=0; j <= 60; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)((x0 + (j*delX)) & kMaxSigned);
      pXYdata->ydata = (int16_t)((y0 + (j*delY)) & kMaxSigned);
      *pIndex += sizeof(struct lg_xydata); 
    }
  // pause at the end of line
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(x1 & kMaxSigned);
      pXYdata->ydata = (int16_t)(y1 & kMaxSigned);
      *pIndex += sizeof(struct lg_xydata); 
    }
  return;
}

static void off_pause(int32_t x1, int32_t y1, char *tmpPtr, uint32_t *pIndex)
{
  struct lg_xydata *pXYdata;
  uint32_t j;

  pXYdata = (struct lg_xydata *)(tmpPtr + *pIndex);
  for (j = 0; j < 10; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = (int16_t)(x1 & kMaxSigned);
      pXYdata->ydata = (int16_t)(y1 & kMaxSigned);
      *pIndex += sizeof(struct lg_xydata); 
    }
  return;
}

static void limitXY(int32_t currentX, int32_t currentY,
		    int32_t *eolXNeg, int32_t *eolXPos,
		    int32_t *eolYNeg, int32_t * eolYPos)
{
    
    if (currentX >= (kMaxSigned - gNumberStep))
      *eolXPos   =  kMaxSigned;
    else
      *eolXPos =  currentX + gNumberStep; 
    if (currentX <= (kMinSigned + gNumberStep))
      *eolXNeg   =  kMinSigned;
    else
      *eolXNeg = currentX - gNumberStep; 
    
    if (currentY >= (kMaxSigned - gNumberStep))
      *eolYPos   =  kMaxSigned;
    else
      *eolYPos = currentY + gNumberStep; 
    if (currentY <= (kMinSigned + gNumberStep))
      *eolYNeg   =  kMinSigned;
    else
      *eolYNeg = currentY - gNumberStep; 
    return;
}

static void limit2C(int32_t currentX, int32_t currentY,
		    int32_t *eolXNeg, int32_t *eolX001,
		    int32_t *eolX002, int32_t *eolXPos,
		    int32_t *eolYNeg, int32_t * eolYPos)
{
    
    if (currentX >= (kMaxSigned - gNumberStep))
      *eolXPos   =  kMaxSigned;
    else
      *eolXPos =  currentX + gNumberStep; 
    if (currentX >= (kMaxSigned - gNumberStep/3))
      *eolX002   =  kMaxSigned;
    else
      *eolX002 = currentX + gNumberStep/3; 
    if (currentX <= (kMinSigned + gNumberStep/3))
      *eolX001   =  kMinSigned;
    else
      *eolX001 = currentX - gNumberStep/3; 
    if (currentX <= (kMinSigned + gNumberStep))
      *eolXNeg   =  kMinSigned;
    else
      *eolXNeg =  currentX - gNumberStep; 
    if (currentY >= (kMaxSigned - gNumberStep))
      *eolYPos   =  kMaxSigned;
    else
      *eolYPos =  currentY + gNumberStep; 
    if (currentY <= (kMinSigned + gNumberStep))
      *eolYNeg   =  kMinSigned;
    else
      *eolYNeg = currentY - gNumberStep;
    return;
}
