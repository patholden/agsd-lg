/*
static char rcsid[] = "$Id$";
*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stddef.h>
#include <syslog.h>
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
static void draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
		      char *tmpPtr, uint32_t *pIndex);
static void draw_dark(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
		      char *tmpPtr, uint32_t *pIndex);
static void off_pause(int16_t x1, int16_t y1, char *tmpPtr, uint32_t *pIndex);
static void Draw0(int16_t xmin, int16_t xmax, int16_t ymin,
		  int16_t ymax, char *tmpPtr, uint32_t *pIndex);
static void Draw1(int16_t x0, int16_t ymin, int16_t ymax,
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
static void Draw2(int16_t xmin, int16_t xmax, int16_t ymin, int16_t y0, int16_t ymax,
		  char *tmpPtr, uint32_t *pIndex);
static void Draw3(int16_t xmin, int16_t xmax, int16_t ymin, int16_t y0, int16_t ymax,
		  char *tmpPtr, uint32_t *pIndex);
static void Draw4(int16_t xmin, int16_t xmax, int16_t ymin, int16_t y0, int16_t ymax,
		  char *tmpPtr, uint32_t *pIndex);
static void Draw5(int16_t xmin, int16_t xmax, int16_t ymin, int16_t y0, int16_t ymax,
		  char *tmpPtr, uint32_t *pIndex);
static void Draw6(int16_t xmin, int16_t xmax, int16_t ymin, int16_t y0, int16_t ymax,
		  char *tmpPtr, uint32_t *pIndex);
static void Draw7(int16_t xmin, int16_t xmax, int16_t ymin, int16_t ymax,
		  char *tmpPtr, uint32_t *pIndex);
static void Draw8(int16_t xmin, int16_t xmax, int16_t ymin, int16_t y0, int16_t ymax,
		  char *tmpPtr, uint32_t *pIndex);
static void Draw9(int16_t xmin, int16_t xmax, int16_t ymin, int16_t y0, int16_t ymax,
		  char *tmpPtr, uint32_t *pIndex);
static void show_move(int32_t lastX, int32_t lastY,
                int32_t currentX, int32_t currentY,
                char * tmpPtr, uint32_t *pIndex);
static void limit2C(int32_t currentX, int32_t currentY,
		    int16_t *eolXNeg, int16_t *eolX001,
		    int16_t *eolX002, int16_t *eolXPos,
		    int16_t *eolYNeg, int16_t * eolYPos);
static void limitXY(int32_t currentX, int32_t currentY,
		    int16_t *eolXNeg, int16_t *eolXPos,
		    int16_t *eolYNeg, int16_t * eolYPos);

//Start of local functions
void DoShowTargets(struct lg_master *pLgMaster, struct parse_showtgt_parms *Parameters, uint32_t respondToWhom )
{
    struct parse_basic_resp *pResp = (struct parse_basic_resp *)pLgMaster->theResponseBuffer;
    char *wr_buf;
    char *tmpPtr;
    int maxSize;
    uint32_t index;
    int nTargets; 
    double Xtemp;
    double Ytemp;
    int16_t Xarr[32];
    int16_t Yarr[32];
    int32_t flag[32];
    int i;
    int16_t Xi, Yi, currentX, currentY, lastX, lastY;

    memset((char *)pResp, 0, sizeof(struct parse_basic_resp));
    memset((char *)&Xarr, 0, sizeof(Xarr));
    memset((char *)&Yarr, 0, sizeof(Yarr));
    memset((char *)&flag, 0, sizeof(flag));
    maxSize = NPOINTS * 24;

    nTargets = Parameters->inp_numpairs;
    if(nTargets > kNumberOfFlexPoints)
      {
	pResp->hdr.status = RESPFAIL;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	return;
      }
    wr_buf = (char *)calloc(NPOINTS * 24, 4);
    
    tmpPtr = wr_buf;
    gCoarse3SearchStep = 0x0008;
    for (i = 0; i < nTargets; i++) {
        Xtemp = Parameters->inp_targetpairs[i].Xangle;
        Ytemp = Parameters->inp_targetpairs[i].Yangle;
        ConvertExternalAnglesToBinary(pLgMaster, Xtemp, Ytemp, &Xi, &Yi );
        Xarr[i] = Xi;
        Yarr[i] = Yi;
        flag[i] = Parameters->inp_targetpairs[i].flag;
#ifdef AGS_DEBUG
	syslog(LOG_DEBUG,"SHOWTGT: Xangle %fx,Yangle %fx,Xbin %x, Ybin %x, flag %d",Xtemp,Ytemp,Xarr[i],Yarr[i], flag[i]);
#endif
    }

    /*
     * draw marker
     */
    index = 0;
    lastX = Xarr[0];
    lastY = Yarr[0];
    for ( i = 0; i < nTargets; i++ ) {
      currentX = Xarr[i];
      currentY = Yarr[i];
      show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
      lastX = currentX;
      lastY = currentY;
#ifdef AGS_DEBUG
      syslog(LOG_DEBUG,"SHOWTGT: curX=%x, curY=%x, lastX=%x, lastY=%x, flag %d",currentX, currentY, lastX, lastY, flag[i]);
#endif
      switch(flag[i])
	{
	case SHOWTARGETNUM1:
	  Do1(currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETNUM2:
	  Do2(currentX, currentY, tmpPtr, &index);
	  break;
	case SHOWTARGETNUM3:
	  Do3(currentX, currentY, tmpPtr, &index);
	  break;
	case SHOWTARGETNUM4:
	  Do4(currentX, currentY, tmpPtr, &index);
	  break;
	case SHOWTARGETNUM5:
	  Do5(currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETNUM6:
	  Do6(currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETNUM7:
	  Do7(currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETNUM8:
	  Do8(currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETNUM9:
	  Do9(currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETNUM10:
	  Do10(currentX, currentY, tmpPtr, &index);
	  break;
	case SHOWTARGETNUM11:
	  Do11(currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETNUM12:
	  Do12(currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETNUM13:
	  Do13(currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETNUM14:
	  Do14(currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETNUM15:
	  Do15(currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETNUM16:
	  Do16( currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETNUM17:
	  Do17(currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETNUM18:
	  Do18(currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETNUM19:
	  Do19(currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETNUM20:
	  Do20(currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETNUM21:
	  Do21(currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETNUM22:
	  Do22(currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETNUM23:
	  Do23(currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETNUM24:
	  Do24(currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETCROSS:
	  DoCross(currentX, currentY, tmpPtr, &index);
	  break;
        case SHOWTARGETBOX:
	  DoSquare(currentX, currentY, tmpPtr, &index);
	  break;
	}
    }
    currentX = Xarr[0];
    currentY = Yarr[0];
    show_move( lastX, lastY, currentX, currentY, tmpPtr, &index );
#ifdef AGS_DEBUG
    syslog(LOG_DEBUG,"SHOWTGT: JUSTDODISP, first x=%x,y=%x, buflen %d",((struct lg_xydata *)wr_buf)->xdata, ((struct lg_xydata *)wr_buf)->ydata, index);
#endif

    if (index < maxSize)
      {
	JustDoDisplay(pLgMaster, wr_buf, index );
	pResp->hdr.status = RESPGOOD;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      }
    else
      {
	pResp->hdr.status = RESPFAIL;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      }
    // Let everything fall through here to free up buffer
    free(wr_buf);
    return;
}

static void DoCross(int32_t currentX, int32_t currentY,
		    char *tmpPtr, uint32_t *pIndex)
{
  struct lg_xydata *pXYdata;
  int16_t eolXNeg;
  int16_t eolXPos;
  int16_t eolYNeg;
  int16_t eolYPos;
  int16_t x;
  int16_t y;
  int j;
  int16_t outX;
  int16_t outY;

  if (!tmpPtr)
    return;
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

  outX = (int16_t)(currentX & kMaxUnsigned);
  outY = (int16_t)(currentY & kMaxUnsigned);
    
  pXYdata = (struct lg_xydata *)tmpPtr;
  for (j = 0; j < 10; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = outX;
      pXYdata->ydata = outY;
      SetLowBeam(pXYdata);
      //SetDarkBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = outX;
      pXYdata->ydata = outY;
      //SetDarkBeam(pXYdata);
      SetLowBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (x=outX; ((x<=eolXPos) && (x>=outX)); (x+= 64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = x;
      pXYdata->ydata = outY;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = eolXPos;
      pXYdata->ydata = outY;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (x=eolXPos; ((x>=eolXNeg) && (x<=eolXPos)); (x-=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = x;
      pXYdata->ydata = outY;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (j=0; j<2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = eolXNeg;
      pXYdata->ydata = outY;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (x=eolXNeg; ((x<=outX) && (x>=eolXNeg)); (x+=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = x;
      pXYdata->ydata = outY;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = outX;
      pXYdata->ydata = outY;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (y=outY; ((y>=eolYNeg) && (y<=outY)); (y-=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = outX;
      pXYdata->ydata = y;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = outX;
      pXYdata->ydata = eolYNeg;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (y=eolYNeg; ((y<=eolYPos) && (y>=eolYNeg)); (y+=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = outX;
      pXYdata->ydata = y;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = outX;
      pXYdata->ydata = eolYPos;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (y=eolYPos; ((y>=outY) && (y<=eolYPos)); (y-=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = outX;
      pXYdata->ydata = y;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = currentX;
      pXYdata->ydata = outY;
      //SetDarkBeam(pXYdata);
      SetLowBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata);
    }
    return;
}

static void DoSquare(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
  struct lg_xydata *pXYdata;
  int16_t eolXNeg;
  int16_t eolXPos;
  int16_t eolYNeg;
  int16_t eolYPos;
  int16_t x,outX;
  int16_t y;
  int j;

  if (!tmpPtr)
    return;
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

  outX = (int16_t)(currentX & kMaxUnsigned);

  pXYdata = (struct lg_xydata *)tmpPtr;
  for (j = 0; j < 10; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = eolXNeg;
      pXYdata->ydata = eolYNeg;
      //SetDarkBeam(pXYdata);
      SetLowBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = eolXNeg;
      pXYdata->ydata = eolYNeg;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (x=outX; ((x<=eolXPos) && (x>=outX)); (x+=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = x;
      pXYdata->ydata = eolYNeg;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (j = 0; j < 5; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = eolXPos;
      pXYdata->ydata = eolYNeg;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (y=eolYNeg; ((y<=eolYPos) && (y>=eolYNeg)); (y+=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = eolXPos;
      pXYdata->ydata = y;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (j = 0; j < 5; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = eolXPos;
      pXYdata->ydata = eolYPos;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (x = eolXPos; ((x>=eolXNeg) && (x<=eolXPos)); (x-=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = x;
      pXYdata->ydata = eolYPos;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (j = 0; j < 5; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = eolXNeg;
      pXYdata->ydata = eolYPos;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (y=eolYPos; ((y>=eolYNeg) && (y<=eolYPos)); (y-=64*gCoarse3SearchStep))
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = eolXNeg;
      pXYdata->ydata = y;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = eolXNeg;
      pXYdata->ydata = eolYNeg;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = eolXNeg;
      pXYdata->ydata = eolYNeg;
      //SetDarkBeam(pXYdata);
      SetLowBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  return;
}

void show_move(int32_t lastX, int32_t lastY,
                int32_t currentX, int32_t currentY,
                char * tmpPtr, uint32_t *pIndex)
{
    int16_t cur_xpos;
    int16_t cur_ypos;
    int16_t last_xpos;
    int16_t last_ypos;

    last_xpos = lastX & kMaxUnsigned;
    last_ypos = lastY & kMaxUnsigned;
    cur_xpos = currentX & kMaxUnsigned;
    cur_ypos = currentY & kMaxUnsigned;
    draw_dark(last_xpos, last_ypos, cur_xpos, cur_ypos, tmpPtr, pIndex);
    return;
}

static void Do1(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex)
{
  int16_t XNeg;
  int16_t XPos;
  int16_t YNeg;
  int16_t YPos;
  int16_t cur_xpos;

  limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);
#ifdef AGS_DEBUG
  syslog(LOG_DEBUG, "curx %x,cury %x, xneg %x,xpos %x,yneg %x,ypos %x",currentX,currentY,XNeg,XPos,YNeg,YPos);
#endif
  cur_xpos = currentX & kMaxUnsigned;
  Draw1(cur_xpos, YNeg, YPos, tmpPtr, pIndex );
  return;
}
static void Do2(int32_t currentX, int32_t currentY, char *tmpPtr, uint32_t *pIndex)
{
  int16_t XNeg;
  int16_t XPos;
  int16_t YNeg;
  int16_t YPos;
  int16_t cur_ypos;

  cur_ypos = currentY & kMaxUnsigned;
  limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);
  Draw2(XNeg,XPos,YNeg,cur_ypos,YPos, tmpPtr, pIndex);
  return;
}

static void Do3(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;
    int16_t cur_ypos;

    cur_ypos = currentY & kMaxUnsigned;
    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);
    Draw3(XNeg,XPos,YNeg,cur_ypos,YPos, tmpPtr, pIndex);
    return;
}


static void Do4(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;
    int16_t cur_ypos;

    cur_ypos = currentY & kMaxUnsigned;
    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);
    Draw4(XNeg,XPos,YNeg,cur_ypos,YPos, tmpPtr, pIndex);
    return;
}


static void Do5(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;
    int16_t cur_ypos;

    cur_ypos = currentY & kMaxUnsigned;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);
    Draw5(XNeg,XPos,YNeg,cur_ypos,YPos, tmpPtr, pIndex);
    return;
}

static void Do6(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;
    int16_t cur_ypos;

    cur_ypos = currentY & kMaxUnsigned;
    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);
    Draw6( XNeg,XPos,YNeg,cur_ypos,YPos, tmpPtr, pIndex );
    return;
}
static void Do7(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;

    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);
    Draw7( XNeg,XPos,YNeg,YPos, tmpPtr, pIndex );
    return;
}
static void Do8(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;
    int16_t cur_ypos;
    
    cur_ypos = currentY & kMaxUnsigned;
    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);
    Draw8( XNeg,XPos,YNeg,cur_ypos,YPos, tmpPtr, pIndex );
    return;
}
static void Do9(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;
    int16_t cur_ypos;
    
    cur_ypos = currentY & kMaxUnsigned;
    limitXY(currentX,currentY,&XNeg,&XPos,&YNeg,&YPos);
    Draw9(XNeg,XPos,YNeg,cur_ypos,YPos, tmpPtr, pIndex);
    return;
}
static void Do10(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t X001;
    int16_t X002;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);

    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw0(X002,XPos,YNeg,YPos,tmpPtr,pIndex);
    return;
}


static void Do11(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t X001;
    int16_t X002;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw1(XPos,YNeg,YPos,tmpPtr,pIndex);
    return;
}


static void Do12(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t X001;
    int16_t X002;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;
    int16_t cur_ypos;
    
    cur_ypos = currentY & kMaxUnsigned;
    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw2(X002,XPos,YNeg,cur_ypos,YPos,tmpPtr,pIndex);
    return;
}


static void Do13(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t X001;
    int16_t X002;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;
    int16_t cur_ypos;
    
    cur_ypos = currentY & kMaxUnsigned;
    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw3(X002,XPos,YNeg,cur_ypos,YPos,tmpPtr,pIndex);
    return;
}

static void Do14(int32_t currentX, int32_t currentY,
                char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t X001;
    int16_t X002;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;
    int16_t cur_ypos;
    
    cur_ypos = currentY & kMaxUnsigned;
    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw4(X002,XPos,YNeg,cur_ypos,YPos,tmpPtr,pIndex);
    return;
}
static void Do15(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t X001;
    int16_t X002;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;
    int16_t cur_ypos;
    
    cur_ypos = currentY;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw5(X002,XPos,YNeg,cur_ypos,YPos,tmpPtr,pIndex);
    return;
}
static void Do16(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t X001;
    int16_t X002;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;
    int16_t cur_ypos;
    
    cur_ypos = currentY & kMaxUnsigned;
    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw6(X002,XPos,YNeg,cur_ypos,YPos,tmpPtr,pIndex);
    return;
}

static void Do17(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t X001;
    int16_t X002;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;

    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw7(X002,XPos,YNeg,YPos,tmpPtr,pIndex);
    return;
}


static void Do18(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t X001;
    int16_t X002;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;
    int16_t cur_ypos;
    
    cur_ypos = currentY & kMaxUnsigned;
    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw8(X002,XPos,YNeg,cur_ypos,YPos,tmpPtr,pIndex);
    return;
}

static void Do19(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t X001;
    int16_t X002;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;
    int16_t cur_ypos;
    
    cur_ypos = currentY & kMaxUnsigned;
    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw1(XNeg,YNeg,YPos,tmpPtr,pIndex);
    Draw9(X002,XPos,YNeg,cur_ypos,YPos,tmpPtr,pIndex);
    return;
}

static void Do20(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t X001;
    int16_t X002;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;
    int16_t cur_ypos;
    
    cur_ypos = currentY & kMaxUnsigned;
    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw2(XNeg,X001,YNeg,cur_ypos,YPos,tmpPtr,pIndex);
    Draw0(X002,XPos,YNeg,YPos,tmpPtr,pIndex);
    return;
}


static void Do21(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t X001;
    int16_t X002;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;
    int16_t cur_ypos;
    
    cur_ypos = currentY & kMaxUnsigned;
    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw2(XNeg,X001,YNeg,cur_ypos, YPos,tmpPtr,pIndex);
    Draw1(XPos,YNeg,YPos,tmpPtr,pIndex);
    return;
}


static void Do22(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t X001;
    int16_t X002;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;
    int16_t cur_ypos;
    
    cur_ypos = currentY & kMaxUnsigned;
    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw2(XNeg,X001,YNeg,cur_ypos,YPos,tmpPtr,pIndex);
    Draw2(X002,XPos,YNeg,cur_ypos,YPos,tmpPtr,pIndex);
    return;
}
static void Do23(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t X001;
    int16_t X002;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;
    int16_t cur_ypos;
    
    cur_ypos = currentY & kMaxUnsigned;
    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw2(XNeg,X001,YNeg,cur_ypos,YPos,tmpPtr,pIndex);
    Draw3(X002,XPos,YNeg,cur_ypos,YPos,tmpPtr,pIndex);
    return;
}

static void Do24(int32_t currentX, int32_t currentY,
		 char *tmpPtr, uint32_t *pIndex)
{
    int16_t XNeg;
    int16_t X001;
    int16_t X002;
    int16_t XPos;
    int16_t YNeg;
    int16_t YPos;
    int16_t cur_ypos;
    
    cur_ypos = currentY & kMaxUnsigned;
    limit2C(currentX,currentY,&XNeg,&X001,&X002,&XPos,&YNeg,&YPos);
    Draw2(XNeg,X001,YNeg,cur_ypos,YPos,tmpPtr,pIndex);
    Draw4(X002,XPos,YNeg,cur_ypos,YPos,tmpPtr,pIndex);
    return;
}

static void Draw0(int16_t xmin, int16_t xmax, int16_t ymin,
		  int16_t ymax, char *tmpPtr, uint32_t *pIndex)
{
    off_pause(xmin, ymin, tmpPtr, pIndex );
    draw_line(xmin, ymin, xmin, ymax, tmpPtr, pIndex );
    draw_line(xmin, ymax, xmax, ymax, tmpPtr, pIndex );
    draw_line(xmax, ymax, xmax, ymin, tmpPtr, pIndex );
    draw_line(xmax, ymin, xmin, ymin, tmpPtr, pIndex );
    off_pause(xmin, ymin, tmpPtr, pIndex );
    return;
}

static void Draw1(int16_t x0, int16_t ymin, int16_t ymax,
		  char *tmpPtr, uint32_t *pIndex)
{
    off_pause(x0, ymin, tmpPtr, pIndex);
    draw_line(x0, ymin, x0, ymax, tmpPtr, pIndex );
    off_pause(x0, ymin, tmpPtr, pIndex);
    return;
}

static void Draw2(int16_t xmin, int16_t xmax, int16_t ymin,
		  int16_t y0, int16_t ymax, char *tmpPtr,
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

static void Draw3(int16_t xmin, int16_t xmax, int16_t ymin,
		  int16_t y0, int16_t ymax, char *tmpPtr,
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

static void Draw4(int16_t xmin, int16_t xmax, int16_t ymin,
		  int16_t y0, int16_t ymax, char *tmpPtr,
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

static void Draw5(int16_t xmin, int16_t xmax, int16_t ymin,
		  int16_t y0, int16_t ymax, char *tmpPtr,
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

static void Draw6(int16_t xmin, int16_t xmax, int16_t ymin,
		  int16_t y0, int16_t ymax, char *tmpPtr,
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

static void Draw7(int16_t xmin, int16_t xmax, int16_t ymin,
		  int16_t ymax, char *tmpPtr, uint32_t *pIndex)
{
    off_pause(  xmin, ymin,             tmpPtr, pIndex );
    draw_line(  xmin, ymin, xmax, ymax, tmpPtr, pIndex );
    draw_line(  xmax, ymax, xmin, ymax, tmpPtr, pIndex );
    off_pause(  xmin, ymax,             tmpPtr, pIndex );
    return;
}
static void Draw8(int16_t xmin, int16_t xmax, int16_t ymin,
		  int16_t y0, int16_t ymax, char *tmpPtr,
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
static void Draw9(int16_t xmin, int16_t xmax, int16_t ymin,
		  int16_t y0, int16_t ymax, char *tmpPtr,
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

void draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
	       char *tmpPtr, uint32_t *pIndex)
{
  struct lg_xydata *pXYdata;
  int16_t delX;
  int16_t delY;
  uint32_t j;

  delX = (x1 - x0) / 16;
  delY = (y1 - y0) / 16;

#ifdef AGS_DEBUG
  syslog(LOG_DEBUG,"DRAWLINE: x0 %x, y0 %x, x1 %x, y1 %x delx %x,dely %x",x0,y0,x1,y1,delX,delY);
#endif
  // Make sure XY is write-ready for laser device
  // pause at the start of line
  pXYdata = (struct lg_xydata *)tmpPtr;
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = x0;
      pXYdata->ydata = y0;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
  }
  for (j=0; j <= 16; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = x0 + (j * delX);
      pXYdata->ydata = y0 + (j * delY);
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  // pause at the end of line
  for ( j = 0; j < 2; j++ )
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = x1;
      pXYdata->ydata = y1;
      SetHighBeam(pXYdata);
      *pIndex += sizeof(struct lg_xydata); 
    }
  return;
}

static void draw_dark(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
		      char *tmpPtr, uint32_t *pIndex)
{
  struct lg_xydata *pXYdata;
  int16_t delX;
  int16_t delY;
  uint32_t j;

  delX = (x1 - x0) / 60;
  delY = (y1 - y0) / 60;

#ifdef AGS_DEBUG
  syslog(LOG_DEBUG,"DRAWDARK: x0 %x, y0 %x, x1 %x, y1 %x",x0,y0,x1,y1);
#endif
  pXYdata = (struct lg_xydata *)tmpPtr;
  // pause at the start of line
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = x0;
      pXYdata->ydata = y0;
      *pIndex += sizeof(struct lg_xydata); 
    }
  for (j=0; j <= 60; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = x0 + (j*delX);
      pXYdata->ydata = y0 + (j*delY);
      *pIndex += sizeof(struct lg_xydata); 
    }
  // pause at the end of line
  for (j = 0; j < 2; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = x1;
      pXYdata->ydata = y1;
      *pIndex += sizeof(struct lg_xydata); 
    }
  return;
}

static void off_pause(int16_t x1, int16_t y1, char *tmpPtr, uint32_t *pIndex)
{
  struct lg_xydata *pXYdata;
  uint32_t j;

  if (!tmpPtr)
    return;
#ifdef AGS_DEBUG
  syslog(LOG_DEBUG,"PAUSE: x1 %x, y1 %x",x1,y1);
#endif
  pXYdata = (struct lg_xydata *)tmpPtr;
  for (j = 0; j < 10; j++)
    {
      pXYdata = (struct lg_xydata *)((char *)pXYdata + *pIndex + (j*sizeof(struct lg_xydata)));
      pXYdata->xdata = x1;
      pXYdata->ydata = y1;
    }

  *pIndex += 10 * sizeof(struct lg_xydata); 
  return;
}

static void limitXY(int32_t currentX, int32_t currentY,
		    int16_t *eolXNeg, int16_t *eolXPos,
		    int16_t *eolYNeg, int16_t * eolYPos)
{
    
    if (currentX >= (kMaxSigned + gNumberStep))
      *eolXPos   =  kMaxSigned;
    else
      *eolXPos =  currentX + gNumberStep; 
    if (currentX <= (kMinSigned - gNumberStep))
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
		    int16_t *eolXNeg, int16_t *eolX001,
		    int16_t *eolX002, int16_t *eolXPos,
		    int16_t *eolYNeg, int16_t * eolYPos)
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
