#include <stdint.h>
//static char rcsid[] = "$Id: BoardComm.c,v 1.36 2007/03/30 20:13:58 pickle Exp pickle $";

#include <fcntl.h>
#include <stdio.h>
#include <stddef.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <sys/io.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <sys/ioctl.h>
#include <linux/laser_ioctl.h>
#include "AppCommon.h"
#include "BoardCommDefines.h"
#include "AppErrors.h"
#include "AppStrListIDs.h"
#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "laserguide.h"
#include "L3DTransform.h"
#include "3DTransform.h"
#include "SensorSearch.h"
#include "SensorRegistration.h"
#include "LaserInterface.h"
#include "Video.h"
#include "Init.h"
#include "ROI.h"
#include "Net.h"
#include "Hobbs.h"
#include "QuickCheckManager.h"

static time_t start_QC_timer;
static time_t end_QC_timer;
static char   *tmp_pattern=0;
static char   *out_pattern=0;
static int    gLedState;

uint32_t  gRespondToWhom = kDoNotRespond;

static  int32_t     gQuickCheckCounterStart = -1;

static  uint32_t  gSaveX = 0;
static  uint32_t  gSaveY = 0;

enum
{
  kInitializingBoardMsg = 1,
  kInitUnrecognizedMemModeMsg,
  kInitBoardNotFoundMsg,
  kInitOBCodeNotFound,
  kInitCannotLoadCode,
  kInitCannotResetBoard,
  kInitCannotFindSerialPack
};

enum
{
  kGSOffLineMsg = 1,
  kGSInitializingMsg,
  kGSOnLineMsg
};

int gStopFlag;
int gSearchFlag;
extern void ClearStopFlag ( void ) ;
extern int CheckStopFlag ( void ) ;
 
void ClearStopFlag ( void ) { gStopFlag = 0; }
int CheckStopFlag ( void ) { return gStopFlag; }
static int move_lite(struct lg_master *pLgMaster, int32_t new_xpos, int32_t new_ypos);
static  void  DoRespond (struct lg_master *pLgMaster, struct k_header *pHdr);
static void ClearVideoSwitch(struct lg_master *pLgMaster);
static void RestoreBeamPosition(struct lg_master *pLgMaster);
static void SetDisplayClock(struct lg_master *pLgMaster);
static int doSetQCFlag(struct lg_master *pLgMaster, uint32_t qcflag);
static int doSetHWTrigger(struct lg_master *pLgMaster);
static int doSensor(struct lg_master *pLgMaster);
static int doDarkSensor(struct lg_master *pLgMaster);
 
void PostCmdDisplay(struct lg_master *pLgMaster, struct displayData *p_dispdata, uint32_t respondToWhom)
{
  double         n, dx, dy, dsqr, dlen;
  char           *wr_ptr;
  uint32_t       *out32;
  uint32_t       *tmp32;
  struct k_header pRespHdr;
  int32_t        Npoints;
  uint32_t       xout;
  uint32_t       yout;
  uint32_t       ptn_len;
  int            last_xpos;
  int            last_ypos;
  int            cur_xpos;
  int            cur_ypos;
  int            xstep, ystep;
  int            i, j, k, count;
  int            first_xpos;
  int            first_ypos;

  memset((char *)&pRespHdr, 0, sizeof(struct k_header));
  memset((char *)pLgMaster->theResponseBuffer, 0, sizeof(struct parse_basic_resp));
  if (!p_dispdata)
    return;

  wr_ptr = (char *)&p_dispdata->sensorAngles[0];
  ptn_len = pLgMaster->patternLength;
  if (!wr_ptr || !ptn_len || (ptn_len > MAX_DATA))
    {
      ROIoff(pLgMaster);
      initQCcounter(pLgMaster);
      if (!wr_ptr || !ptn_len)
	{
	  pRespHdr.status = RESPGOOD;
	  DoRespond (pLgMaster, (struct k_header *)&pRespHdr);
	}
      else
	{
	  pRespHdr.status = RESPFAIL;
	  DoRespond (pLgMaster, (struct k_header *)&pRespHdr);
	}
      return;
    }
  out32 = (uint32_t *)out_pattern;
  tmp32 = (uint32_t *)tmp_pattern;
  if (!out32 || !tmp32)
    return;

  memset(tmp_pattern, 0, MAX_LG_BUFFER);
  memset(out_pattern, 0, MAX_LG_BUFFER);

  ROIoff(pLgMaster);
  setROIlength(pLgMaster, 0);
  if ( gVideoCheck == 0 )
    initQCcounter(pLgMaster);

  memcpy (tmp_pattern, wr_ptr, ptn_len);
  cur_xpos = tmp32[0];
  cur_ypos = tmp32[1];

  move_dark(pLgMaster, cur_xpos, cur_ypos );
  last_xpos = tmp32[0];
  last_ypos = tmp32[1];
  first_xpos = tmp32[0];
  first_ypos = tmp32[1];
  j = 0;
  count = 0;
  Npoints = ptn_len / 8;
  for (i=0; i < Npoints; i++)
    {
      cur_xpos = tmp32[2*i+0];
      cur_ypos = tmp32[2*i+1];
      out32[2*j+0] = last_xpos;
      out32[2*j+1] = last_ypos;
      j++;
      count++;
      dx = (double)cur_xpos - (double)last_xpos;
      dy = (double)cur_ypos - (double)last_ypos;
      dsqr = dx*dx + dy*dy;
      dlen = sqrt( dsqr );
      if (dlen > pLgMaster->dmax)
	{
	  n = (int)( (dlen / pLgMaster->dmax) + 0.001 );
	  n += 2.0;
	  xstep =  (int)(dx / n) & kMaxUnsigned;
	  ystep =  (int)(dy / n) & kMaxUnsigned;
	  for (k = 0; k < n ; k++)
	    {
	      xout = last_xpos + k * (int)xstep;
	      yout = last_ypos + k * (int)ystep;
	      out32[2*j+0] = xout;
	      out32[2*j+1] = yout;
	      j++;
	      count++;
	    }
	}
      last_xpos = cur_xpos;
      last_ypos = cur_ypos;
    }
  out32[2*j+0] = last_xpos;
  out32[2*j+1] = last_ypos;
  j++;
  count++;
  dx = first_xpos - last_xpos;
  dy = first_ypos - last_ypos;
  dsqr = dx*dx + dy*dy;
  if ( dsqr > (pLgMaster->dmax * pLgMaster->dmax))
    {
      n = (int) (sqrt((double)(dsqr / pLgMaster->dmax)));
    n++; n++;
    xstep =  (int)(dx / n) & kMaxUnsigned;
    ystep =  (int)(dy / n) & kMaxUnsigned;
    for ( k = 0; k < n ; k++ ) {
      xout = last_xpos + k * xstep;
      yout = last_ypos + k * ystep;
      out32[2*j+0] = xout;
      out32[2*j+1] = yout;
      j++;
      count++;
    }
  }
  Npoints  = count;
  ptn_len = 8 * Npoints;

  memcpy(wr_ptr, (char *)out_pattern, ptn_len);
  SaveBeamPosition(wr_ptr);
  write( pLgMaster->fd_laser, wr_ptr, ptn_len);
  doLoadWriteNum(pLgMaster, Npoints);
  if ( gVideoCheck == 0 )
    SaveAnglesForQuickCheck(p_dispdata, kRespondExtern );
  SetDisplayClock(pLgMaster);
  StartHobbs(pLgMaster);
  pLgMaster->gDisplayFlag = 1;
  doDevDisplay(pLgMaster);
  // reset display period to the default value
  pLgMaster->gPeriod = GPERIOD_DEFAULT;

  pRespHdr.status = RESPGOOD;
  if (pLgMaster->gOutOfRange.errtype1 || pLgMaster->gOutOfRange.errtype2)
    {
      pRespHdr.status = RESPFAIL;
      pRespHdr.errtype1 = pLgMaster->gOutOfRange.errtype1;
      pRespHdr.errtype2 = pLgMaster->gOutOfRange.errtype2;
    }
  DoRespond (pLgMaster, (struct k_header *)&pRespHdr);
  return;
}
void PostCmdDispNoResp(struct lg_master *pLgMaster, struct displayData *p_dispdata, uint32_t respondToWhom)
{
  double          n, dx, dy, dsqr, dlen;
  char            *wr_ptr;
  uint32_t        *out32;
  uint32_t        *tmp32;
  struct k_header pRespHdr;
  int32_t         Npoints;
  uint32_t        xout;
  uint32_t        yout;
  uint32_t        ptn_len;
  int             xstep, ystep;
  int             last_xpos;
  int             last_ypos;
  int             cur_xpos;
  int             cur_ypos;
  int             i, j, k, count;
  int             first_xpos;
  int             first_ypos;

  if (!p_dispdata || !pLgMaster)
    return;

  wr_ptr = (char *)&p_dispdata->sensorAngles[0];
  ptn_len = pLgMaster->patternLength;
  if (!wr_ptr || !ptn_len || (ptn_len > MAX_DATA))
    {
#ifdef PATDEBUG
      fprintf(stderr,"\nPOSTCMD:  NORESP, Turn off ROI, InitQC");
#endif
      ROIoff(pLgMaster);
      initQCcounter(pLgMaster);
      if (!wr_ptr || !ptn_len)
	{
	  pRespHdr.status = RESPGOOD;
	  DoRespond (pLgMaster, (struct k_header *)&pRespHdr);
	}
      else
	{
	  pRespHdr.status = RESPFAIL;
	  fprintf(stderr,"\nKDISP-WRSP:  BAD LENGTH %d", ptn_len);
	  DoRespond (pLgMaster, (struct k_header *)&pRespHdr);
	}
      return;
    }
  out32 = (uint32_t *)out_pattern;
  tmp32 = (uint32_t *)tmp_pattern;
  if (!out32 || !tmp32)
    return;
  memset((char *)&pRespHdr, 0, sizeof(struct k_header)); 
  memset((char *)pLgMaster->theResponseBuffer, 0, sizeof(struct parse_basic_resp));
  memset(out_pattern, 0, MAX_LG_BUFFER);
  memset(tmp_pattern, 0, MAX_LG_BUFFER);

#ifdef PATDEBUG
  fprintf(stderr,"\nPOSTCMD:  NORESP, Set ROI length");
#endif
  ROIoff(pLgMaster);
  setROIlength(pLgMaster, 0);
  if (gVideoCheck == 0)
    initQCcounter(pLgMaster);

  memcpy ((char *)&tmp32[0], wr_ptr, ptn_len);
  cur_xpos = tmp32[0];
  cur_ypos = tmp32[1];
#ifdef PATDEBUG
  fprintf(stderr,"\nPOSTCMD:  NORESP, move-dark xpos %d, ypos %d", cur_xpos, cur_ypos);
#endif

  move_dark(pLgMaster, cur_xpos, cur_ypos );

  last_xpos = tmp32[0];
  last_ypos = tmp32[1];
  first_xpos = tmp32[0];
  first_ypos = tmp32[1];
  j = 0;
  count = 0;
  Npoints = ptn_len / 8;
  for (i=0; i < Npoints; i++)
    {
      cur_xpos = tmp32[2*i+0];
      cur_ypos = tmp32[2*i+1];
      out32[2*j+0] = last_xpos;
      out32[2*j+1] = last_ypos;
      j++;
      count++;
      dx = (double)cur_xpos - (double)last_xpos;
      dy = (double)cur_ypos - (double)last_ypos;
      dsqr = dx*dx + dy*dy;
      dlen = sqrt( dsqr );
      if (dlen > pLgMaster->dmax)
	{
	  n = (int)(dlen / pLgMaster->dmax );
	  n += 2.0;
	  xstep =  (int)(dx / n) & kMaxUnsigned;
	  ystep =  (int)(dy / n) & kMaxUnsigned;
	  for (k = 0; k < n ; k++)
	    {
	      xout = last_xpos + k * (int)xstep;
	      yout = last_ypos + k * (int)ystep;
	      out32[2*j+0] = xout;
	      out32[2*j+1] = yout;
	      j++;
	      count++;
	    }
	}
      last_xpos = cur_xpos;
      last_ypos = cur_ypos;
    }
  out32[2*j+0] = last_xpos;
  out32[2*j+1] = last_ypos;
  j++;
  count++;
  dx = first_xpos - last_xpos;
  dy = first_ypos - last_ypos;
  dsqr = dx*dx + dy*dy;
  if (dsqr > (pLgMaster->dmax * pLgMaster->dmax))
    {
      n = (int) (sqrt((double)(dsqr / pLgMaster->dmax)));
      n++; n++;
      xstep =  (int)(dx / n) & kMaxUnsigned;
      ystep =  (int)(dy / n) & kMaxUnsigned;
      for (k = 0; k < n; k++)
	{
	  xout = last_xpos + k * xstep;
	  yout = last_ypos + k * ystep;
	  out32[2*j+0] = xout;
	  out32[2*j+1] = yout;
	  j++;
	  count++;
	}
    }
  Npoints  = count;
  ptn_len = 8 * Npoints;
  
  memcpy (wr_ptr, (char *)&out32[0], ptn_len);
  SaveBeamPosition(wr_ptr);
#ifdef PATDEBUG
  fprintf(stderr,"\nPOSTCMD:  NORSP START WRITE TO LASER-DEV, patlen %d", ptn_len);
#endif
  // Don't send if too much data for laser-dev to handle
  if (ptn_len > MAX_LGDEV_BUFF)
    {
      pRespHdr.status = RESPFAIL;
      DoRespond(pLgMaster, (struct k_header *)&pRespHdr);
      return;
    }
  write( pLgMaster->fd_laser, wr_ptr, ptn_len);
#ifdef PATDEBUG
  fprintf(stderr,"\nPOSTCMD:  NORSP WRITE TO LASER-DEV SUCCEED, patlen %d, x0 %d, y0 %d", ptn_len, out32[0],out32[1]);
#endif
  doLoadWriteNum(pLgMaster, Npoints);
  if ( gVideoCheck == 0 ) {
    SaveAnglesForQuickCheck(p_dispdata, kRespondExtern );
  }

  SetDisplayClock(pLgMaster);
  StartHobbs(pLgMaster);
  pLgMaster->gDisplayFlag = 1;
  doDevDisplay(pLgMaster);

  pRespHdr.status = RESPGOOD;
  if (pLgMaster->gOutOfRange.errtype1 || pLgMaster->gOutOfRange.errtype2)
    {
      pRespHdr.status = RESPFAIL;
      pRespHdr.errtype1 = pLgMaster->gOutOfRange.errtype1;
      pRespHdr.errtype2 = pLgMaster->gOutOfRange.errtype2;
    }
  DoRespond(pLgMaster, (struct k_header *)&pRespHdr);
#ifdef PATDEBUG
  fprintf(stderr,"\nPOSTCMD:  NORSP DONE");
#endif
  return;
}

void PostCommand(struct lg_master *pLgMaster, uint32_t theCommand, char *data, uint32_t respondToWhom )
{
  struct k_header gResponseBuffer;
  struct displayData *dispData;
  char * wr_ptr;
  uint32_t     *data32;
  uint32_t     *out32;
  uint32_t     *tmp32;
  int32_t      Npoints;
  uint32_t     ptn_len;
  
  memset((char *)&gResponseBuffer, 0, sizeof(gResponseBuffer));
  
  //  assume display is off or being turned off
  pLgMaster->gDisplayFlag = 0;
  gRespondToWhom = respondToWhom;
  
  out32 = (uint32_t *)out_pattern;
  tmp32 = (uint32_t *)tmp_pattern;

  if (!pLgMaster || !out32 || !tmp32)
    {
      fprintf(stderr,"POSTCMD:  BAD POINTER %d",theCommand);
      return;
    }
  switch ( theCommand )
    {
      case kStop:
	// stop, slow down and turn off comm err LED
	doLGSTOP(pLgMaster);
	doSlowDownTimer(pLgMaster);
	doStopPulse(pLgMaster);
	pLgMaster->optic_status |= 0x10;
	outb(pLgMaster->optic_status, LG_IO_OPTIC);
	gLedState = 0;
	ROIoff(pLgMaster);
	g_FODflag = 0;
	gQuickCheckCounterStart = -1;
	stopQCcounter(pLgMaster);   /* never quick check */
	gVideoCount = 0;
	gVideoCheck = 0;
	ClearVideoSwitch(pLgMaster);
	setROIlength(pLgMaster, 0);
	gStopFlag = -1;
	gResponseBuffer.status = RESPSTOPOK;
	EndHobbs(pLgMaster);
	
	if ( !(pLgMaster->gHeaderSpecialByte & 0x80 ) ) {
	  DoRespond (pLgMaster, (struct k_header *)&gResponseBuffer);
	}
#ifdef PATDEBUG
	fprintf(stderr,"\nPOSTCMD:  kSTOP DONE");
#endif	    
        break;
    case kDisplayVideoCheck:
	dispData = (struct displayData *)data;
	wr_ptr = (char *)&dispData->sensorAngles[0];
	if (!wr_ptr)
	  return;
      ptn_len = pLgMaster->patternLength;
      if (ptn_len > MAX_DATA)
	{
	  fprintf(stderr,"\nKDISP-VCHK:  BAD LENGTH %d", ptn_len);
	  return;
	  }
      ROIoff(pLgMaster);
      setROIlength(pLgMaster, 0);
      gQuickCheckCounterStart = -1;
      initQCcounter(pLgMaster);   /* never quick check */
      Npoints = ptn_len / 8;
          
      SaveBeamPosition(wr_ptr);
      write( pLgMaster->fd_laser, wr_ptr, ptn_len);
      doLoadWriteNum(pLgMaster, Npoints);
      
      SetDisplayClock(pLgMaster);
      StartHobbs(pLgMaster);
      pLgMaster->gDisplayFlag = 1;
      doDevDisplay(pLgMaster);

      if (pLgMaster->gOutOfRange.errtype1 || pLgMaster->gOutOfRange.errtype2)
	{
	  gResponseBuffer.status1 = RESPFAIL;
	  gResponseBuffer.errtype1 = pLgMaster->gOutOfRange.errtype1;
	  gResponseBuffer.errtype2 = pLgMaster->gOutOfRange.errtype2;
	}
      else { 
	gResponseBuffer.status = RESPGOOD;
      } 
      DoRespond (pLgMaster, (struct k_header *)&gResponseBuffer);
      break;
      case kGoAngle:
	dispData = (struct displayData *)data;
	data32 = dispData->sensorAngles;
	if (!data32)
	  return;
	doSlowDownTimer(pLgMaster);
	move_lite(pLgMaster, (data32[0] | 0x40), data32[1]);
	doWriteDevPoints(pLgMaster, (data32[0] | 0x40), data32[1]);
	gResponseBuffer.status = RESPGOOD;
	if (!(pLgMaster->gHeaderSpecialByte & 0x80))
	  DoRespond (pLgMaster, (struct k_header *)&gResponseBuffer);
        break;

      case kEtherAngle:
	dispData = (struct displayData *)data;
	data32 = dispData->sensorAngles;
	if (!data32)
	  return;
	gResponseBuffer.status = RESPGOOD;
	doSlowDownTimer(pLgMaster);
	doStopPulse(pLgMaster);
	move_lite(pLgMaster,  (data32[0] | 0x40), data32[1]);
	doWriteDevPoints(pLgMaster, (data32[0] | 0x40), data32[1]);
	DoRespond (pLgMaster, (struct k_header *)&gResponseBuffer);
        break;

      case kDarkAngle:
	dispData = (struct displayData *)data;
	data32 = dispData->sensorAngles;
	if (!data32)
	  return;
	doSlowDownTimer(pLgMaster);
	move_dark(pLgMaster, (data32[0] & 0xFFFFFF00), data32[1]);
	doWriteDevPoints(pLgMaster, (data32[0] & 0xFFFFFF00), data32[1]);
	gResponseBuffer.status = RESPGOOD;
	DoRespond (pLgMaster, (struct k_header *)&gResponseBuffer);
        break;

      case kAngleOffset:
	dispData = (struct displayData *)data;
	data32 = dispData->sensorAngles;
	if (!data32)
	  return;
	doSetXOffset(pLgMaster, (0xFFFF & (data32[0] >> 16)));
	doSetYOffset(pLgMaster, (0xFFFF & (data32[1] >> 16)));
	gResponseBuffer.status = RESPGOOD;
	DoRespond (pLgMaster, (struct k_header *)&gResponseBuffer);
        break;

      case kSegmentDisplay:
	DoRespond (pLgMaster, (struct k_header *)&gResponseBuffer);
        break;
    case kSearchForASensor:
      break;
      case kDisplayNoQuickCheck:
	 if (!data)
	   return;
	dispData = (struct displayData *)data;
	wr_ptr = (char *)&dispData->sensorAngles[0];
	if (!wr_ptr)
	  return;
	ptn_len = pLgMaster->patternLength;
	 if (ptn_len > MAX_DATA)
	   {
	     fprintf(stderr,"\nKDISP-NOQCHK:  BAD LENGTH %d", ptn_len);
	     return;
	   }
	 ROIoff(pLgMaster);
	 setROIlength(pLgMaster, 0);
	 gQuickCheckCounterStart = -1;
	 initQCcounter(pLgMaster);   /* never quick check */
	 Npoints = ptn_len / 8;
          
	 SaveBeamPosition(wr_ptr);
	 write( pLgMaster->fd_laser, wr_ptr, ptn_len);
	 doLoadWriteNum(pLgMaster, Npoints);
	 SetDisplayClock(pLgMaster);
	 StartHobbs(pLgMaster);
	 pLgMaster->gDisplayFlag = 1;
	 doDevDisplay(pLgMaster);

	 if (pLgMaster->gOutOfRange.errtype1 || pLgMaster->gOutOfRange.errtype2)
	   {
	     gResponseBuffer.status1 = RESPFAIL;
	     gResponseBuffer.errtype1 = pLgMaster->gOutOfRange.errtype1;
	     gResponseBuffer.errtype2 = pLgMaster->gOutOfRange.errtype2;
	   }
	 else { 
	   gResponseBuffer.status = RESPGOOD;
	 }
	 DoRespond (pLgMaster, (struct k_header*)&gResponseBuffer);
        break;
      case kQuickCheck:
	PerformAndSendQuickCheck (pLgMaster, data, gRespondToWhom );
        break;
      case kQuickCheckSensor:
        break;
      default:
          break;
    }
  return;
}

static void DoRespond (struct lg_master *pLgMaster, struct k_header *pHdr)
{
  struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;

  memset((char *)pResp, 0, sizeof(struct parse_basic_resp));
  memcpy((char *)pResp, (char *)pHdr, sizeof(struct k_header));
  pResp->hdr.status = pHdr->status1;
  pResp->hdr.errtype1 = pHdr->errtype1;
  pResp->hdr.errtype2 = htons(pHdr->errtype2);
  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), gRespondToWhom);
  return;
}

void ReleaseBoard (struct lg_master *pLgMaster)
{
  if (pLgMaster->fd_laser > 0)
    close( pLgMaster->fd_laser );
  if (tmp_pattern)
    free(tmp_pattern);
  if (out_pattern)
    free(out_pattern);
  return;
}

int InitBoard (struct lg_master *pLgMaster)
{
  
  pLgMaster->fd_laser = open("/dev/laser0", O_RDWR);
  if (pLgMaster->fd_laser < 0)
    {
      fprintf(stderr,"\nlaser device %d not opened,errno %d", pLgMaster->fd_laser, errno);
      perror( "/dev/laser" );
      return(-1);
    }
  if (ioperm(LG_BASE, LASER_REGION, 1))
    {
      perror("ioperm");
      return(-2);
    }
  doLGSTOP(pLgMaster);
  doSlowDownTimer(pLgMaster);
  doStopPulse(pLgMaster);
  gLedState = 0;

  // initialize buffers

  tmp_pattern = malloc(MAX_LG_BUFFER);
  if (!tmp_pattern)
    return(-2);
  out_pattern = malloc(MAX_LG_BUFFER);
  if (!out_pattern)
    return(-3);
  return(0);
}

int DoLineSearch(struct lg_master *pLgMaster,
		 uint32_t x0,
                 uint32_t y0, 
                 uint32_t dx,
                 uint32_t dy,
                 uint32_t n,
                 unsigned char *c_out)
{
    int itest, num;
    int count, blank;

    itest = doWriteDevPoints(pLgMaster, x0, y0);
    if (itest)
      return(itest);
	  
    count = 10000;
    blank = 500;

    pLgMaster->optic_status = inb(LG_IO_OPTIC);
    while (count--  && blank)
      {
	if (pLgMaster->optic_status & kSensorBit )
	  blank = 500;
	else
	  blank--;
      }
    itest = doWriteDevDelta(pLgMaster, dx, dy);
    if (itest)
      return itest;
	    
    itest = doLoadReadNum(pLgMaster, n);
    if (itest)
      return(itest);
    itest = doSetClock(pLgMaster, 40);
    if (itest)
      return itest;

    itest = doSensor(pLgMaster);
    if (itest)
      return itest;

    itest = EAGAIN;
    num = 0;
    while (itest == EAGAIN)
      {
	num = read( pLgMaster->fd_laser, c_out, n);
	if (num < 0)
	  itest = errno;
	else
	  itest = num;
      }
    if ((c_out[0] == 0xA5) && (c_out[1] == 0x5A)
	&& (c_out[2] == 0xA5) && (c_out[3] == 0x5A))
      {
	itest = -1;
	return itest;
      }

    itest = doSlowDownTimer(pLgMaster);
    if (itest)
      return itest;
    return 0;
}


 int DoLevelSearch(struct lg_master *pLgMaster,
		   uint32_t x0, uint32_t y0, uint32_t dx,
		   uint32_t dy, uint32_t n, uint32_t *c_out)
{
  int itest, num;
  uint32_t index;
  unsigned short searchbuff[65536];

  itest = doWriteDevPoints(pLgMaster, x0, y0);
  if (itest && (errno != ENOTTY))
    return itest;
  pLgMaster->optic_status |= 0x40;
  outb(pLgMaster->optic_status, LG_IO_OPTIC);
  itest = doWriteDevDelta(pLgMaster, dx, dy);
  if (itest && (errno != ENOTTY))
    return itest;
  itest = doLoadReadNum(pLgMaster, n);
  if (itest)
    return itest;
  itest = doSetClock(pLgMaster, pLgMaster->gSrchStpPeriod);
  if (itest && (errno != ENOTTY))
    return itest;
  itest = doSensor(pLgMaster);
  if (itest && (errno != ENOTTY))
    return itest;
  itest = EAGAIN;
  num = 0;

  for ( index=0; index < n ; index++ ) {
    searchbuff[index] = 0;
  }

  while (itest == EAGAIN)
    {
      num = read( pLgMaster->fd_laser, (void *)searchbuff, 2*n);
      if (num < 0)
	itest = errno;
      else
	itest = num;
    }
  for (index=0; index < n ; index++)
    c_out[index] = (uint32_t)searchbuff[index];
  if ((searchbuff[0] == 0xA5) && (searchbuff[1] == 0x5A)
      && (searchbuff[2] == 0xA5) && (searchbuff[3] == 0x5A))
    {
      itest = -1;
      return itest;
    }

  itest = doSlowDownTimer(pLgMaster);
  if (itest && (errno != ENOTTY))
    return itest;

  pLgMaster->optic_status |= 0x20;
  outb(pLgMaster->optic_status, LG_IO_OPTIC);
  return 0;
}

int SearchBeamOn(struct lg_master *pLgMaster)
{
  pLgMaster->optic_status |= 0x40;
  outb(pLgMaster->optic_status, LG_IO_OPTIC);
  usleep( 10000 );  /* pause a bit after turning on beam */
  return 0;
}

int SearchBeamOff(struct lg_master *pLgMaster)
{
  int rc=0;

  pLgMaster->optic_status &= 0xBF;
  outb(pLgMaster->optic_status, LG_IO_OPTIC);

  // attempt to turn off beam
  rc = doLGSTOP(pLgMaster);
  return(rc);
}

int doWriteDevCmdNoData(struct lg_master *pLgMaster, uint32_t command)
{
  int    rc=0;

  rc = write(pLgMaster->fd_laser, (char *)&command, sizeof(uint32_t));
  if (rc <=0)
    fprintf(stderr,"\nCMDW-NODATA: cmd %d, ERROR rc%d, errno %d\n", command, rc, errno);
  return(rc);
}
int doLGSTOP(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_STOP));
}
int doROIOff(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_ROIOFF));
}
int doDevDisplay(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_DISPLAY));
}
int doStopPulse(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_STOPPULSE));
}
int doStartPulse(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_STARTPULSE));
}
int doSlowDownTimer(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_SLOWTIMER));
}
int doRestartTimer(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_RSTRTTMR));
}
static int doSetHWTrigger(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_HWTRIGGER));
}
static int doSensor(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_DOSENSOR));
}
static int doDarkSensor(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_DODARKSENS));
}
int doWriteDevCmd32(struct lg_master *pLgMaster, uint32_t command, uint32_t write_val)
{
  int    rc=0;
  struct cmd_rw_base *p_cmd_data;

  p_cmd_data = (struct cmd_rw_base *)malloc(sizeof(struct cmd_rw_base));
  if (!p_cmd_data)
    return(-1);
  
  p_cmd_data->cmd = command;
  p_cmd_data->val32 = write_val;
  p_cmd_data->length = sizeof(uint32_t);
  rc = write(pLgMaster->fd_laser, (char *)p_cmd_data, sizeof(struct cmd_rw_base));
  if (rc <=0)
    fprintf(stderr,"\nCMDW-VAL32: ERROR cmd %d, val %d, rc %d, errno %d\n", command, write_val, rc, errno);
  return(rc);
}
int doWriteDevDelta(struct lg_master *pLgMaster, uint32_t xval, uint32_t yval)
{
  int    rc=0;
  struct cmd_rw_base *p_cmd_data;

  p_cmd_data = (struct cmd_rw_base *)malloc(sizeof(struct cmd_rw_base));
  if (!p_cmd_data)
    return(-1);
  
  p_cmd_data->cmd = CMDW_SETDELTA;
  p_cmd_data->xydel.xdel = xval;
  p_cmd_data->xydel.xdel = xval;
  p_cmd_data->length = sizeof(struct lg_delta);
  rc = write(pLgMaster->fd_laser, (char *)p_cmd_data, sizeof(struct cmd_rw_base));
  if (rc <=0)
    fprintf(stderr,"\nCMDW-SETDELTA: ERROR xval %d, yval %d, rc %d, errno %d\n", xval, yval, rc, errno);
  return(rc);
}
int doWriteDevPoints(struct lg_master *pLgMaster, uint32_t xval, uint32_t yval)
{
  int    rc=0;
  struct cmd_rw_base *p_cmd_data;

  p_cmd_data = (struct cmd_rw_base *)malloc(sizeof(struct cmd_rw_base));
  if (!p_cmd_data)
    return(-1);
  
  p_cmd_data->cmd = CMDW_GOANGLE;
  p_cmd_data->xypos.xpos = xval;
  p_cmd_data->xypos.xpos = xval;
  p_cmd_data->length = sizeof(struct lg_delta);
  rc = write(pLgMaster->fd_laser, (char *)p_cmd_data, sizeof(struct cmd_rw_base));
  if (rc <=0)
    fprintf(stderr,"\nCMDW-GOANGLE: ERROR xval %d, yval %d, rc %d, errno %d\n", xval, yval, rc, errno);
  return(rc);
}
int doLoadWriteNum(struct lg_master *pLgMaster, uint32_t write_count)
{
  return(doWriteDevCmd32(pLgMaster, CMDW_LOADWRTCNT, write_count));
}
int doLoadReadNum(struct lg_master *pLgMaster, uint32_t read_count)
{
  return(doWriteDevCmd32(pLgMaster, CMDW_LOADRDCNT, read_count));
}
int doSetPulseOn(struct lg_master *pLgMaster, uint32_t pulse_on)
{
  return(doWriteDevCmd32(pLgMaster, CMDW_SETPULSEONVAL, pulse_on));
}
int doSetPulseOff(struct lg_master *pLgMaster, uint32_t pulse_off)
{
  return(doWriteDevCmd32(pLgMaster, CMDW_SETPULSEOFFVAL, pulse_off));
}
static int ApplyQCCounter(struct lg_master *pLgMaster, uint32_t qcCount)
{
  return(doWriteDevCmd32(pLgMaster, CMDW_SETQCCOUNTER, qcCount));
}
static int doSetQCFlag(struct lg_master *pLgMaster, uint32_t qcflag)
{
  return(doWriteDevCmd32(pLgMaster, CMDW_SETQCFLAG, qcflag));
}
int doLoadThreshold(struct lg_master *pLgMaster, uint32_t threshold)
{
  return(doWriteDevCmd32(pLgMaster, CMDW_LOADTHRESH, threshold));
}
int doSetClock(struct lg_master *pLgMaster, uint32_t clock_rate)
{
  return(doWriteDevCmd32(pLgMaster, CMDW_SETCLOCK, clock_rate));
}
int doSetXOffset(struct lg_master *pLgMaster, uint32_t xoff)
{
  return(doWriteDevCmd32(pLgMaster, CMDW_SETXOFFSET, xoff));
}
int doSetYOffset(struct lg_master *pLgMaster, uint32_t yoff)
{
  return(doWriteDevCmd32(pLgMaster, CMDW_SETYOFFSET, yoff));
}
int initQCcounter(struct lg_master *pLgMaster)
{
  gQuickCheckCounterStart = pLgMaster->gQCcount;
  return(ApplyQCCounter(pLgMaster, gQuickCheckCounterStart));
}
int resetQCcounter(struct lg_master *pLgMaster)
{
  return(ApplyQCCounter(pLgMaster, gQuickCheckCounterStart));
}
int SetQCcounter(struct lg_master *pLgMaster, int count)
{
  int   itest;
  
  itest = ApplyQCCounter(pLgMaster, count);
#if defined(KITDEBUG)
  fprintf( stderr, "SetCQ qcCounter %d\n", count);
#endif
  if (itest)
    return(itest);
  return 0;
}
int stopQCcounter(struct lg_master *pLgMaster)
{
  int       itest;
  uint32_t  qcflag=0;
  uint32_t  qc_value = -1;
  
  itest = (ApplyQCCounter(pLgMaster, qc_value));
  if (itest)
    return itest;

  itest = doSetQCFlag(pLgMaster, qcflag);
  if (itest)
    return itest;
  return 0;
}
int32_t GetQCflag(struct lg_master *pLgMaster)
{
  int itest;
  struct lg_info  loc_info;

  memset((char *)&loc_info, 0, sizeof(loc_info));
  itest = ioctl( pLgMaster->fd_laser, LGGETQCFLAG, &loc_info );
  if( itest ) { return -1; }
	 
  return  loc_info.qcFlag;
}

int32_t InitQCtimer(void)
{
  start_QC_timer = time((time_t *)NULL );
  end_QC_timer = time((time_t *)NULL );

  return( 0 );
}

int32_t GetQCtimer( void )
{
            int32_t diff;

            end_QC_timer = time((time_t *)NULL );

            diff = (int32_t)(end_QC_timer - start_QC_timer);
#ifdef ZDEBUG
fprintf( stderr
       , "BC1525 QCtime %d %d diff %d\n"
       , end_QC_timer 
       ,  start_QC_timer
       , diff
       );
#endif
            return( diff );
}

int32_t GetQCcounter(struct lg_master *pLgMaster)
{
  int itest;
  struct lg_info  loc_info;

  memset((char *)&loc_info, 0, sizeof(loc_info));
  itest = ioctl( pLgMaster->fd_laser, LGGETQCCOUNTER, &loc_info );
  if( itest ) { return -1; }

#ifdef ZDEBUG
  fprintf( stderr, "GetQCcounter  %d\n", loc_info.qcCounter );
#endif
  return  loc_info.qcCounter;
}


void OnHWflag(struct lg_master *pLgMaster)
{
  doRestartTimer(pLgMaster);
}

void OffHWflag(struct lg_master *pLgMaster)
{
  doRestartTimer(pLgMaster);
}
void ResumeDisplay(struct lg_master *pLgMaster)
{

  RestoreBeamPosition(pLgMaster);
  SetDisplayClock(pLgMaster);
  StartHobbs(pLgMaster);
  pLgMaster->gDisplayFlag = 1;
  doDevDisplay(pLgMaster);
}

void SetDisplayClock(struct lg_master *pLgMaster)
{
  doSetClock(pLgMaster, pLgMaster->gPeriod);
  return;
}

/*  intended to save initial position */
void SaveBeamPosition( char * data )
{
  uint32_t *data32 = (uint32_t *)&data[0];

  gSaveX = data32[0] & 0xFFFFFF00;
  gSaveY = data32[1];
  return;
}

void RestoreBeamPosition(struct lg_master *pLgMaster)
{
  /* mask out least sig. byte of X */
  move_dark(pLgMaster, gSaveX, gSaveY );
  doWriteDevPoints(pLgMaster, (gSaveX & 0xFFFFFF00), gSaveY);
  usleep( 10000 );
  return;
}

 void GoToRaw(struct lg_master *pLgMaster, uint32_t *x, uint32_t *y )
{
  doSlowDownTimer(pLgMaster);
  move_dark(pLgMaster, *x, *y);
  doWriteDevPoints(pLgMaster, *x, *y);
  return;
}

void GoToPulse(struct lg_master *pLgMaster,
                 uint32_t *x,
                 uint32_t *y,
                 int32_t pulseoffvalue,
                 int32_t pulseonvalue
              )
{
    doSetClock(pLgMaster, 50);
    doStopPulse(pLgMaster);
    move_lite(pLgMaster, *x, *y);
    doWriteDevPoints(pLgMaster, *x, *y);
    doSetPulseOff(pLgMaster, pulseoffvalue);
    doSetPulseOn(pLgMaster, pulseonvalue);
    doStartPulse(pLgMaster);
    return;
}

void StopPulse (struct lg_master *pLgMaster)
{
    doStopPulse(pLgMaster);
}

int ROIoff (struct lg_master *pLgMaster)
{
  int rc=0;
  
  SearchBeamOff(pLgMaster);
  rc = doROIOff(pLgMaster);
  usleep( 250000L );
  return(rc);
}
int setROIlength(struct lg_master *pLgMaster, int32_t half_pattern )
{
  return(doWriteDevCmd32(pLgMaster, CMDW_SETROI, half_pattern));
}
void FlashLed(struct lg_master *pLgMaster, int numFlash )
{
  int i;

  for (i = 0; i < numFlash; i++)
    {
      usleep( 250000U );
      pLgMaster->optic_status &= 0xEF;
      outb(pLgMaster->optic_status, LG_IO_OPTIC);
      usleep( 250000U );
      pLgMaster->optic_status |= 0x10;
      outb(pLgMaster->optic_status, LG_IO_OPTIC);
    }
  gLedState = 0;
  return;
}

void LedBoot(struct lg_master *pLgMaster)
{
  usleep( 250000U );
  pLgMaster->optic_status &= 0xEF;
  outb(pLgMaster->optic_status, LG_IO_OPTIC);
}

void SetHWtrigger(struct lg_master *pLgMaster)
{
  doSetHWTrigger(pLgMaster);
}

static void ClearVideoSwitch(struct lg_master *pLgMaster)
{
  pLgMaster->optic_status &= 0xDF;
  outb(pLgMaster->optic_status, LG_IO_OPTIC);
}

void ClearLinkLED(struct lg_master *pLgMaster)
{
  fprintf( stderr, " about to Clear Link LED\n" );
  pLgMaster->optic_status &= 0xFB;
  outb(pLgMaster->optic_status, LG_IO_OPTIC);
}

void SlowDownAndStop(struct lg_master *pLgMaster)
{
  // stop display and slow down the clock
  // check and update hobbs meters, if necessary
  doLGSTOP(pLgMaster);
  doSlowDownTimer(pLgMaster);
  EndHobbs(pLgMaster);
  return;
}

void ZeroLGoffset(struct lg_master *pLgMaster)
{
  /* first reset offset positions */
  doSetXOffset(pLgMaster, 0);
  doSetYOffset(pLgMaster, 0);
}

void JustDoDisplay(struct lg_master *pLgMaster, char * wr_ptr, int pattern_len)
{
  int Npoints;
  struct lg_info  loc_info;
  double n, dx, dy, dsqr, dlen;
  int i,j,k;
  int32_t last_xpos;
  int32_t last_ypos;
  int32_t cur_xpos;
  int32_t cur_ypos;
  int32_t first_xpos;
  int32_t first_ypos;
  int     count;
  int     xstep, ystep;
  uint32_t xout;
  uint32_t yout;
  uint32_t ptn_len;
  uint32_t     *tmp32;
  uint32_t     *out32;

  tmp32 = (uint32_t *)tmp_pattern;
  out32 = (uint32_t *)out_pattern;

  if (!tmp32 || !out32)
    return;

  memset((char *)&loc_info, 0, sizeof(loc_info));
  ptn_len = pattern_len;
  
#ifdef PATDEBUG
  fprintf(stderr,"\nJUSTDODISP:  Got here");
#endif	 
#ifdef  LASER_DEFINED
  doLGSTOP(pLgMaster);
  doSlowDownTimer(pLgMaster);
  doStopPulse(pLgMaster);
  usleep( 10000 );
  ioctl( pLgMaster->fd_laser, LGGETANGLE, &loc_info );
#endif
  ROIoff(pLgMaster);
  setROIlength(pLgMaster, 0);
  stopQCcounter(pLgMaster);   /* never quick check */

//   fault avoidance
  Npoints = ptn_len / 8;
  memcpy ((char *)tmp_pattern, wr_ptr, ptn_len);
  cur_xpos = tmp32[0];
  cur_ypos = tmp32[1];
  move_dark(pLgMaster, cur_xpos, cur_ypos );
  last_xpos = tmp32[0];
  last_ypos = tmp32[1];
  first_xpos = tmp32[0];
  first_ypos = tmp32[1];
  j = 0;
  count = 0;
  for (i=0; i < Npoints; i++)
    {
      cur_xpos = tmp32[2*i+0];
      cur_ypos = tmp32[2*i+1];
      out32[2*j+0] = last_xpos;
      out32[2*j+1] = last_ypos;
      j++;
      count++;
      dx = (double)cur_xpos - (double)last_xpos;
      dy = (double)cur_ypos - (double)last_ypos;
      dsqr = dx*dx + dy*dy;
      dlen = sqrt( dsqr );
      if (dlen > pLgMaster->dmax)
	{
	  n = (int)(dlen / pLgMaster->dmax );
	  n += 2.0;
	  xstep =  (int)(dx / n) & kMaxUnsigned;
	  ystep =  (int)(dy / n) & kMaxUnsigned;
	  for ( k = 0; k < n ; k++ ) {
	    xout = last_xpos + k * (int)xstep;
	    yout = last_ypos + k * (int)ystep;
	    out32[2*j+0] = xout;
	    out32[2*j+1] = yout;
	    j++;
	    count++;
	  }
	}
      last_xpos = cur_xpos;
      last_ypos = cur_ypos;
    }
  out32[2*j+0] = last_xpos;
  out32[2*j+1] = last_ypos;
  j++;
  count++;
  //  take care of jump between last and first points
  dx = first_xpos - last_xpos;
  dy = first_ypos - last_ypos;
  dsqr = dx*dx + dy*dy;
  if (dsqr > (pLgMaster->dmax * pLgMaster->dmax))
    {
      n = (int) (sqrt((double)(dsqr / pLgMaster->dmax)));
    n++; n++;
    xstep =  (int)(dx / n) & kMaxUnsigned;
    ystep =  (int)(dy / n) & kMaxUnsigned;
    for ( k = 0; k < n ; k++ ) {
      xout = last_xpos + k * xstep;
      yout = last_ypos + k * ystep;
      out32[2*j+0] = xout;
      out32[2*j+1] = yout;
      j++;
      count++;
    }
  }
  Npoints  = count;
  ptn_len = 8 * Npoints;

//   fault avoidance
  SaveBeamPosition(out_pattern);
  write( pLgMaster->fd_laser, out_pattern, ptn_len);
  doLoadWriteNum(pLgMaster, Npoints);

  SetDisplayClock(pLgMaster);
  StartHobbs(pLgMaster);
  pLgMaster->gDisplayFlag = 1;
  doDevDisplay(pLgMaster);
#ifdef PATDEBUG
  fprintf(stderr,"\nJUSTDODISP: DONE Got here");
#endif	 
  return;
}

int move_lite(struct lg_master *pLgMaster, int32_t new_xpos, int32_t new_ypos)
{
  int32_t last_xpos;
  int32_t last_ypos;
  double n, dx, dy, dsqr, dlen;
  struct lg_info  loc_info;
  int xstep, ystep;
  int rc;

  /* move slowly from current position to start of pattern */
  memset((char *)&loc_info, 0, sizeof(loc_info));
#ifdef  LASER_DEFINED
  doLGSTOP(pLgMaster);
  doStopPulse(pLgMaster);
  usleep( 2000 );
  doSetClock(pLgMaster, pLgMaster->gPeriod);
  ioctl( pLgMaster->fd_laser, LGGETANGLE, &loc_info );
#endif
  last_xpos = loc_info.ptns.xpos;
  last_ypos = loc_info.ptns.ypos;
  dx = (double)new_xpos - (double)last_xpos;
  dy = (double)new_ypos - (double)last_ypos;
  dsqr = dx*dx + dy*dy;
  dlen = sqrt( dsqr );
  if (dlen > pLgMaster->dmax)
    {
      n = (int)(dlen / pLgMaster->dmax);
      n += 60;
      xstep =  (int)(dx / n) & kMaxUnsigned;
      ystep =  (int)(dy / n) & kMaxUnsigned;

      rc = doWriteDevDelta(pLgMaster, xstep, ystep);
      if (rc)
	return(rc);
      rc = doLoadReadNum(pLgMaster, n);
      if (rc)
	return(rc);
      rc = doSetClock(pLgMaster, 150);
      if (rc)
	return(rc);
      rc = doSensor(pLgMaster);
      if (rc)
	return(rc);
      rc = doSensor(pLgMaster);
      if (rc)
	return(rc);
      usleep( 2 * (int)n * 150  + 2000 );
    }

  doSlowDownTimer(pLgMaster);
  /* end of lite move */
  return(0);
}


int move_dark(struct lg_master *pLgMaster, int32_t new_xpos, int32_t new_ypos)
{
  int32_t last_xpos;
  int32_t last_ypos;
  double n, dx, dy, dsqr, dlen;
  struct lg_info  loc_info;
  int xstep, ystep;
  int itest;

  memset((char *)&loc_info, 0, sizeof(loc_info));
  /* move slowly from current position to start of pattern */
#ifdef  LASER_DEFINED
  doLGSTOP(pLgMaster);
  doStopPulse(pLgMaster);
  usleep( 20000 );
  loc_info.clock_tick =  pLgMaster->gPeriod;
  doSetClock(pLgMaster, pLgMaster->gPeriod);
  ioctl( pLgMaster->fd_laser, LGGETANGLE, &loc_info );
#endif
  last_xpos = loc_info.ptns.xpos;
  last_ypos = loc_info.ptns.ypos;
  dx = (double)new_xpos - (double)last_xpos;
  dy = (double)new_ypos - (double)last_ypos;
  dsqr = dx*dx + dy*dy;
  dlen = sqrt( dsqr );
  n = (int)(dlen / pLgMaster->dmax );
  n += 20;
  xstep =  (int)(dx / n) & kMaxUnsigned;
  ystep =  (int)(dy / n) & kMaxUnsigned;

  itest = doWriteDevDelta(pLgMaster, xstep, ystep);
  if (itest)
    return itest;
  itest = doLoadReadNum(pLgMaster, n);
  if (itest)
    return itest;
  itest = doSetClock(pLgMaster, 150);
  if (itest)
    return(itest);
  itest = doDarkSensor(pLgMaster);
  if (itest)
    return(itest);

  usleep( 2 * (int)n * 150  + 2000 );
  doSlowDownTimer(pLgMaster);
  /* end of dark move */
  return(0);
}

int CDRHflag(struct lg_master *pLgMaster)
{
  pLgMaster->optic_status = inb(LG_IO_OPTIC);
  if (pLgMaster->optic_status < 0)
    { 
      fprintf(stderr, "CDRHflag lg_getoptic error %d\n", pLgMaster->optic_status);
      return (-1);
    }

  if (pLgMaster->optic_status & kCDRHbit)
    { 
      // shutter should be open
      pLgMaster->optic_status |= 0x80;
      outb(pLgMaster->optic_status, LG_IO_OPTIC);
      return(0);
    }
  else
    {
      fprintf(stderr, "optics %02x\n", pLgMaster->optic_status);
      pLgMaster->optic_status &= 0x7F;
      outb(pLgMaster->optic_status, LG_IO_OPTIC);
      return(1); 
    }
  return(-1);
}
