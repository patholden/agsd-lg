/*
static char rcsid[] = "$Id: BoardComm.c,v 1.36 2007/03/30 20:13:58 pickle Exp pickle $";
*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stddef.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include <assert.h>
#include <sys/io.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <sys/ioctl.h>
#include <linux/laser_api.h>
#include "AppCommon.h"
#include "BoardCommDefines.h"
#include "AppErrors.h"
#include "AppStrListIDs.h"
#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
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

uint32_t  gRespondToWhom = kDoNotRespond;

static  int32_t     gQuickCheckCounterStart = -1;


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
static  void  DoRespond (struct lg_master *pLgMaster, struct k_header *pHdr);
static void RestoreBeamPosition(struct lg_master *pLgMaster);
static int doSetQCFlag(struct lg_master *pLgMaster, uint32_t qcflag);
static int doSensor(struct lg_master *pLgMaster);
static int doDarkSensor(struct lg_master *pLgMaster);
static int move_lite(struct lg_master *pLgMaster, struct lg_xydata *pNewData);
static void SaveBeamPosition(struct lg_master *pLgMaster, char *data);
 
void PostCmdDisplay(struct lg_master *pLgMaster, struct displayData *p_dispdata, uint32_t respondToWhom)
{
  struct k_header pRespHdr;
  struct lg_xydata xydata;
  double         n, dx, dy, dsqr, dlen;
  char           *wr_ptr;
  uint32_t       *out32;
  uint32_t       *tmp32;
  struct cmd_rw  *cmd_buff;
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

  memset((char *)pLgMaster->theResponseBuffer, 0, sizeof(struct parse_basic_resp));
  if (!p_dispdata)
    return;

  wr_ptr = (char *)p_dispdata->sensorAngles;
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
  memset((char *)&xydata, 0, sizeof(struct lg_xydata));
  cmd_buff = (struct cmd_rw *)malloc(sizeof(struct cmd_rw));
  if (!cmd_buff)
    return;

  memset((char *)cmd_buff, 0, sizeof(struct cmd_rw));
  ROIoff(pLgMaster);
  setROIlength(pLgMaster, 0);
  if ( gVideoCheck == 0 )
    initQCcounter(pLgMaster);

  memcpy (tmp_pattern, wr_ptr, ptn_len);

  // Send delta for first xy val
  xydata.xdata = (uint16_t)(tmp32[0] & kMaxUnsigned);
  xydata.ydata = (uint16_t)(tmp32[1] & kMaxUnsigned);
  move_dark(pLgMaster, (struct lg_xydata *)&xydata);
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
      dx = (double)(cur_xpos - last_xpos);
      dy = (double)(cur_ypos - last_ypos);
      dsqr = dx*dx + dy*dy;
      dlen = sqrt( dsqr );
      if (dlen > pLgMaster->dmax)
	{
	  n = ((dlen / pLgMaster->dmax) + 0.001);
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

  memcpy(wr_ptr, out_pattern, ptn_len);  // Update sensor buffer
  SaveBeamPosition(pLgMaster, wr_ptr);
  memcpy(cmd_buff, out_pattern, ptn_len);
  cmd_buff->base.cmd = CMDW_BUFFER;
  cmd_buff->base.length = ptn_len;
  write( pLgMaster->fd_laser, cmd_buff, sizeof(struct cmd_rw));
#ifdef PATDEBUG
  fprintf(stderr,"\nPostCmdDisp: writing to device, len %d",ptn_len);
#endif
  doLoadWriteNum(pLgMaster, Npoints);
  if ( gVideoCheck == 0 )
    SaveAnglesForQuickCheck(p_dispdata, kRespondExtern );
  doSetClock(pLgMaster, pLgMaster->gPeriod);
  StartHobbs(pLgMaster);
  pLgMaster->gDisplayFlag = 1;
  doDevDisplay(pLgMaster);
  // reset display period to the default value (75 usec)
  pLgMaster->gPeriod = KETIMER_75U;

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
  struct k_header pRespHdr;
  struct lg_xydata xydata;
  double          n, dx, dy, dsqr, dlen;
  char            *wr_ptr;
  uint32_t        *out32;
  uint32_t        *tmp32;
  struct cmd_rw   *cmd_buff;
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

  memset((char *)pLgMaster->theResponseBuffer, 0, sizeof(struct parse_basic_resp));
  wr_ptr = (char *)p_dispdata->sensorAngles;
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

#ifdef PATDEBUG
  fprintf(stderr,"\nGOT INTO POSTCMDDISPNORESP");
#endif
  cmd_buff = (struct cmd_rw *)malloc(sizeof(struct cmd_rw));
  if (!cmd_buff)
    return;

  memset((char *)cmd_buff, 0, sizeof(struct cmd_rw));
  memset(out_pattern, 0, MAX_LG_BUFFER);
  memset(tmp_pattern, 0, MAX_LG_BUFFER);
  memset((char *)&xydata, 0, sizeof(struct lg_xydata));
  
  ROIoff(pLgMaster);
  setROIlength(pLgMaster, 0);
  if (gVideoCheck == 0)
    initQCcounter(pLgMaster);

  memcpy ((char *)&tmp32[0], wr_ptr, ptn_len);

  // Send delta for first xy val
  xydata.xdata = (uint16_t)(tmp32[0] & kMaxUnsigned);
  xydata.ydata = (uint16_t)(tmp32[1] & kMaxUnsigned);
  move_dark(pLgMaster, (struct lg_xydata *)&xydata);

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
      dx = (double)(cur_xpos - last_xpos);
      dy = (double)(cur_ypos - last_ypos);
      dsqr = dx*dx + dy*dy;
      dlen = sqrt( dsqr );
      if (dlen > pLgMaster->dmax)
	{
	  n = dlen / pLgMaster->dmax;
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
      n = sqrt(dsqr / pLgMaster->dmax);
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
  SaveBeamPosition(pLgMaster, wr_ptr);
#ifdef PATDEBUG
  fprintf(stderr,"\nPOSTCMDDISPNORESP: GOT PAST MEMCPY INTO WR_PTR, len %d",ptn_len);
#endif

  // Don't send if too much data for laser-dev to handle
  if (ptn_len > MAX_LGDEV_BUFF)
    {
      pRespHdr.status = RESPFAIL;
      DoRespond(pLgMaster, (struct k_header *)&pRespHdr);
      return;
    }
  memcpy(cmd_buff, (char *)&out32[0], ptn_len);
  cmd_buff->base.cmd = CMDW_BUFFER;
  cmd_buff->base.length = ptn_len;
  write( pLgMaster->fd_laser, cmd_buff, sizeof(struct cmd_rw));
#ifdef PATDEBUG
  fprintf(stderr,"\nPOSTCMDDISPNORESP: GOT PAST WRITE TO LASERDEV, len %d",ptn_len);
#endif
  fprintf(stderr,"\nPostCmdDispNoRsp: writing to device, len %d",ptn_len);
  doLoadWriteNum(pLgMaster, Npoints);
  if ( gVideoCheck == 0 ) {
    SaveAnglesForQuickCheck(p_dispdata, kRespondExtern );
  }

  doSetClock(pLgMaster, pLgMaster->gPeriod);
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
  return;
}
void PostCmdEtherAngle(struct lg_master *pLgMaster, struct lg_xydata *pAngleData, uint32_t respondToWhom)
{
  struct k_header gResponseBuffer;

  memset((char *)&gResponseBuffer, 0, sizeof(struct k_header));

  gResponseBuffer.status = RESPGOOD;
  move_lite(pLgMaster, pAngleData);
  SetHighBeam(pAngleData);
#ifdef PATDEBUG
  fprintf(stderr,"\nxPOSTETHER: TODEV xdata %d, ydata %d, flags %x", pAngleData->xdata, pAngleData->ydata, pAngleData->ctrl_flags);
#endif
  doWriteDevPoints(pLgMaster, pAngleData);
  DoRespond (pLgMaster, (struct k_header *)&gResponseBuffer);
  return;
}
void PostCmdGoAngle(struct lg_master *pLgMaster, struct lg_xydata *pAngleData, uint32_t respondToWhom)
{
  struct k_header gResponseBuffer;

  memset((char *)&gResponseBuffer, 0, sizeof(struct k_header));
  
  gResponseBuffer.status = RESPGOOD;
  move_lite(pLgMaster, pAngleData);
  SetHighBeam(pAngleData);
#ifdef PATDEBUG
  fprintf(stderr,"\nxdata %d,ydata %d, ctrlflg %d",pAngleData->xdata,pAngleData->ydata, pAngleData->ctrl_flags);
#endif
  doWriteDevPoints(pLgMaster, pAngleData);
  if (!(pLgMaster->gHeaderSpecialByte & 0x80))
    DoRespond (pLgMaster, (struct k_header *)&gResponseBuffer);
  return;
}
void PostCommand(struct lg_master *pLgMaster, uint32_t theCommand, char *data, uint32_t respondToWhom )
{
  struct lg_xydata   xydata;
  struct k_header    gResponseBuffer;
  struct displayData *dispData;
  char               *wr_ptr;
  uint32_t           *data32;
  uint32_t           *out32;
  uint32_t           *tmp32;
  struct cmd_rw      *cmd_buff;
  int32_t            Npoints;
  uint32_t           ptn_len;
  
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
	doSetClock(pLgMaster, KETIMER_10M);
	doStopPulse(pLgMaster);
	doSetReadyLED(pLgMaster);
	ROIoff(pLgMaster);
	g_FODflag = 0;
	gQuickCheckCounterStart = -1;
	stopQCcounter(pLgMaster);   /* never quick check */
	gVideoCount = 0;
	gVideoCheck = 0;
	setROIlength(pLgMaster, 0);
	gStopFlag = -1;
	gResponseBuffer.status = RESPSTOPOK;
	EndHobbs(pLgMaster);
	
	if ( !(pLgMaster->gHeaderSpecialByte & 0x80 ) ) {
	  DoRespond (pLgMaster, (struct k_header *)&gResponseBuffer);
	}
        break;
    case kDisplayVideoCheck:
	dispData = (struct displayData *)data;
	wr_ptr = (char *)dispData->sensorAngles;
	if (!wr_ptr)
	  return;
      ptn_len = pLgMaster->patternLength;
      if (ptn_len > MAX_DATA)
	{
	  fprintf(stderr,"\nKDISP-VCHK:  BAD LENGTH %d", ptn_len);
	  return;
	  }
      cmd_buff = (struct cmd_rw *)malloc(sizeof(struct cmd_rw));
      if (!cmd_buff)
	return;
      
      memset((char *)cmd_buff, 0, sizeof(struct cmd_rw));
      ROIoff(pLgMaster);
      setROIlength(pLgMaster, 0);
      gQuickCheckCounterStart = -1;
      initQCcounter(pLgMaster);   /* never quick check */
      Npoints = ptn_len / 8;
          
      SaveBeamPosition(pLgMaster, wr_ptr);
      memcpy(cmd_buff, wr_ptr, ptn_len);
      cmd_buff->base.cmd = CMDW_BUFFER;
      cmd_buff->base.length = ptn_len;
      write( pLgMaster->fd_laser, cmd_buff, sizeof(struct cmd_rw));
      fprintf(stderr,"\nPostCmdVideo: writing to device, len %d",ptn_len);
      doLoadWriteNum(pLgMaster, Npoints);
      
      doSetClock(pLgMaster, pLgMaster->gPeriod);
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
        break;
    case kDarkAngle:
      data32 = (uint32_t *)&data[0];
      if (!data32)
	return;
      doSetClock(pLgMaster, KETIMER_10M);
      memset((char *)&xydata, 0, sizeof(struct lg_xydata));
      xydata.xdata = (data32[0] & kMaxUnsigned);
      xydata.ydata = (data32[1] & kMaxUnsigned);
      move_dark(pLgMaster, (struct lg_xydata *)&xydata);
      doWriteDevPoints(pLgMaster, (struct lg_xydata *)&xydata);
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
      wr_ptr = (char *)dispData->sensorAngles;
      if (!wr_ptr)
	return;
      ptn_len = pLgMaster->patternLength;
      if (ptn_len > MAX_DATA)
	{
	  fprintf(stderr,"\nKDISP-NOQCHK:  BAD LENGTH %d", ptn_len);
	  return;
	}
      cmd_buff = (struct cmd_rw *)malloc(sizeof(struct cmd_rw));
      if (!cmd_buff)
	return;
      
      memset((char *)cmd_buff, 0, sizeof(struct cmd_rw));
      ROIoff(pLgMaster);
      setROIlength(pLgMaster, 0);
      gQuickCheckCounterStart = -1;
      initQCcounter(pLgMaster);   /* never quick check */
      Npoints = ptn_len / 8;
          
      SaveBeamPosition(pLgMaster, wr_ptr);
      memcpy(cmd_buff, wr_ptr, ptn_len);
      cmd_buff->base.cmd = CMDW_BUFFER;
      cmd_buff->base.length = ptn_len;
      write( pLgMaster->fd_laser, cmd_buff, sizeof(struct cmd_rw));
      fprintf(stderr,"\nPostCmdQChk: writing to device, len %d",ptn_len);
      doLoadWriteNum(pLgMaster, Npoints);
      doSetClock(pLgMaster, pLgMaster->gPeriod);
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
  
  pLgMaster->fd_laser = open("/dev/laser", O_RDWR);
  if (pLgMaster->fd_laser < 0)
    {
      fprintf(stderr,"\nlaser device %d not opened,errno %d", pLgMaster->fd_laser, errno);
      perror( "/dev/laser" );
      return(-1);
    }
  doLGSTOP(pLgMaster);
  doSetClock(pLgMaster, KETIMER_10M);
  doStopPulse(pLgMaster);

  // initialize buffers

  tmp_pattern = malloc(MAX_LG_BUFFER);
  if (!tmp_pattern)
    return(-2);
  out_pattern = malloc(MAX_LG_BUFFER);
  if (!out_pattern)
    return(-3);
  return(0);
}

int DoLineSearch(struct lg_master *pLgMaster, struct lg_xydata *pSrchData,
		 struct lg_xydata *pDeltaData, uint32_t n, unsigned char *c_out)
{
    int        itest, num;
    int        count, blank;
    int        optic_status=0;

    itest = doWriteDevPoints(pLgMaster, pSrchData);
    if (itest)
      return(itest);
	  
    count = 10000;
    blank = 500;

  
    ioctl(pLgMaster->fd_laser, LGGETCTL2STAT, &optic_status);
    while (count--  && blank)
      {
	if (optic_status & PHTEDGEDTCT)
	  blank = 500;
	else
	  blank--;
      }
    itest = doWriteDevDelta(pLgMaster, pDeltaData);
    if (itest)
      return itest;
	    
    itest = doLoadReadNum(pLgMaster, n);
    if (itest)
      return(itest);
    itest = doSetClock(pLgMaster, KETIMER_40U);
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

    itest = doSetClock(pLgMaster, KETIMER_10M);
    if (itest)
      return itest;
    return 0;
}


int DoLevelSearch(struct lg_master *pLgMaster, struct lg_xydata *pSrchData,
		  struct lg_xydata *pDeltaData, uint32_t n, uint32_t *c_out)
{
  int itest, num;
  uint32_t index;
  unsigned short searchbuff[65536];

  itest = doWriteDevPoints(pLgMaster, pSrchData);
  if (itest && (errno != ENOTTY))
    return itest;
  doSetSearchBeam(pLgMaster);
  itest = doWriteDevDelta(pLgMaster, pDeltaData);
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

  itest = doSetClock(pLgMaster, KETIMER_10M);
  if (itest && (errno != ENOTTY))
    return itest;
  return 0;
}

int SearchBeamOn(struct lg_master *pLgMaster)
{
  doSetSearchBeam(pLgMaster);
  usleep( 10000 );  /* pause a bit after turning on beam */
  return 0;
}

int SearchBeamOff(struct lg_master *pLgMaster)
{
  int rc=0;

  doClearSearchBeam(pLgMaster);
  // attempt to turn off beam
  rc = doLGSTOP(pLgMaster);
  return(rc);
}

int doWriteDevCmdNoData(struct lg_master *pLgMaster, uint32_t command)
{
  int    rc=0;

  rc = write(pLgMaster->fd_laser, (char *)&command, sizeof(uint32_t));
  if (rc < 0)
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
int doRestartTimer(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_RSTRTTMR));
}
static int doSensor(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_DOSENSOR));
}
static int doDarkSensor(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_DODARKSENS));
}
int doSetSearchBeam(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_SEARCHBEAMON));
}
int doClearSearchBeam(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_SEARCHBEAMOFF));
}
int doSetReadyLED(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_READYLEDON));
}
int doClearReadyLED(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_READYLEDOFF));
}
void doClearLinkLED(struct lg_master *pLgMaster)
{
  fprintf( stderr, " about to Clear Link LED\n" );
  doWriteDevCmdNoData(pLgMaster, CMDW_LINKLEDOFF);
  return;
}
int doClearShutterENB(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_CLEARSHUTENB));
}
int doSetShutterENB(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_SETSHUTENB));
}
int doWriteDevCmd32(struct lg_master *pLgMaster, uint32_t command, uint32_t write_val)
{
  int    rc=0;
  struct cmd_rw_base *p_cmd_data;

  p_cmd_data = (struct cmd_rw_base *)malloc(sizeof(struct cmd_rw_base));
  if (!p_cmd_data)
    return(-1);
  
  p_cmd_data->cmd = command;
  p_cmd_data->dat32.val32 = write_val;
  p_cmd_data->length = sizeof(uint32_t);
  rc = write(pLgMaster->fd_laser, (char *)p_cmd_data, sizeof(struct cmd_rw_base));
  if (rc < 0)
    fprintf(stderr,"\nCMDW-VAL32: ERROR cmd %d, val %d, rc %d, errno %d\n", command, write_val, rc, errno);
  free(p_cmd_data);
  return(rc);
}
int doWriteDevDelta(struct lg_master *pLgMaster, struct lg_xydata *pDelta)
{
  int    rc=0;
  struct cmd_rw_base *p_cmd_data;

  p_cmd_data = (struct cmd_rw_base *)malloc(sizeof(struct cmd_rw_base));
  if (!p_cmd_data)
    return(-1);
  
  p_cmd_data->cmd = CMDW_SETDELTA;
  p_cmd_data->xydata.xdata = pDelta->xdata;
  p_cmd_data->xydata.ydata = pDelta->ydata;
  p_cmd_data->length = sizeof(struct lg_xydata);
  rc = write(pLgMaster->fd_laser, (char *)p_cmd_data, sizeof(struct cmd_rw_base));
  if (rc < 0)
    fprintf(stderr,"\nCMDW-SETDELTA: ERROR xval %d, yval %d, rc %d, errno %d\n", pDelta->xdata, pDelta->ydata, rc, errno);
  free(p_cmd_data);
  return(rc);
}
int doWriteDevPoints(struct lg_master *pLgMaster, struct lg_xydata *pPoints)
{
  int    rc=0;
  struct cmd_rw_base *p_cmd_data;

  p_cmd_data = (struct cmd_rw_base *)malloc(sizeof(struct cmd_rw_base));
  if (!p_cmd_data)
    return(-1);
  
  p_cmd_data->cmd = CMDW_GOANGLE;
  p_cmd_data->xydata.ctrl_flags = pPoints->ctrl_flags;
  p_cmd_data->xydata.xdata = pPoints->xdata;
  p_cmd_data->xydata.ydata = pPoints->ydata;
  p_cmd_data->length = sizeof(struct lg_xydata);
  rc = write(pLgMaster->fd_laser, (char *)p_cmd_data, sizeof(struct cmd_rw_base));
  if (rc < 0)
    fprintf(stderr,"\nCMDW-GOANGLE: ERROR xval %d, yval %d, rc %d, errno %d\n", pPoints->xdata, pPoints->ydata, rc, errno);
#ifdef PATDEBUG
  fprintf(stderr,"\nCMDW-GOANGLE: cnt %d, xval %x, yval %x, flags %x\n", rc, pPoints->xdata, pPoints->ydata,pPoints->ctrl_flags);
#endif
  free(p_cmd_data);
  return(rc);
}

void SetHighBeam(struct lg_xydata *pDevXYData)
{
  // Set beam HIGH, set laser ENABLED
  pDevXYData->ctrl_flags = BRIGHTISSET | UNBLANKISSET;
  return;
}
void SetLowBeam(struct lg_xydata *pDevXYData)
{
  // Set beam LOW, keep laser enabled
  pDevXYData->ctrl_flags = UNBLANKISSET;
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
int doSetClock(struct lg_master *pLgMaster, uint32_t clock_rate)
{
  return(doWriteDevCmd32(pLgMaster, CMDW_SETCLOCK, clock_rate));
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
  uint32_t qcFlag=0;
  
  itest = ioctl( pLgMaster->fd_laser, LGGETQCFLAG, &qcFlag);
  if (itest)
    return(-1);
	 
  return(qcFlag);
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
  uint32_t  qcCounter=0;

  itest = ioctl( pLgMaster->fd_laser, LGGETQCCOUNTER, &qcCounter);
  if (itest)
    return(-1);

  return(qcCounter);
}

void ResumeDisplay(struct lg_master *pLgMaster)
{

  RestoreBeamPosition(pLgMaster);
  doSetClock(pLgMaster, pLgMaster->gPeriod);
  StartHobbs(pLgMaster);
  pLgMaster->gDisplayFlag = 1;
  doDevDisplay(pLgMaster);
}

/*  intended to save initial position */
static void SaveBeamPosition(struct lg_master *pLgMaster, char *data)
{
  memcpy((char *)&pLgMaster->gSaveXY, data, sizeof(struct lg_xydata));
  return;
}

void RestoreBeamPosition(struct lg_master *pLgMaster)
{
  move_dark(pLgMaster, (struct lg_xydata *)&pLgMaster->gSaveXY);
  doWriteDevPoints(pLgMaster, (struct lg_xydata *)&pLgMaster->gSaveXY);
  // FIXME---PAT---Is this sleep necessary?
  usleep( 10000 );
  return;
}

void GoToRaw(struct lg_master *pLgMaster, struct lg_xydata *pRawData)
{
  doSetClock(pLgMaster, KETIMER_10M);
  move_dark(pLgMaster, pRawData);
  doWriteDevPoints(pLgMaster, pRawData);
  return;
}

void GoToPulse(struct lg_master *pLgMaster, struct lg_xydata *pPulseData,
	       int32_t pulseoffvalue, int32_t pulseonvalue)
{
    pLgMaster->gPeriod = KETIMER_50U;
    move_lite(pLgMaster, pPulseData);
    doWriteDevPoints(pLgMaster, pPulseData);
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
      doClearReadyLED(pLgMaster);
      usleep( 250000U );
      doSetReadyLED(pLgMaster);
    }
  return;
}
void SlowDownAndStop(struct lg_master *pLgMaster)
{
  // stop display and slow down the clock
  // check and update hobbs meters, if necessary
  doLGSTOP(pLgMaster);
  doSetClock(pLgMaster, KETIMER_10M);
  EndHobbs(pLgMaster);
  return;
}

void JustDoDisplay(struct lg_master *pLgMaster, char * wr_ptr, int pattern_len)
{
  struct  lg_xydata xydata;
  int Npoints;
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
  if (!tmp32 || !out32 || !wr_ptr)
    return;

  memset((char *)&xydata, 0, sizeof(struct lg_xydata));
  ptn_len = pattern_len;

  // Send new settings to laser-dev
  doLGSTOP(pLgMaster);
  doSetClock(pLgMaster, KETIMER_10M);
  doStopPulse(pLgMaster);
  usleep( 10000 );
  ROIoff(pLgMaster);
  setROIlength(pLgMaster, 0);
  stopQCcounter(pLgMaster);   /* never quick check */

  // fault avoidance
  Npoints = ptn_len / 8;
  memcpy ((char *)tmp_pattern, wr_ptr, ptn_len);
  xydata.xdata = tmp32[0] & kMaxUnsigned;
  xydata.ydata = tmp32[1] & kMaxUnsigned;
  move_dark(pLgMaster, (struct lg_xydata *)&xydata);
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
  dx = (double)(first_xpos - last_xpos);
  dy = (double)(first_ypos - last_ypos);
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
  SaveBeamPosition(pLgMaster, out_pattern);
  write( pLgMaster->fd_laser, out_pattern, ptn_len);
  doLoadWriteNum(pLgMaster, Npoints);

  doSetClock(pLgMaster, pLgMaster->gPeriod);
  StartHobbs(pLgMaster);
  pLgMaster->gDisplayFlag = 1;
  doDevDisplay(pLgMaster);
  return;
}

static int move_lite(struct lg_master *pLgMaster, struct lg_xydata *pNewData)
{
  double n, dx, dy, dsqr, dlen;
  struct lg_xydata  xydata;
  int rc;

  /* move slowly from current position to start of pattern */
  memset((char *)&xydata, 0, sizeof(struct lg_xydata));
  // FIXME---PAH---Do we need usleep?
  // Can 4 writes to device be replaced with 1 single command?
  doLGSTOP(pLgMaster);
  doStopPulse(pLgMaster);
  usleep( 2000 );
  ioctl(pLgMaster->fd_laser, LGGETANGLE, &xydata);
  dx = (double)(pNewData->xdata - xydata.xdata);
  dy = (double)(pNewData->ydata - xydata.ydata);
  dsqr = dx*dx + dy*dy;
  dlen = sqrt( dsqr );
  if (dlen > pLgMaster->dmax)
    {
      // Just re-use the struct, delta doesn't care about ctrl-flags
      memset((char *)&xydata, 0, sizeof(struct lg_xydata));
      n = dlen / pLgMaster->dmax;
      n += 60;
      xydata.xdata =  (uint16_t)(dx / n) & kMaxUnsigned;
      xydata.ydata =  (uint16_t)(dy / n) & kMaxUnsigned;

      rc = doWriteDevDelta(pLgMaster, (struct lg_xydata *)&xydata);
      if (rc)
	return(rc);
      rc = doLoadReadNum(pLgMaster, n);
      if (rc)
	return(rc);
      rc = doSensor(pLgMaster);
      if (rc)
	return(rc);
      usleep( 2 * (int)n * 150  + 2000 );
    }

  doSetClock(pLgMaster, KETIMER_10M);
  /* end of lite move */
  return(0);
}

int move_dark(struct lg_master *pLgMaster, struct lg_xydata *pNewData)
{
  struct lg_xydata  xydata;
  double            n, dx, dy, dsqr, dlen;
  int               rc;

  memset((char *)&xydata, 0, sizeof(struct lg_xydata));
  /* move slowly from current position to start of pattern */
  doLGSTOP(pLgMaster);
  doStopPulse(pLgMaster);
  // FIXME---PAH--Do we really need this sleep() call?
  usleep( 20000 );
  doSetClock(pLgMaster, pLgMaster->gPeriod);
  ioctl( pLgMaster->fd_laser, LGGETANGLE, &xydata);
  dx = (double)(pNewData->xdata - xydata.xdata);
  dy = (double)(pNewData->ydata - xydata.ydata);
  dsqr = dx*dx + dy*dy;
  dlen = sqrt( dsqr );
  n = dlen / pLgMaster->dmax;
  n += 20;
  // Just re-use the struct, delta doesn't care about ctrl-flags
  memset((char *)&xydata, 0, sizeof(struct lg_xydata));
  xydata.xdata =  (uint16_t)(dx / n) & kMaxUnsigned;
  xydata.ydata =  (uint16_t)(dy / n) & kMaxUnsigned;
  rc = doWriteDevDelta(pLgMaster, (struct lg_xydata *)&xydata);
  if (rc)
    return(rc);
  rc = doLoadReadNum(pLgMaster, n);
  if (rc)
    return(rc);
  rc = doSetClock(pLgMaster, KETIMER_150U);
  if (rc)
    return(rc);
  rc = doDarkSensor(pLgMaster);
  if (rc)
    return(rc);

  // FIXME---PAH---why is this sleep here?
  usleep( 2 * (int)n * 150  + 2000 );
  doSetClock(pLgMaster, KETIMER_10M);
  /* end of dark move */
  return(0);
}

int CDRHflag(struct lg_master *pLgMaster)
{
  int  optic_status=0;


  // FIXME---PAH---NEED TO FIND OUT WHY STATUS IS NOT RETURNED
  ioctl(pLgMaster->fd_laser, LGGETCTL2STAT &optic_status);
  if (optic_status & CDRHBITMASK)
    { 
      // shutter should be open
      fprintf(stderr, "\nCDRHFlag:  optics %x, shutter OPEN\n", optic_status);
      doSetShutterENB(pLgMaster);
      return(0);
    }

  // shutter is closed, not good!
  fprintf(stderr, "\nCDRHFlag:  optics %x, shutter CLOSED\n", optic_status);
  doClearShutterENB(pLgMaster);
  
  // return(1); 
  return(0); 
}
