/*
static char rcsid[] = "$Id: BoardComm.c,v 1.36 2007/03/30 20:13:58 pickle Exp pickle $";
*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
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
static struct lg_xydata *tmp_pattern=0;
static struct lg_xydata *out_pattern=0;
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
 
static  void  DoRespond (struct lg_master *pLgMaster, struct k_header *pHdr);
static void RestoreBeamPosition(struct lg_master *pLgMaster);
static int doSetQCFlag(struct lg_master *pLgMaster, uint32_t qcflag);
static int doSensor(struct lg_master *pLgMaster);
static int doDarkSensor(struct lg_master *pLgMaster);
static int move_lite(struct lg_master *pLgMaster, struct lg_xydata *pNewData);
static void SaveBeamPosition(struct lg_master *pLgMaster, char *data);
 
static void FillDispBuff(struct lg_master *pLgMaster, uint32_t ptn_len)
{
    struct k_header  pRespHdr;
    struct lg_xydata first_xypos;
    struct lg_xydata cur_xypos;
    struct lg_xydata last_xypos;
    struct lg_xydata *pXYout;
    struct lg_xydata *pXYtmp;
    struct cmd_rw    *cmd_buff;
    double           n,dx,dy,dsqr,dlen;
    uint32_t         i,count, Npoints;
    uint16_t         k,xstep, ystep;

    // Prepare for response back to PC host
    memset((void *)&pRespHdr, 0, sizeof(struct k_header));

    // Set up buffer for laser-dev data  
    cmd_buff = (struct cmd_rw *)malloc(sizeof(struct cmd_rw));
    if (!cmd_buff)
      return;

    // Start with a clean slate
    memset((char *)&cur_xypos, 0, sizeof(struct lg_xydata));
    memset((char *)&first_xypos, 0, sizeof(struct lg_xydata));
    memset((char *)&last_xypos, 0, sizeof(struct lg_xydata));
    memset((char *)cmd_buff, 0, sizeof(struct cmd_rw));

    // Send delta for first xy val
    pXYtmp = tmp_pattern;
    move_dark(pLgMaster, pXYtmp);

    // Start with first & last pointing to initial XY points
    first_xypos.xdata = last_xypos.xdata = pXYtmp->xdata;
    first_xypos.ydata = last_xypos.ydata = pXYtmp->ydata;
    first_xypos.ctrl_flags = last_xypos.ctrl_flags = pXYtmp->ctrl_flags;
    count = 0;
    Npoints = ptn_len / sizeof(struct lg_xydata);
    for (i=0; i < Npoints; i++)
      {
	pXYtmp = (struct lg_xydata *)((char *)tmp_pattern + (sizeof(struct lg_xydata) * i));
	pXYout = (struct lg_xydata *)((char *)out_pattern + (sizeof(struct lg_xydata) * count));
	cur_xypos.xdata = pXYtmp->xdata;
	cur_xypos.ydata = pXYtmp->ydata;
	cur_xypos.ctrl_flags = pXYtmp->ctrl_flags;
	pXYout->xdata = last_xypos.xdata;
	pXYout->ydata = last_xypos.ydata;
	pXYout->ctrl_flags = last_xypos.ctrl_flags;
	count++;
	dx = (double)cur_xypos.xdata - (double)last_xypos.xdata;
	dy = (double)cur_xypos.ydata - (double)last_xypos.ydata;
	dsqr = dx*dx + dy*dy;
	dlen = sqrt(dsqr);
	if (dlen > pLgMaster->dmax)
	  {
	    n = ((dlen / pLgMaster->dmax) + 0.001);
	    n += 2.0;
	    xstep =  (uint16_t)(dx / n);
	    ystep =  (uint16_t)(dy / n);
	    for (k = 0; k < n; k++)
	      {
		pXYout = (struct lg_xydata *)((char *)out_pattern + (sizeof(struct lg_xydata) * count));
		pXYout->xdata = last_xypos.xdata + (k * xstep);
		pXYout->ydata = last_xypos.ydata + (k * ystep);
		pXYout->ctrl_flags = last_xypos.ctrl_flags;
		count++;
	      }
	  }
	last_xypos.xdata = cur_xypos.xdata;
	last_xypos.ydata = cur_xypos.ydata;
	last_xypos.ctrl_flags = cur_xypos.ctrl_flags;
      }
    // Set up last position
    pXYout = (struct lg_xydata *)((char *)out_pattern + (sizeof(struct lg_xydata) * count));
    pXYout->xdata = last_xypos.xdata;
    pXYout->ydata = last_xypos.ydata;
    pXYout->ctrl_flags = last_xypos.ctrl_flags;
    dx = (double)first_xypos.xdata - (double)last_xypos.xdata;
dy = (double)first_xypos.ydata - (double)last_xypos.ydata;
    dsqr = dx*dx + dy*dy;
    if (dsqr > (pLgMaster->dmax * pLgMaster->dmax))
      {
	count++;
	n = (sqrt((double)(dsqr / pLgMaster->dmax)));
	n+= 2.0;
	xstep =  (uint16_t)(dx / n);
	ystep =  (uint16_t)(dy / n);
	for (k = 0; k < n ; k++)
	  {
	    pXYout = (struct lg_xydata *)((char *)out_pattern + (sizeof(struct lg_xydata) * count));
	    pXYout->xdata = last_xypos.xdata + (k * xstep);
	    pXYout->ydata = last_xypos.ydata + (k * ystep);
	    pXYout->ctrl_flags = last_xypos.ctrl_flags;
	    count++;
	  }
      }
    ptn_len = count * sizeof(struct lg_xydata);
    
    SaveBeamPosition(pLgMaster, (char *)out_pattern);
    // Don't send if too much data for laser-dev to handle
    if (ptn_len > MAX_LG_BUFFER)
      {
	pRespHdr.status = RESPFAIL;
	DoRespond(pLgMaster, (struct k_header *)&pRespHdr);
	return;
      }
    memcpy((char *)&cmd_buff->xydata[0], (char *)out_pattern, ptn_len);
    cmd_buff->base.cmd = CMDW_BUFFER;
    cmd_buff->base.length = ptn_len;
    write(pLgMaster->fd_laser, cmd_buff, sizeof(struct cmd_rw));
    free(cmd_buff);
    return;
}
void PostCmdDisplay(struct lg_master *pLgMaster, struct displayData *p_dispdata, int32_t do_response, uint32_t respondToWhom)
{
  struct k_header pRespHdr;
  uint32_t         ptn_len;

  // Prepare for response back to PC host
  memset((void *)&pRespHdr, 0, sizeof(struct k_header));

  // Check input data
  if (!p_dispdata)
    return;

  ptn_len = pLgMaster->gBuiltPattern;
  if (!p_dispdata->pattern || !ptn_len || (ptn_len > MAX_LG_BUFFER))
    {
      ROIoff(pLgMaster);
      initQCcounter(pLgMaster);
      // Only fail if trying to display more than driver can handle.
      // Can get here to display nothing too for some reason
      if (!p_dispdata->pattern || !ptn_len)
	pRespHdr.status = RESPGOOD;
      else
	pRespHdr.status = RESPFAIL;
      if ((do_response == SENDRESPONSE) ||  (pRespHdr.status == RESPFAIL))
	DoRespond (pLgMaster, (struct k_header *)&pRespHdr);
      return;
    }

  // Initialize all buffers to be used
  memset(tmp_pattern, 0, MAX_LG_BUFFER);
  memset(out_pattern, 0, MAX_LG_BUFFER);

  // Send initial commands to laser-dev
  ROIoff(pLgMaster);
  setROIlength(pLgMaster, 0);
  if ( gVideoCheck == 0 )
    initQCcounter(pLgMaster);

  // Get pattern data
  memcpy (tmp_pattern, (char *)p_dispdata->pattern, ptn_len);
  FillDispBuff(pLgMaster, ptn_len);
  doLoadWriteNum(pLgMaster, ptn_len);
  if ( gVideoCheck == 0 )
    SaveAnglesForQuickCheck(p_dispdata, kRespondExtern );
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
  if (do_response == SENDRESPONSE)
    DoRespond (pLgMaster, (struct k_header *)&pRespHdr);
  return;
}
void PostCmdEtherAngle(struct lg_master *pLgMaster, struct lg_xydata *pAngleData)
{
    pLgMaster->gDisplayFlag = 0;
    SetHighBeam(pAngleData);
    move_lite(pLgMaster, pAngleData);
    doWriteDevPoints(pLgMaster, pAngleData);
    return;
}
void PostCmdGoAngle(struct lg_master *pLgMaster, struct lg_xydata *pAngleData, uint32_t respondToWhom)
{
  struct k_header gResponseBuffer;

  memset((char *)&gResponseBuffer, 0, sizeof(struct k_header));
  
  gResponseBuffer.status = RESPGOOD;
  SetHighBeam(pAngleData);
  move_lite(pLgMaster, pAngleData);
  doWriteDevPoints(pLgMaster, pAngleData);
  if (!(pLgMaster->gHeaderSpecialByte & 0x80))
    DoRespond (pLgMaster, (struct k_header *)&gResponseBuffer);
  usleep(1000000);
  return;
}
void PostCommand(struct lg_master *pLgMaster, uint32_t theCommand, char *data, uint32_t respondToWhom )
{
  struct lg_xydata   xydata;
  struct k_header    gResponseBuffer;
  struct event_times last_times;
  struct displayData *dispData;
  struct cmd_rw      *cmd_buff;
  struct lg_xydata   *pXYData;
  int32_t            Npoints;
  uint32_t           ptn_len;
  
  memset((char *)&gResponseBuffer, 0, sizeof(gResponseBuffer));
  
  // Get current execution time value for laser device driver display
  ioctl(pLgMaster->fd_laser, LGGETEVENTTIMES, &last_times);
  syslog(LOG_NOTICE, "Last driver display execution time %lu usec",last_times.last_exec_usec);
  syslog(LOG_NOTICE, "Last driver display interval %lu usec",last_times.last_gap_usec);

  //  assume display is off or being turned off
  pLgMaster->gDisplayFlag = 0;
  gRespondToWhom = respondToWhom;
  
  if (!pLgMaster)
    {
      syslog(LOG_ERR,"POSTCMD:  BAD POINTER %d",theCommand);
      return;
    }
  switch ( theCommand )
    {
      case kStop:
	// stop, slow down and turn off Ready LED
	doLGSTOP(pLgMaster);
	doSetClock(pLgMaster, KETIMER_10M);
	doStopPulse(pLgMaster);
	doClearReadyLED(pLgMaster);
	ROIoff(pLgMaster);
	g_FODflag = 0;
	gQuickCheckCounterStart = -1;
	stopQCcounter(pLgMaster);   /* never quick check */
	gVideoCount = 0;
	gVideoCheck = 0;
	setROIlength(pLgMaster, 0);
	gResponseBuffer.status = RESPSTOPOK;
	EndHobbs(pLgMaster);
	
	if ( !(pLgMaster->gHeaderSpecialByte & 0x80 ) ) {
	  DoRespond (pLgMaster, (struct k_header *)&gResponseBuffer);
	}
        break;
    case kDisplayVideoCheck:
	dispData = (struct displayData *)data;
	if (!dispData->pattern)
	  return;
      ptn_len = pLgMaster->gBuiltPattern;
      if (ptn_len > MAX_LG_BUFFER)
	{
	  syslog(LOG_ERR,"\nKDISP-VCHK:  BAD LENGTH %d", ptn_len);
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
      Npoints = ptn_len / sizeof(struct lg_xydata);
          
      SaveBeamPosition(pLgMaster, (char *)dispData->pattern);
      memcpy((char *)&cmd_buff->xydata[0], (char *)dispData->pattern, ptn_len);
      cmd_buff->base.cmd = CMDW_BUFFER;
      cmd_buff->base.length = ptn_len;
      write( pLgMaster->fd_laser, cmd_buff, sizeof(struct cmd_rw));
      free(cmd_buff);
#ifdef AGS_DEBUG
      syslog(LOG_DEBUG,"\nPostCmdVideo: writing to device, len %d",ptn_len);
#endif
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
    case kDarkAngle:
      doSetClock(pLgMaster, KETIMER_10M);
      memset((char *)&xydata, 0, sizeof(struct lg_xydata));
      pXYData = (struct lg_xydata *)data;
      move_dark(pLgMaster, pXYData);
#ifdef AGS_DEBUG
      syslog(LOG_DEBUG,"\nDARKANGLE: x=%d,y=%d",pXYData->xdata, pXYData->ydata);
#endif
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
      if (!dispData->pattern)
	return;
      ptn_len = pLgMaster->gBuiltPattern;
      if (ptn_len > MAX_LG_BUFFER)
	{
	  syslog(LOG_ERR,"\nKDISP-NOQCHK:  BAD LENGTH %d", ptn_len);
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
      Npoints = ptn_len / sizeof(struct lg_xydata);
          
      SaveBeamPosition(pLgMaster, (char *)dispData->pattern);
      memcpy((char *)&cmd_buff->xydata[0], (char *)dispData->pattern, ptn_len);
      cmd_buff->base.cmd = CMDW_BUFFER;
      cmd_buff->base.length = ptn_len;
      write(pLgMaster->fd_laser, cmd_buff, sizeof(struct cmd_rw));
      free(cmd_buff);
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
    close(pLgMaster->fd_laser);
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
      syslog(LOG_ERR,"\nlaser device %d not opened,errno %d", pLgMaster->fd_laser, errno);
      return(-1);
    }
  doLGSTOP(pLgMaster);
  doSetClock(pLgMaster, KETIMER_10M);
  doStopPulse(pLgMaster);

  // initialize buffers

  tmp_pattern = (struct lg_xydata *)malloc(MAX_LG_BUFFER);
  if (!tmp_pattern)
    return(-2);
  out_pattern = (struct lg_xydata *)malloc(MAX_LG_BUFFER);
  if (!out_pattern)
    return(-3);
  return(0);
}

int DoLineSearch(struct lg_master *pLgMaster, struct lg_xydata *pSrchData,
		 struct lg_xydelta *pDeltaData, int16_t n, unsigned char *c_out)
{
    int        itest, num;
    int        count, blank;
    int        optic_status=0;

#ifdef AGS_DEBUG
    syslog(LOG_DEBUG,"\nLINESEARCH: Writing x %d, y %d", pSrchData->xdata,pSrchData->ydata);
#endif
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
#ifdef AGS_DEBUG
    syslog(LOG_DEBUG, "LINESEARCH:  Delta x=%x, y=%x",pDeltaData->xdata, pDeltaData->ydata);
#endif
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
#ifdef AGS_DEBUG
    syslog(LOG_DEBUG,"\nLINESEARCH: Reading data from laser dev, fd %d", pLgMaster->fd_laser);
#endif
    while (itest == EAGAIN)
      {
	num = read(pLgMaster->fd_laser, c_out, n);
	if (num < 0)
	  itest = errno;
	else
	  itest = num;
      }
 #ifdef AGS_DEBUG
    int i;
    for (i=0; i < n; i++)
      syslog(LOG_DEBUG,"Level Search read value %x",c_out[i]);
 #endif
    itest = doSetClock(pLgMaster, KETIMER_10M);
    if (itest)
      return itest;
    return 0;
}


int DoLevelSearch(struct lg_master *pLgMaster, struct lg_xydata *pSrchData,
		  struct lg_xydelta *pDeltaData, int16_t n, int16_t *c_out)
{
  int      itest, num;
  uint32_t index;
  char     searchbuff[MAX_TGFIND_BUFFER * sizeof(int16_t)];

  memset((char *)&searchbuff[0], 0, sizeof(searchbuff));
  
#ifdef AGS_DEBUG
  syslog(LOG_DEBUG,"\nLEVELSEARCH: data x=%d y=%d, delta x=%d y=%d",pSrchData->xdata, pSrchData->ydata,pDeltaData->xdata,pDeltaData->ydata);
#endif
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

  while (itest == EAGAIN)
    {
      num = read( pLgMaster->fd_laser, (char *)&searchbuff, 2*n);
      if (num < 0)
	itest = errno;
      else
	itest = num;
    }
  memcpy((char *)c_out, searchbuff, (n * sizeof(uint32_t)));
 #ifdef AGS_DEBUG
  for (index=0; index < n ; index++)
      syslog(LOG_DEBUG,"Level Search read value %x",c_out[index]);
#endif
  doSetClock(pLgMaster, KETIMER_10M);
  return(0);
}

int SearchBeamOn(struct lg_master *pLgMaster)
{
  doSetSearchBeam(pLgMaster);
  return 0;
}

int SearchBeamOff(struct lg_master *pLgMaster)
{
  int rc=0;

  // FIXME---PAH---I don't think we need this next call
  // doClearSearchBeam(pLgMaster);
  // attempt to turn off beam
  // rc = doLGSTOP(pLgMaster);
  //  doClearSearchBeam(pLgMaster);
  return(rc);
}

int doWriteDevCmdNoData(struct lg_master *pLgMaster, uint32_t command)
{
  int    rc=0;

  rc = write(pLgMaster->fd_laser, (char *)&command, sizeof(uint32_t));
  if (rc < 0)
    syslog(LOG_ERR,"\nCMDW-NODATA: cmd %d, ERROR rc%d, errno %d\n", command, rc, errno);
  return(rc);
}
int doLGSTOP(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_STOP));
}
int doROIOff(struct lg_master *pLgMaster)
{
    usleep(250000);
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
  return(doWriteDevCmdNoData(pLgMaster, CMDW_SEARCHBEAMON));
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
  doWriteDevCmdNoData(pLgMaster, CMDW_LINKLEDOFF);
  return;
}
void doSetLinkLED(struct lg_master *pLgMaster)
{
  doWriteDevCmdNoData(pLgMaster, CMDW_LINKLEDON);
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
    syslog(LOG_ERR,"\nCMDW-VAL32: ERROR cmd %d, val %d, rc %d, errno %d\n", command, write_val, rc, errno);
  free(p_cmd_data);
  return(rc);
}
int doWriteDevDelta(struct lg_master *pLgMaster, struct lg_xydelta *pDelta)
{
  int    rc=0;
  struct cmd_rw_base *p_cmd_data;

  p_cmd_data = (struct cmd_rw_base *)malloc(sizeof(struct cmd_rw_base));
  if (!p_cmd_data)
    return(-1);
  
  p_cmd_data->cmd = CMDW_SETDELTA;
  p_cmd_data->xydelta.xdata = pDelta->xdata;
  p_cmd_data->xydelta.ydata = pDelta->ydata;
  p_cmd_data->length = sizeof(struct lg_xydata);
  rc = write(pLgMaster->fd_laser, (char *)p_cmd_data, sizeof(struct cmd_rw_base));
  if (rc < 0)
    syslog(LOG_ERR,"\nCMDW-SETDELTA: ERROR xval %d, yval %d, rc %d, errno %d\n", pDelta->xdata, pDelta->ydata, rc, errno);
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
    syslog(LOG_ERR,"\nCMDW-GOANGLE: ERROR xval %d, yval %d, rc %d, errno %d\n", pPoints->xdata, pPoints->ydata, rc, errno);
  free(p_cmd_data);
  return(rc);
}

void SetHighBeam(struct lg_xydata *pDevXYData)
{
  // Set beam HIGH, set laser ENABLED
  pDevXYData->ctrl_flags = BRIGHTBEAMISSET | BEAMONISSET | LASERENBISSET;
  return;
}
void SetLowBeam(struct lg_xydata *pDevXYData)
{
    // Set beam LOW, keep laser enabled
    pDevXYData->ctrl_flags = BEAMONISSET | LASERENBISSET;
    return;
}
void SetDarkBeam(struct lg_xydata *pDevXYData)
{
    // No beam, keep laser enabled
    pDevXYData->ctrl_flags = LASERENBISSET;
    return;
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
  syslog(LOG_NOTICE, "SetCQ qcCounter %d\n", count);
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
  start_QC_timer = time(NULL);
  end_QC_timer = time(NULL);

  return( 0 );
}

int32_t GetQCtimer( void )
{
            int32_t diff;

            end_QC_timer = time(NULL);

            diff = (int32_t)(end_QC_timer - start_QC_timer);
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
#ifdef AGS_DEBUG
    syslog(LOG_DEBUG,"\nRESTOREBEAM: x=%d,y=%d",pLgMaster->gSaveXY.xdata, pLgMaster->gSaveXY.ydata);
#endif
    doWriteDevPoints(pLgMaster, (struct lg_xydata *)&pLgMaster->gSaveXY);
    usleep( 10000 );
    return;
}

void GoToRaw(struct lg_master *pLgMaster, struct lg_xydata *pRawData)
{
  doSetClock(pLgMaster, KETIMER_10M);
  move_dark(pLgMaster, pRawData);
#ifdef AGS_DEBUG
  syslog(LOG_DEBUG,"\nGOTORAW: x=%d,y=%d",pRawData->xdata, pRawData->ydata);
#endif
  doWriteDevPoints(pLgMaster, pRawData);
  return;
}

//FIXME---PAH---THIS CODE IS NEVER USED
#if 0
void GoToPulse(struct lg_master *pLgMaster, struct lg_xydata *pPulseData,
	       int32_t pulseoffvalue, int32_t pulseonvalue)
{
    pLgMaster->gPeriod = KETIMER_50U;
    SetHighBeam(pPulseData);
    move_lite(pLgMaster, pPulseData);
    doWriteDevPoints(pLgMaster, pPulseData);
    doSetPulseOff(pLgMaster, pulseoffvalue);
    doSetPulseOn(pLgMaster, pulseonvalue);
    doStartPulse(pLgMaster);
    return;
}
#endif
void StopPulse (struct lg_master *pLgMaster)
{
    doStopPulse(pLgMaster);
}

int ROIoff (struct lg_master *pLgMaster)
{
  int rc=0;
  
  SearchBeamOff(pLgMaster);
  rc = doROIOff(pLgMaster);
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
      usleep(250000);
      doClearReadyLED(pLgMaster);
      usleep(250000);
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

void JustDoDisplay(struct lg_master *pLgMaster, char *wr_ptr, int pattern_len)
{
  uint32_t ptn_len;
  
  if (!wr_ptr)
    return;

  // Initialize all buffers about to be used
  memset(tmp_pattern, 0, MAX_LG_BUFFER);
  memset(out_pattern, 0, MAX_LG_BUFFER);

  // Set pattern length established in command arguments
  ptn_len = pattern_len;
  // Send new settings to laser-dev
  doLGSTOP(pLgMaster);
  doSetClock(pLgMaster, KETIMER_10M);
  doStopPulse(pLgMaster);
  ROIoff(pLgMaster);
  setROIlength(pLgMaster, 0);
  stopQCcounter(pLgMaster);   /* never quick check */

  memcpy((char *)tmp_pattern, wr_ptr, ptn_len);

  FillDispBuff(pLgMaster, ptn_len);
  doLoadWriteNum(pLgMaster, ptn_len);
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
    struct lg_xydelta  xydelta;

    /* move slowly from current position to start of pattern */
    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    doLGSTOP(pLgMaster);
    doSetClock(pLgMaster, KETIMER_10M);
    doStopPulse(pLgMaster);
    usleep( 10000 );
    ioctl(pLgMaster->fd_laser, LGGETANGLE, &xydata);
    dx = (double)pNewData->xdata - (double)xydata.xdata;
    dy = (double)pNewData->ydata - (double)xydata.ydata;
    dsqr = dx*dx + dy*dy;
    dlen = sqrt(dsqr);
    if (dlen > pLgMaster->dmax)
      {
	// Just re-use the struct, delta doesn't care about ctrl-flags
	memset((char *)&xydata, 0, sizeof(struct lg_xydata));
	n = dlen / pLgMaster->dmax;
	n += 60;
	xydelta.xdata =  (int16_t)(dx / n);
	xydelta.ydata =  (int16_t)(dy / n);
	doWriteDevDelta(pLgMaster, (struct lg_xydelta *)&xydelta);
	doLoadReadNum(pLgMaster, n);
	doSensor(pLgMaster);
	usleep(2 * (int)n * 150  + 2000);
      }

    doSetClock(pLgMaster, KETIMER_10M);
    /* end of lite move */
  return(0);
}

int move_dark(struct lg_master *pLgMaster, struct lg_xydata *pNewData)
{
  struct lg_xydata  xydata;
  struct lg_xydata  xydelta;
  double            n, dx, dy, dsqr, dlen;

  memset((char *)&xydata, 0, sizeof(struct lg_xydata));
  /* move slowly from current position to start of pattern */
  doLGSTOP(pLgMaster);
  doStopPulse(pLgMaster);
  usleep(20000);
  doSetClock(pLgMaster, pLgMaster->gPeriod);
  ioctl( pLgMaster->fd_laser, LGGETANGLE, &xydata);
  dx = (double)pNewData->xdata - (double)xydata.xdata;
  dy = (double)pNewData->ydata - (double)xydata.ydata;
  dsqr = dx*dx + dy*dy;
  dlen = sqrt(dsqr);
  n = dlen / pLgMaster->dmax;
  n += 20;
  // Just re-use the struct, delta doesn't care about ctrl-flags
  memset((char *)&xydelta, 0, sizeof(struct lg_xydelta));
  xydelta.xdata =  (int16_t)(dx / n);
  xydelta.ydata =  (int16_t)(dy / n);
  xydelta.ctrl_flags = pNewData->ctrl_flags;
  doWriteDevDelta(pLgMaster, (struct lg_xydelta *)&xydelta);
  doLoadReadNum(pLgMaster, n);
  doSetClock(pLgMaster, KETIMER_150U);
  doDarkSensor(pLgMaster);

  // FIXME---PAH---why is this sleep here?
  // And why is clock changed mid operation?
  usleep((2 * (int)n * 150) + 2000);
  doSetClock(pLgMaster, KETIMER_10M);
  /* end of dark move */
  return(0);
}

int CDRHflag(struct lg_master *pLgMaster)
{
  int  optic_status=0;

  ioctl(pLgMaster->fd_laser, LGGETCTL2STAT, &optic_status);
  if (optic_status & CDRHBITMASK)
    { 
      doSetShutterENB(pLgMaster);
      return(0);
    }

  syslog(LOG_NOTICE, "\nCDRHFlag clear, try to CLOSE shutter\n");
  doClearShutterENB(pLgMaster);
  return(1); 
}
double ArrayToDouble(double inp_data)
{
   union
   {
      double ull;
      int8_t  c[8];
   } x;

   int8_t c = 0;
   x.ull = inp_data;
   c = x.c[0]; x.c[0] = x.c[7]; x.c[7] = c;
   c = x.c[1]; x.c[1] = x.c[6]; x.c[6] = c;
   c = x.c[2]; x.c[2] = x.c[5]; x.c[5] = c;
   c = x.c[3]; x.c[3] = x.c[4]; x.c[4] = c;
   return(x.ull);
}


