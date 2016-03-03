//static char rcsid[] = "$Id: LaserCmds.c,v 1.18 2003/04/25 10:40:04 ags-sw Exp ags-sw $";

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <stddef.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include <syslog.h>
#include "BoardComm.h"
#include "AppCommon.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "AppStrListIDs.h"
#include "LaserCmds.h"
#include "AppErrors.h"
#include "L3DTransform.h"
#include "3DTransform.h"
#include "SensorRegistration.h"
#include "LaserInterface.h"
#include "LaserPattern.h"
#include "APTParser.h"
#include "FullRegManager.h"
#include "Video.h"
#include "QuickCheckManager.h"
#include "Protocol.h"
#include "Web.h"


#define _SIKORSKY_DEMO_		0

#if _SIKORSKY_DEMO_
#define	kTheHardCodedHeight	-108.5L
#endif

enum
{
	kInitDataBufferErr = 1,
	kInitializingLaserCmd
};


uint32_t                gPlysReceived = 0;
uint16_t                gPlysToDisplay = 0;
unsigned char		gAbortDisplay = false;

#define MAXLINE 1024
#define BUFFSIZE  (2048+3*512*512)

static char FromVideo[BUFFSIZE];
static char ToVideo[MAXLINE];


#ifdef __unix__
#define gSameEndian false
#else
#define gSameEndian true
#endif

void ResetPlyCounter(void)
{
    // Resets all relevant data for a display to initial values
    gPlysToDisplay = 0;
    gPlysReceived = 0;
    return;
}
void ResetFlexPlyCounter(void)
{
  ResetPlyCounter();
}

static	unsigned char	CheckSourceAndMode ( uint32_t respondToWhom );

unsigned char CheckSourceAndMode ( uint32_t respondToWhom )
{
  respondToWhom = respondToWhom;
  // FIXME---PAH---Why is the function even used???
  //  char theRespBuff[sizeof ( uint32_t ) + kCRCSize];
  return true;
}


void DoDisplayChunksStart (struct lg_master *pLgMaster, struct parse_chunkstart_parms *pInp, uint32_t respondToWhom )
{
  struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
  uint32_t                dataLength;

  // Note this input is in BIG endian format on the wire
  dataLength = ntohl(pInp->apt_len);
  memset(pResp, 0, sizeof(struct parse_basic_resp));
  if (!dataLength || (dataLength > kMaxDataLength))
    {
      pResp->hdr.status = RESPFAIL;
      if (dataLength)
	pResp->hdr.errtype = htons(RESPDATATOOLARGE);
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      return;
    }
  else
    {
#ifdef PATDEBUG
      syslog(LOG_DEBUG,"\nDISPSTART: length %d",dataLength);
#endif
      if (!pLgMaster->gDataChunksBuffer)
	{
#ifdef PATDEBUG
	  syslog(LOG_DEBUG,"\nDISPSTART: Bad buffer pointer");
#endif
	  pResp->hdr.status = RESPFAIL;
	  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	  return;
	}
      memset(pLgMaster->gDataChunksBuffer, 0, kMaxDataLength);
#ifdef PATDEBUG
      syslog(LOG_DEBUG,"\nDISPSTART: MEMSET gDataChunksBuffer %lx, length %d",(long)&pLgMaster->gDataChunksBuffer, dataLength);
#endif
      pLgMaster->gDataChunksLength = dataLength;
      pLgMaster->gTransmitLengthSum = 0;
      pResp->hdr.status = RESPGOOD;
    }
  // If no-response flag is set, don't send response
  if (!(pLgMaster->gHeaderSpecialByte & 0x80))
    {
#ifdef PATDEBUG
      syslog(LOG_DEBUG,"\nDISPSTART: We won't send response");
#endif
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
    }
  return;
}

void DoDisplayChunks(struct lg_master *pLgMaster, struct parse_chunksdo_parms *pInp, uint32_t respondToWhom)
{
  struct displayData  dispData;
  struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
  char *tmpPtr;
  double *p_transform;
  int index;
  int i;
  double tmpDoubleArr[MAX_NEW_TRANSFORM_ITEMS];
  int numberOfTargets;
  int checkQC;
  int error;

  // Do some validations before continuing
  if (pLgMaster->gTransmitLengthSum < pLgMaster->gDataChunksLength)
    {
      pResp->hdr.status1 = RESPFAIL;
      pResp->hdr.errtype1 = RESPE1APTERROR;
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      ResetPlyCounter();
      return;
    }
  if (!pLgMaster->gSensorBuffer)
    {
      pResp->hdr.status1 = RESPFAIL;
      pResp->hdr.errtype1 = RESPE1BOARDERROR;
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      ResetPlyCounter();
      return;
    }
	
  gAbortDisplay = false;
  memset((char *)pResp, 0, sizeof(struct parse_basic_resp));
  memset((char *)&dispData, 0, sizeof(struct displayData));
  
  index  = gPlysReceived * kNumberOfRegPoints * 2 * sizeof(uint32_t);
  tmpPtr = (char *)((char *)pLgMaster->gSensorBuffer + index);
  numberOfTargets = PARSE_MAX_TARGETSOLD;
  gQuickCheckTargetNumber[gPlysReceived] = numberOfTargets;
#ifdef PATDEBUG
  syslog(LOG_ERR,"\nDISPCHUNKS: targets %d, plynum %d, index %x",numberOfTargets,gPlysReceived,index);
#endif
  // Get angle pairs
  memcpy(tmpPtr, pInp->chunk_anglepairs, (kNumberOfRegPoints * 2 * sizeof(uint32_t)));

  //  KLUDGE to prevent quickcheck
  checkQC = 0;
  for (i = 0; i < numberOfTargets; i++)
    {
      // Check pair
      if ((pInp->chunk_anglepairs[i] != 0) || (pInp->chunk_anglepairs[i+1] != 0))
	checkQC++;
    }
  if (checkQC == 0)
    {
      pLgMaster->gQCcount = -1;
      initQCcounter(pLgMaster);
    }

  p_transform = (double *)((char *)pInp + offsetof(struct parse_chunksdo_parms,chunk_transform));
  error = 0;
  for (i=0; i<12; i++)
    {
      tmpDoubleArr[i] = p_transform[i];
      if (isnan(tmpDoubleArr[i]))
	error = 1;
      if (isinf(tmpDoubleArr[i]))
	error = 1;
      if ((i < 9) && (fabs(tmpDoubleArr[i]) > 10.0))
	error = 1;
    }
  if (error)
    {
      pResp->hdr.status = RESPFAIL;
      pResp->hdr.errtype = RESPOTHERERROR; 
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp) -kCRCSize), respondToWhom);
      ResetPlyCounter();
      return;
    }
  if (gPlysReceived++)
    ChangeTransform((double *)&tmpDoubleArr);
  else
    {
      dispData.numberOfSensorSets = gPlysToDisplay;
      dispData.sensorAngles = (int32_t *)pLgMaster->gSensorBuffer;
      dispData.pattern = SetUpLaserPattern(pLgMaster, tmpDoubleArr);
    }
  pResp->hdr.errtype = ProcessPatternData(pLgMaster, pLgMaster->gDataChunksBuffer, pLgMaster->gDataChunksLength);
  if (pResp->hdr.errtype)
    {
      pResp->hdr.status = RESPFAIL;
      pResp->hdr.errtype = pResp->hdr.errtype;
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      ResetPlyCounter();
      return;
    }
  
  if (gPlysReceived < gPlysToDisplay)
    {
      SetPenUp();
      PendPenDown();
      pResp->hdr.errtype = 0;
    }
  else
    pResp->hdr.errtype = (uint16_t)FinishPattern(pLgMaster);

  if (pResp->hdr.errtype)
    {
      pResp->hdr.status  = RESPFAIL;
      pResp->hdr.errtype = htons(pResp->hdr.errtype);
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      ResetPlyCounter();
      return;
    }
  if (gAbortDisplay || (gPlysReceived < gPlysToDisplay))
    {
      pResp->hdr.status  = RESPGOOD;
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      ResetPlyCounter();
      return;
    }
  else
    {
      if ((CDRHflag(pLgMaster)))
	{
	  pResp->hdr.status1 = RESPFAIL;
	  pResp->hdr.errtype1 = RESPE1BOARDERROR;
	  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
	  ResetPlyCounter();
	  return;
	}
      gVideoCheck = 0;
      SearchBeamOff(pLgMaster);
      if ((CDRHflag(pLgMaster)))
	{
	  pResp->hdr.status1 = RESPFAIL;
	  pResp->hdr.errtype1 = RESPE1BOARDERROR; 
	  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	  ResetPlyCounter();
	  return;
	}
#ifdef PATDEBUG
      syslog(LOG_DEBUG,"\nPostCmdDisp from DoDisplayChunks");
#endif
      PostCmdDisplay(pLgMaster, (struct displayData *)&dispData, SENDRESPONSE, respondToWhom);
      ResetPlyCounter();
    }
  return;
}
	
void AddDisplayChunksData(struct lg_master *pLgMaster, uint32_t dataLength,
			  uint32_t dataOffset, char *patternData, uint32_t respondToWhom)
{
    struct displayData     dispData;
    struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
  
    memset((char *)pResp, 0, sizeof(struct parse_basic_resp));
    memset((char *)&dispData, 0, sizeof(struct displayData));
  
    if ((dataLength + dataOffset) > pLgMaster->gDataChunksLength)
      {
#ifdef PATDEBUG
	syslog(LOG_DEBUG,"\nADDCHUNKS error1, length %d, offset %d",dataLength, dataOffset);
#endif
	pResp->hdr.status = RESPFAIL;
	pResp->hdr.errtype = htons(RESPDATATOOLARGE);
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	return;
      }
    pLgMaster->gTransmitLengthSum += dataLength;
    if (pLgMaster->gTransmitLengthSum > pLgMaster->gDataChunksLength)
      {
#ifdef PATDEBUG
	syslog(LOG_DEBUG,"\nADDCHUNKS error2, transmitted %d, chunklen %d",pLgMaster->gTransmitLengthSum, pLgMaster->gDataChunksLength);
#endif
	pResp->hdr.status = RESPFAIL;
	pResp->hdr.errtype = htons(RESPDATATOOLARGE);
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	return;
      }
    memcpy((char *)(pLgMaster->gDataChunksBuffer + dataOffset), patternData, dataLength);
    pResp->hdr.status = RESPGOOD;
#ifdef PATDEBUG
    syslog(LOG_DEBUG,"\nADDCHUNKS good response, special byte %d",pLgMaster->gHeaderSpecialByte);
#endif
    if (!(pLgMaster->gHeaderSpecialByte & 0x80))
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
    return;
}

void DoStop(struct lg_master *pLgMaster, uint32_t respondToWhom)
{
    PostCommand(pLgMaster, kStop, 0, respondToWhom);
    return;
}


void DoGoAngle (struct lg_master *pLgMaster, struct parse_goangle_parms *pInp, uint32_t respondToWhom )
{
  struct lg_xydata xydata;
  int32_t xPoint=0;
  int32_t yPoint=0;
  double   x;
  double   y;
  int      return_val;
  struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;

  memset(pResp, 0, sizeof(struct parse_basic_resp));
  if ( !CheckSourceAndMode ( respondToWhom ) ) return;

  // Check input parms pointer & get x/y data
  if (!pInp)
    return;
  x = pInp->xData;
  y = pInp->yData;
  return_val = ConvertExternalAnglesToBinary(pLgMaster, x, y,
					     &xPoint, &yPoint);
  if (return_val)
    {
      pResp->hdr.status1 = RESPFAIL;
      pResp->hdr.errtype1 = RESPE1INANGLEOUTOFRANGE;
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
    }
  else
    {
      if ((CDRHflag(pLgMaster)))
	{
	  pResp->hdr.status1 =RESPFAIL;
	  pResp->hdr.errtype1 = RESPE1BOARDERROR; 
	  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	  return;
	}
      xydata.xdata = (int16_t)xPoint & kMaxSigned;
      xydata.ydata = (int16_t)yPoint & kMaxSigned;
      PostCmdGoAngle(pLgMaster, (struct lg_xydata *)&xydata, respondToWhom );
    }
  return;
}

void DoEtherAngle (struct lg_master *pLgMaster, struct parse_ethangle_parms *pInp, uint32_t respondToWhom )
{
  struct lg_xydata xydata;
  int32_t xPoint=0;
  int32_t yPoint=0;
  double   x, y;
  int      return_val;
  struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
	
  memset((char *)pResp, 0, sizeof(struct parse_basic_resp));
  memset((char *)&xydata, 0, sizeof(struct lg_xydata));
  if ( !CheckSourceAndMode ( respondToWhom ) ) return;

  if (!pInp)
    return;
  x = pInp->xData;
  y = pInp->yData;
  return_val =  ConvertExternalAnglesToBinary(pLgMaster, x, y, &xPoint, &yPoint);
  if (return_val)
    {
      pResp->hdr.status1 = RESPFAIL;
      pResp->hdr.errtype1 = RESPE1INANGLEOUTOFRANGE;
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
    }
  else
    {
      if ((CDRHflag(pLgMaster)))
	{
	  pResp->hdr.status1 = RESPFAIL;
	  pResp->hdr.errtype1 = RESPE1BOARDERROR;
	  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	  return;
	}
      xydata.xdata = (int16_t)(xPoint & kMaxSigned);
      xydata.ydata = (int16_t)(yPoint & kMaxSigned);
      PostCmdEtherAngle(pLgMaster, (struct lg_xydata *)&xydata, respondToWhom);
    }
  return;
}
void DoFullReg (struct lg_master *pLgMaster, 
		struct parse_dofullreg_parms *pInp, 
		uint32_t respondToWhom )
{
	double theCoordinateBuffer[kNumberOfRegPoints * 3];
	int32_t theAngleBuffer[kNumberOfRegPoints * 2];
	struct lg_xydata *pXYData;
	struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
	double    *currentData;
	int       return_val;
	short     i;
	unsigned char dontBother;

	memset(pResp, 0, sizeof(struct parse_basic_resp));
	memset((char *)&theAngleBuffer, 0, sizeof(theAngleBuffer));
	
	if ( !CheckSourceAndMode ( respondToWhom ) ) return;

	currentData = (double *)&pInp->inp_tgt_angles[0];
	i = 0;
	while ( i++ < kNumberOfRegPoints )
	{
	  return_val = ConvertExternalAnglesToBinary(pLgMaster, currentData[0],
						     currentData[1],
						     &theAngleBuffer[i],
						     &theAngleBuffer[i+1]);
	  if (return_val)
	    {
	      pResp->hdr.status1 = RESPFAIL;
	      pResp->hdr.errtype1 = RESPE1INANGLEOUTOFRANGE;
	      switch ( i )
		{
		case 1:
		  pResp->hdr.errtype2 = kFirstSensor;
		  break;
		case 2:
		  pResp->hdr.errtype2 = kSecondSensor;
		  break;
		case 3:
		  pResp->hdr.errtype2 = kThirdSensor;
		  break;
		case 4:
		  pResp->hdr.errtype2 = kFourthSensor;
		  break;
		case 5:
		  pResp->hdr.errtype2 = kFifthSensor;
		  break;
		case 6:
		  pResp->hdr.errtype2 = kSixthSensor;
		  break;
		}
	      pResp->hdr.errtype2 = htons(pResp->hdr.errtype2);
	      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	      return;
	    }
	  ConvertToNumber(&theAngleBuffer[i], &theAngleBuffer[i+1]);
	  pXYData = (struct lg_xydata *)&theAngleBuffer[i];
	  SetHighBeam(pXYData);
	  currentData += 2;
	}
	currentData = (double *)pInp->inp_targets;
	i = 0;
	dontBother = true;
	while ( i < ( kNumberOfRegPoints * 3 ) )
	{
		if ( dontBother )
			dontBother = false;
		theCoordinateBuffer[i] = (double)DoubleFromCharConv
			( (unsigned char *)currentData++ );
		FromAPTtoInch ( &theCoordinateBuffer[i++] );
	}
	DontFindTransform ( dontBother );
	if ( dontBother ) return;
	SaveFullRegCoordinates ( kNumberOfRegPoints, theCoordinateBuffer );

        gRespondToWhom = respondToWhom;

        if ( (CDRHflag(pLgMaster) )  ) {
	  pResp->hdr.status1 = RESPFAIL;
	  pResp->hdr.errtype1 = RESPE1BOARDERROR;
	  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	  return;
        }
        PerformAndSendFullReg( pLgMaster, (char *)theAngleBuffer, gRespondToWhom );
	return;
}
void DoDisplayKitVideo (struct lg_master *pLgMaster, uint32_t dataLength,
			unsigned char *otherParameters, char *patternData,
			uint32_t respondToWhom)
{
    short i;
    double tmpDoubleArr[12];
    struct displayData dispData;
    struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
    int index;
    unsigned char * tmpPtr;
    int32_t   tmpLong;
    transform   invTransform, CurrentTransform;
    int count;
    double inputPoint[3], outputPoint[3];

    // Init all buffers
    memset((char *)&inputPoint, 0, sizeof(inputPoint));
    memset((char *)&outputPoint, 0, sizeof(outputPoint));
    memset((char *)&tmpDoubleArr, 0, sizeof(tmpDoubleArr));
    memset(pResp, 0, sizeof(struct parse_basic_resp));
    memset((char *)&dispData, 0, sizeof(struct displayData));
    
    index = 0;
    tmpPtr = &otherParameters[index];
    ArrayIntoTransform ( (double *)tmpPtr, &CurrentTransform  );
    InvertTransform ( &CurrentTransform, &invTransform );

    index =
                   12 * kSizeOldLongDouble
                 ;
    tmpPtr = &otherParameters[index];
    tmpLong = *(int32_t *)tmpPtr;
    if ( tmpLong > 0 ) {
       gVideoPreDwell = tmpLong;
    } else {
       gVideoPreDwell = 1000;
    }
#if defined(SDEBUG) || defined(KITDEBUG)
    syslog(LOG_NOTICE, "gVideoPreDwell %d\n", gVideoPreDwell );
#endif


    index =   
                   12 * kSizeOldLongDouble
              +         sizeof ( int32_t )
                 ;
    tmpPtr = &otherParameters[index];
    tmpLong = *(int32_t *)tmpPtr;
    if ( tmpLong > 0 ) {
       gVideoCount = tmpLong;
    } else {
       gVideoCount = 5000;
    }
#if defined(SDEBUG) || defined(KITDEBUG)
    syslof(LOG_NOTICE, "gVideoCount %d\n", gVideoCount );
#endif
    

    index =     12 * kSizeOldLongDouble
              +      sizeof ( int32_t )
              +      sizeof ( int32_t )
              ;
    tmpPtr = &otherParameters[index];

    inputPoint[0] = ((double *)tmpPtr)[0];
    inputPoint[1] = ((double *)tmpPtr)[1];
    inputPoint[2] = ((double *)tmpPtr)[2];
#if defined(SDEBUG) || defined(KITDEBUG)
    syslog(LOG_NOTICE, "XYZ %.5lf %.lf %.lf\n",((double *)tmpPtr)[0],
	   ((double *)tmpPtr)[1], ((double *)tmpPtr)[3]);
#endif
    TransformPoint ( &CurrentTransform, inputPoint, outputPoint );
#if defined(SDEBUG) || defined(KITDEBUG)
    double Xpoint, Ypoint, Zpoint;
    Xpoint = outputPoint[0];
    Ypoint = outputPoint[1];
    Zpoint = outputPoint[2];
    syslog(LOG_NOTICE, "check XYZ  %.5lf %.5lf %.5lf",outputPoint[0],
	     outputPoint[1], outputPoint[2]);
#endif

    PointToBinary(pLgMaster, outputPoint, &pLgMaster->gXcheck, &pLgMaster->gYcheck );

#if defined(SDEBUG) || defined(KITDEBUG)
    syslog(LOG_NOTICE, "check XY raw %x %x", pLgMaster->gXcheck, pLgMaster->gYcheck);
#endif

	gAbortDisplay = false;

	if ( !CheckSourceAndMode ( respondToWhom ) ) return;
	
	index  = 0;
	tmpPtr = &otherParameters[index];
	for (i=0; i<12; i++)
	  tmpDoubleArr[i] = (double)(((double *)tmpPtr)[i]);
	ChangeTransform((double *)&tmpDoubleArr);
	
	dispData.numberOfSensorSets = 0;
	dispData.sensorAngles = (int32_t *)pLgMaster->gSensorBuffer;
	dispData.pattern = SetUpLaserPattern(pLgMaster, tmpDoubleArr);
	pResp->hdr.errtype = ProcessPatternData(pLgMaster, patternData, dataLength);	
 	if (pResp->hdr.errtype)
	  {
	    pResp->hdr.status = RESPFAIL;
	    pResp->hdr.errtype = htons(pResp->hdr.errtype);
	    HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	    ResetPlyCounter();
	    return;
	  }
 	
#if defined(ZDEBUG) || defined(KITDEBUG)
	syslog(LOG_NOTICE, "1555 plys rcvd %d  disp %d", gPlysReceived, gPlysToDisplay);
#endif
 	if (gPlysReceived < gPlysToDisplay)
	  {
	    SetPenUp();
	    pResp->hdr.errtype = 0;
	  }
 	else
	  pResp->hdr.errtype = (uint16_t)FinishPattern(pLgMaster);
 
 	if (pResp->hdr.errtype)
 	{
	  pResp->hdr.status = RESPFAIL; 
	  pResp->hdr.errtype = htons(pResp->hdr.errtype);
	  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	  ResetPlyCounter();
 	}
 	else
 	{
	  if ( gAbortDisplay || ( gPlysReceived < gPlysToDisplay ) )
	    {
#if defined(ZDEBUG) || defined(KITDEBUG)
	      syslog(LOG_NOTICE, "1574 plys rcvd %d  disp %d", gPlysReceived, gPlysToDisplay);
#endif
	      pResp->hdr.status = RESPGOOD;
	      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	    }
	  else
	    {
                        gQuickCheck = 0;
                        gVideoCheck = 1;
 			SearchBeamOff(pLgMaster);

			count = sprintf( ToVideo,
				      "GET  /cgi-bin/reset"
				      "   HTTP/1.0\n\n"
				       );
#if defined(ZDEBUG) || defined(KITDEBUG)
			int slen;
			slen = GetWebVideo(pLgMaster, ToVideo, count, FromVideo );
			syslog(LOG_NOTICE, "1634 getweb slen %d gVideoCount %d\n", slen, gVideoCount);
#else
			GetWebVideo(pLgMaster, ToVideo, count, FromVideo );
#endif

                        if ( (CDRHflag(pLgMaster) )  ) {
			  pResp->hdr.status1 = RESPFAIL;
			  pResp->hdr.errtype1 = RESPE1BOARDERROR;
			  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
			  ResetPlyCounter();
			  return;
                        }
#if defined(KITDEBUG)
			syslog(LOG_NOTICE, "LC1651 setVideoCount %d\n", gVideoCount );
#endif
			SetQCcounter(pLgMaster, gVideoCount );
                        gQCtimer = -1;
#ifdef PATDEBUG
			syslog(LOG_DEBUG,"\nPostCmdDisp from DoDisplayKitVideo");
#endif
			PostCmdDisplay(pLgMaster, (struct displayData *)&dispData, SENDRESPONSE, respondToWhom );
 			ResetPlyCounter();
 		}
  	}
}

void AbortDisplay ( void )
{
	gAbortDisplay = true;
}


void SetDisplaySeveral(struct lg_master *pLgMaster, uint32_t number, uint32_t respondToWhom )
{
  struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;

  memset(pResp, 0, sizeof(struct parse_basic_resp));
  if (number > (kMaxNumberOfPlies * kNumberOfFlexPoints * 2 * sizeof(uint32_t)))
    {
      pResp->hdr.status = RESPFAIL;
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      return;
    }
  memset((char *)&pLgMaster->gOutOfRange, 0, sizeof(struct k_header)); 
  ResetPlyCounter();
  gPlysToDisplay = (uint16_t)number;

  // Clear buffer for display data
  memset(pLgMaster->gSensorBuffer, 0, (kNumberOfFlexPoints * 2 * gPlysToDisplay * sizeof(uint32_t)));
  // Try to open shutter for display operation here, optic status is checked later
  // during actual display operation
  doSetShutterENB(pLgMaster);

  pResp->hdr.status = RESPGOOD;
  if (!(pLgMaster->gHeaderSpecialByte & 0x80))
    HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
  return;
}

void DoQuickCheck (struct lg_master *pLgMaster, char * angles, uint32_t respondToWhom )
{

	if ( !CheckSourceAndMode ( respondToWhom ) ) return;
	gRespondToWhom = respondToWhom;
        PerformAndSendQuickCheck (pLgMaster, angles, 6 );  // must now set target number
}

void CloseLaserCommands (void)
{
	ResetPlyCounter();
}

double DoubleFromCharConv ( unsigned char *theChar )
{
  double   return_val=0;

  memcpy((char *)&return_val, theChar, sizeof(double));
  return(return_val);
}

void ShortConv ( unsigned char *theChar )
{
#if GENERATING68K
	return;
#else
	unsigned char theThing [ sizeof ( uint32_t ) ];
	unsigned char *thing = theThing;
	unsigned short i;
	if ( gSameEndian ) return;
	memcpy(thing, theChar, sizeof(uint16_t));
#if 0
	*(unsigned short *)thing = *(unsigned short *)theChar;
#endif
	i = sizeof ( unsigned short );
	thing += i;
	while ( i-- ) *(theChar++) = *(--thing);
	return;
#endif
}

void InitLaserCommands ( void )
{
	ResetPlyCounter();
}
