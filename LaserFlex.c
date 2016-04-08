/*
static char rcsid[] = "$Id: LaserFlex.c,v 1.18 2003/04/25 10:40:04 ags-sw Exp ags-sw $";

*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
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
/* #include "AppWindows.h" */
#include "AppStrListIDs.h"
#include "LaserCmds.h"
#include "AppErrors.h"
#include "3DTransform.h"
#include "SensorRegistration.h"
#include "LaserInterface.h"
#include "LaserPattern.h"
#include "APTParser.h"
#include "FullRegManager.h"
#include "Video.h"
#include "QuickCheckManager.h"

#define _SIKORSKY_DEMO_		0

#if _SIKORSKY_DEMO_
#define	kTheHardCodedHeight	-108.5L
#endif

enum
{
	kInitDataBufferErr = 1,
	kInitializingLaserCmd
};

#define MAXLINE 1024
#define BUFFSIZE  (2048+3*512*512)

#ifdef __unix__
#define gSameEndian false
#else
#define gSameEndian true
#endif

void DoFlexDisplayChunks (struct lg_master *pLgMaster,
			  struct parse_chunkflex_parms *parameters,
			  uint32_t respondToWhom )
{
    struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
    struct displayData dispData;
    int      i,index;
    int      numberOfTargets;
    double   *tmpDoubleArr;
    char     *tmpPtr;
    int32_t *p_anglepairs;
    int32_t *p_transform;
    int      checkQC=0;
    int      error=0;
  
    pLgMaster->gAbortDisplay = false;
    memset((char *)&dispData, 0, sizeof(struct displayData));
    memset((char *)pResp, 0, sizeof(struct parse_basic_resp));
  
    // Do some validations before continuing
    if (pLgMaster->gTransmitLengthSum < pLgMaster->gDataChunksLength )
      {
	pResp->hdr.status1 = RESPFAIL;
	pResp->hdr.errtype1 = RESPE1APTERROR;
	HandleResponse (pLgMaster, sizeof ( uint32_t ), respondToWhom );
	ResetFlexPlyCounter(pLgMaster);
	return;
      }
    if (!pLgMaster->gSensorBuffer)
      {
	pResp->hdr.status1 = RESPFAIL;
	pResp->hdr.errtype1 = RESPE1BOARDERROR;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	ResetFlexPlyCounter(pLgMaster);
	return;
      }
    numberOfTargets = parameters->inp_numTargets;
    if (numberOfTargets > MAX_ANGLEPAIRS)
      {
	pResp->hdr.status1 = RESPFAIL;
	pResp->hdr.errtype1 = RESPTOOMANYPLIES;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	ResetFlexPlyCounter(pLgMaster);
	return;
      }
    
    index  = pLgMaster->gPlysReceived * kNumberOfFlexPoints *
      2 * sizeof ( uint32_t );

    tmpPtr = pLgMaster->gSensorBuffer + index;
    gQuickCheckTargetNumber[pLgMaster->gPlysReceived] = numberOfTargets;
#ifdef AGS_DEBUG
    syslog(LOG_ERR,"\nFLEXDISPCHUNKS: targets %d, plynum %d, index %x",numberOfTargets,pLgMaster->gPlysReceived,index);
#endif
    memcpy(tmpPtr, parameters->inp_anglepairs, (kNumberOfFlexPoints * 2 * sizeof(uint32_t)));

    checkQC = 0;
    p_anglepairs = (int32_t *)&parameters->inp_anglepairs[0];
    for (i = 0; i < numberOfTargets; i++)
      {
	if (p_anglepairs[i])
	  checkQC++;
	p_transform = (int32_t *)&parameters->inp_transform[i];
	if (p_transform[i])
	  checkQC++;
      }
    if (checkQC == 0)
      {
	pLgMaster->gQCcount = -1;
	initQCcounter(pLgMaster);
      }

    tmpDoubleArr = (double *)&parameters->inp_transform[0];
    error = 0;
    for (i=0; i < 12; i++)
      {
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
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	ResetFlexPlyCounter(pLgMaster);
	return;
      }
    if (pLgMaster->gPlysReceived++)
      ChangeTransform((double *)&tmpDoubleArr);
    else
    {
      // All is good, do display
      dispData.numberOfSensorSets = pLgMaster->gPlysToDisplay;
      dispData.sensorAngles = (int32_t *)pLgMaster->gSensorBuffer;
      dispData.pattern = SetUpLaserPattern(pLgMaster, tmpDoubleArr);
    }
  
  pResp->hdr.errtype = ProcessPatternData(pLgMaster, pLgMaster->gDataChunksBuffer, pLgMaster->gDataChunksLength);
  if (pResp->hdr.errtype)
    {
      pResp->hdr.status = RESPFAIL;
      pResp->hdr.errtype = pResp->hdr.errtype;
      HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
      ResetFlexPlyCounter(pLgMaster);
      return;
    }
  
  if (pLgMaster->gPlysReceived < pLgMaster->gPlysToDisplay)
    {
      SetPenUp();
      PendPenDown();
      pResp->hdr.errtype = 0;
    }
  else
    pResp->hdr.errtype = (uint16_t)FinishPattern(pLgMaster);
  
  if (pResp->hdr.errtype)
    {
      pResp->hdr.status = RESPFAIL;
      pResp->hdr.errtype = htons(pResp->hdr.errtype);
      HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
      ResetFlexPlyCounter(pLgMaster);
      return;
    }
  if (pLgMaster->gAbortDisplay || (pLgMaster->gPlysReceived < pLgMaster->gPlysToDisplay))
    {
      pResp->hdr.status = RESPGOOD;
      if (!(pLgMaster->gHeaderSpecialByte & 0x80))
	HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
      ResetFlexPlyCounter(pLgMaster);
      return;
    }
  else
    {
      if ((CDRHflag(pLgMaster)))
	{
	  if (!(pLgMaster->gHeaderSpecialByte & 0x80))
	    {
	      pResp->hdr.status =  RESPFAIL; 
	      pResp->hdr.errtype1 = RESPE1BOARDERROR; 
	      HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
	    }
	  ResetFlexPlyCounter(pLgMaster);
	  return;
	}
      gVideoCheck = 0;
      SearchBeamOff(pLgMaster);
      if ((CDRHflag(pLgMaster)))
	{
	  pResp->hdr.status =  RESPFAIL; 
	  pResp->hdr.errtype1 = RESPE1BOARDERROR; 
	  HandleResponse (pLgMaster, sizeof(uint32_t), respondToWhom );
	  ResetFlexPlyCounter(pLgMaster);
	  return;
	}
      if ((pLgMaster->gHeaderSpecialByte & 0x80))
	{
	  PostCmdDisplay(pLgMaster, (struct displayData *)&dispData, DONTRESPOND, respondToWhom);
	  ResetFlexPlyCounter(pLgMaster);
	}
      else
	{
	  PostCmdDisplay(pLgMaster,(struct displayData *)&dispData, SENDRESPONSE, respondToWhom);
	  ResetFlexPlyCounter(pLgMaster);
	}
    }
  return;
}

void DoFlexDisplay (struct lg_master *pLgMaster, uint32_t dataLength,
		    struct parse_flexdisp_parms *parameters, char *patternData)
{
  struct displayData dispData;
  double             tmpDoubleArr[12];
  struct lg_xydata   anglePairs[2 * kNumberOfFlexPoints];
  struct lg_xydata   *pCurXY;
  double *currentTransform;
  int32_t *currentData;
  char    *tmpPtr;
  struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
  int    zeroTarget=1;
  int    index;
  int    return_code;
  double x, y, z;
  uint32_t i,j,numberOfTargets;

  memset(pResp, 0, sizeof(struct parse_basic_resp));
  memset((char *)&dispData, 0, sizeof(struct displayData));
  memset((char *)&anglePairs[0], 0, sizeof(anglePairs));
  pLgMaster->gAbortDisplay = false;

  currentData = (int32_t*)&parameters->inp_anglepairs[0];
  currentTransform = (double *)&parameters->inp_transform[0];
  numberOfTargets = parameters->inp_numTargets;
  gQuickCheckTargetNumber[pLgMaster->gPlysReceived] = numberOfTargets;

  for (i=0; i<MAX_NEW_TRANSFORM_ITEMS; i++)
    tmpDoubleArr[i] = currentTransform[i];

  ChangeTransform((double *)&tmpDoubleArr);
  zeroTarget = 1;
  for(i=0,j=0;  i++ < numberOfTargets; i++)
    {
      pCurXY = (struct lg_xydata *)&anglePairs[i];
      x = currentData[j];
      y = currentData[j+1];
      z = currentData[j+2];
      return_code = Transform3DPointToBinary(pLgMaster,
					     x, y, z,
					     &pCurXY->xdata,
					     &pCurXY->ydata);
      if ((fabs(x) > 0.0001) || (fabs(y) > 0.0001))
	zeroTarget = 0;
      SetHighBeam (pCurXY);
      if (return_code)
	{
	  pResp->hdr.status = RESPFAIL;
	  pResp->hdr.errtype1 = RESPE1INANGLEOUTOFRANGE;
	  pResp->hdr.errtype2 = (uint16_t)return_code & 0xFFFF;
	  if (!(pLgMaster->gHeaderSpecialByte & 0x80))
	    HandleResponse (pLgMaster,(sizeof(struct parse_basic_resp)-kCRCSize), 0);
	  return;
	}
	  j += 3;
    }
	
  if (zeroTarget)
    memset((char *)&anglePairs[0], 0, sizeof(anglePairs));
  else
    {
      index  = pLgMaster->gPlysReceived * kNumberOfFlexPoints * 2 * sizeof(uint32_t);
      tmpPtr = (char *)((char *)pLgMaster->gSensorBuffer + index);
      memmove(tmpPtr, (char *)&anglePairs[0], (kNumberOfFlexPoints * 2 * sizeof(uint32_t)));
    }

  if (!pLgMaster->gPlysReceived)
    {
      dispData.numberOfSensorSets = pLgMaster->gPlysToDisplay;
      dispData.sensorAngles = (int32_t *)pLgMaster->gSensorBuffer;
      for (i=0; i<MAX_NEW_TRANSFORM_ITEMS; i++)
	tmpDoubleArr[i] = currentTransform[i];
      dispData.pattern = SetUpLaserPattern(pLgMaster, tmpDoubleArr);
    }
  pLgMaster->gPlysReceived++;      
  pResp->hdr.errtype = ProcessPatternData(pLgMaster, patternData, dataLength );
  if (pResp->hdr.errtype)
    {
      pResp->hdr.status = RESPFAIL; 
      if (!(pLgMaster->gHeaderSpecialByte & 0x80))
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), 0);
      ResetFlexPlyCounter(pLgMaster);
      return;
    }
  
  if (pLgMaster->gPlysReceived < pLgMaster->gPlysToDisplay)
    {
      SetPenUp();
      pResp->hdr.errtype = 0;
    }
  else
    pResp->hdr.errtype = (uint16_t)FinishPattern(pLgMaster);
  
  if (pResp->hdr.errtype)
    {
      pResp->hdr.status = RESPFAIL; 
      if (!(pLgMaster->gHeaderSpecialByte & 0x80))
	HandleResponse(pLgMaster,(sizeof(struct parse_basic_resp)-kCRCSize), 0);
      ResetFlexPlyCounter(pLgMaster);
      return;
    }
  else
    {
      if (pLgMaster->gAbortDisplay || (pLgMaster->gPlysReceived < pLgMaster->gPlysToDisplay))
	{
	  pResp->hdr.status = RESPGOOD;
	  if (!(pLgMaster->gHeaderSpecialByte & 0x80))
	    HandleResponse (pLgMaster,(sizeof(struct parse_basic_resp)-kCRCSize), 0);
	  ResetFlexPlyCounter(pLgMaster);
	  return;
	}
      else
	{
	  gVideoCheck = 0;
	  SearchBeamOff(pLgMaster);
	  if ( (CDRHflag(pLgMaster)))
	    {
	      pResp->hdr.status1 = RESPFAIL;
	      pResp->hdr.errtype1 = RESPE1BOARDERROR; 
	      HandleResponse (pLgMaster,sizeof ( uint32_t ), 0 );
	      ResetFlexPlyCounter(pLgMaster);
	      return;
	    }
	  if ( (pLgMaster->gHeaderSpecialByte & 0x80))
	    PostCmdDisplay(pLgMaster, (struct displayData *)&dispData, SENDRESPONSE, 0);
	  else
	    PostCmdDisplay(pLgMaster, (struct displayData *)&dispData, DONTRESPOND, 0);
	  ResetFlexPlyCounter(pLgMaster);
	}
    }
  return;
}




void DoFlexQuickCheck ( struct lg_master *pLgMaster, struct parse_flexquickcheck_parms* data, uint32_t respondToWhom )
{
        uint32_t nTargets;

        nTargets = data->inp_numTargets;
        PerformAndSendQuickCheck ( pLgMaster, (char *)data->inp_anglepairs, nTargets );
}


// FIXME---PAH---NEEDS cmd/resp structs
void DoThresholdQuickCheck (struct lg_master *pLgMaster, char * data, uint32_t respondToWhom )
{
        int index;
        uint32_t nTargets;
        uint32_t nThresh;
        char * ptr;

        index = 0;
        nThresh  = *((uint32_t *)&(data[index]));
        index    = sizeof(uint32_t);
        nTargets = *((uint32_t *)&(data[index]));
        index = 2 * sizeof(uint32_t);
        ptr = (char *)(&(data[index]));
        PerformThresholdQuickCheck ( pLgMaster, ptr, nTargets, nThresh );
}

