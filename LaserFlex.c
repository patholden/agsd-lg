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
  unsigned char *tmpPtr;
  uint32_t *p_anglepairs;
  uint32_t *p_transform;
  int      checkQC=0;
  int      error=0;
  int      return_code=0;
  
  gAbortDisplay = false;
  memset((char *)&dispData, 0, sizeof(struct displayData));
  memset((char *)pResp, 0, sizeof(struct parse_basic_resp));
  
  if ( gTransmittedLengthSum < pLgMaster->gDataChunksLength )
    {
      pResp->hdr.status1 = RESPFAIL;
      pResp->hdr.errtype1 = RESPE1APTERROR;
      HandleResponse (pLgMaster, sizeof ( uint32_t ), respondToWhom );
      return;
    }
	
  index  = gPlysReceived * kNumberOfFlexPoints *
                             2 * sizeof ( uint32_t );
  tmpPtr = pLgMaster->gSensorBuffer;
  if (!tmpPtr)
    return;

  tmpPtr += index;
 
  numberOfTargets = parameters->inp_numTargets;
  gQuickCheckTargetNumber[gPlysReceived] = numberOfTargets;
  memmove( tmpPtr, parameters->inp_anglepairs,
	   (size_t)( kNumberOfFlexPoints * 2 * sizeof ( uint32_t ) ) );

  checkQC = 0;
  p_anglepairs = (uint32_t *)&parameters->inp_anglepairs[0];
  for (i = 0; i < numberOfTargets; i++)
    {
      if (0 != p_anglepairs[i])
	checkQC++;
      p_transform = (uint32_t *)&parameters->inp_transform[0];
      if (0 != p_transform[0])
                 checkQC++;
    }
  if (checkQC == 0)
    {
      pLgMaster->gQCcount = -1;
      initQCcounter(pLgMaster);
    }

  if ( gPlysReceived++ )
    {
      error = 0;
      tmpDoubleArr = (double *)&parameters->inp_transform[0];
      for( i=0; i<12; i++ ) {
	if ( isnan(tmpDoubleArr[i]) ) {
	  error = 1;
	}
	if ( isinf(tmpDoubleArr[i]) ) {
	  error = 1;
	}
	if ( i < 9 && (fabs(tmpDoubleArr[i]) > 10.0) ) {
	  error = 1;
	}
      }

      if ( error ) {
	pResp->hdr.status = RESPFAIL;
	pResp->hdr.errtype = RESPOTHERERROR;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	ResetPlyCounter();
	return;
      }
      ChangeTransform( tmpDoubleArr );
    }
  else
    {
      dispData.numberOfSensorSets = gPlysToDisplay;
      dispData.sensorAngles = (uint32_t *)&pLgMaster->gSensorBuffer[0];
      error = 0;
      tmpDoubleArr = (double *)&parameters->inp_transform[0];
      for( i=0; i<12; i++ ) {
	if ( isnan(tmpDoubleArr[i]) ) {
	  error = 1;
	}
	if ( isinf(tmpDoubleArr[i]) ) {
	  error = 1;
	}
	if ( i < 9 && (fabs(tmpDoubleArr[i]) > 10.0) ) {
	  error = 1;
	}
      }
      if ( error ) {
	pResp->hdr.status = RESPFAIL;
	pResp->hdr.errtype = RESPOTHERERROR; 
	HandleResponse (pLgMaster, sizeof ( uint32_t ), respondToWhom );
	ResetPlyCounter();
	return;
      }
      
      SetUpLaserPattern(pLgMaster, tmpDoubleArr);
    }
  
  return_code = ProcessPatternData(pLgMaster, pLgMaster->gDataChunksBuffer, pLgMaster->gDataChunksLength);
  if (return_code)
    {
      pResp->hdr.status = RESPFAIL;
      HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
      ResetFlexPlyCounter();
      return;
    }
  
  if ( gPlysReceived < gPlysToDisplay )
    {
      if ( !(return_code = SetPenUp()) )
	return_code = PendPenDown();
    }
  else
    return_code = FinishPattern (pLgMaster);
  
  if (return_code)
    {
      pResp->hdr.status = RESPFAIL;
      HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
      ResetFlexPlyCounter();
      return;
    }
  else
    {
      if ( gAbortDisplay || ( gPlysReceived < gPlysToDisplay ) )
	{
	  pResp->hdr.status = RESPGOOD;
	  if (!(pLgMaster->gHeaderSpecialByte & 0x80))
	    HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
	  return;
	}
      else
	{
	  if ( (CDRHflag(pLgMaster) )  ) {
	    if (!(pLgMaster->gHeaderSpecialByte & 0x80))
	      {
		pResp->hdr.status =  RESPFAIL; 
		pResp->hdr.errtype1 = RESPE1BOARDERROR; 
		HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
	      }
	    ResetFlexPlyCounter();
	    return;
	  }
	  gVideoCheck = 0;
	  SearchBeamOff(pLgMaster);
	  if ( (CDRHflag(pLgMaster) )  ) {
	    pResp->hdr.status =  RESPFAIL; 
	    pResp->hdr.errtype1 = RESPE1BOARDERROR; 
	    HandleResponse (pLgMaster, sizeof(uint32_t), respondToWhom );
	    ResetFlexPlyCounter();
	    return;
	  }
	  if ( (pLgMaster->gHeaderSpecialByte & 0x80) ) {
	    PostCmdDispNoResp(pLgMaster, (struct displayData *)&dispData, respondToWhom);
	    ResetFlexPlyCounter();
	  } else {
	    PostCmdDisplay(pLgMaster,(struct displayData *)&dispData, respondToWhom);
	    ResetFlexPlyCounter();
	  }
	}
    }
}

void DoFlexDisplay (struct lg_master *pLgMaster, uint32_t dataLength,
		    struct parse_flexdisp_parms *parameters, unsigned char *patternData)
{
  struct displayData dispData;
  struct lg_xydata   xydata;
  double             tmpDoubleArr[12];
  uint32_t           anglePairs[2 * kNumberOfFlexPoints];
  double *currentTransform;
  uint32_t *currentData;
  struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
  int zeroTarget=1;
  int return_code;
  double x, y, z;
  uint32_t i,j,numberOfTargets;

  memset(pResp, 0, sizeof(struct parse_basic_resp));
  memset((char *)&dispData, 0, sizeof(struct displayData));
  memset((char *)&anglePairs[0], 0, sizeof(anglePairs));
  memset((char *)&xydata, 0, sizeof(struct lg_xydata));
  gAbortDisplay = false;

  currentData = (uint32_t*)&parameters->inp_anglepairs[0];
  currentTransform = (double *)&parameters->inp_transform[0];
  numberOfTargets = parameters->inp_numTargets;
  gQuickCheckTargetNumber[gPlysReceived] = numberOfTargets;

  for (i=0; i<MAX_NEW_TRANSFORM_ITEMS; i++)
    tmpDoubleArr[i] = currentTransform[i];

  ChangeTransform((double *)&tmpDoubleArr);
  zeroTarget = 1;
  for(i=0,j=0;  i++ < numberOfTargets; i+=2)
    {
      x = currentData[j];
      y = currentData[j+1];
      z = currentData[j+2];
      return_code = Transform3DPointToBinary(pLgMaster,
					     x, y, z,
					     &anglePairs[i],
					     &anglePairs[i+1]);
      if ((fabs(x) > 0.0001) || (fabs(y) > 0.0001))
	zeroTarget = 0;
      xydata.xdata = anglePairs[i] & kMaxUnsigned;
      xydata.ydata = anglePairs[i+1] & kMaxUnsigned;
      SetHighBeam ((struct lg_xydata *)&xydata);
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
    memmove (&pLgMaster->gSensorBuffer[gPlysReceived *
				       kNumberOfFlexPoints * 2 * sizeof(uint32_t)],
	     (char *)&anglePairs[0],
	     (kNumberOfFlexPoints * 2 * sizeof(uint32_t)));

  if (!gPlysReceived)
    {
      dispData.numberOfSensorSets = gPlysToDisplay;
      dispData.sensorAngles = (uint32_t *)&pLgMaster->gSensorBuffer[0];
      for (i=0; i<MAX_NEW_TRANSFORM_ITEMS; i++)
	tmpDoubleArr[i] = currentTransform[i];
      SetUpLaserPattern(pLgMaster, tmpDoubleArr);
    }
  else
    gPlysReceived++;      
  
  pResp->hdr.errtype = ProcessPatternData(pLgMaster, patternData, dataLength );
  if (pResp->hdr.errtype)
    {
      pResp->hdr.status = RESPFAIL; 
      if (!(pLgMaster->gHeaderSpecialByte & 0x80))
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), 0);
      ResetFlexPlyCounter();
      return;
    }
  
  if ( gPlysReceived < gPlysToDisplay )
    pResp->hdr.errtype = (uint16_t)SetPenUp (  );
  else
    pResp->hdr.errtype = (uint16_t)FinishPattern(pLgMaster);
  
  if (pResp->hdr.errtype)
    {
      pResp->hdr.status = RESPFAIL; 
      if (!(pLgMaster->gHeaderSpecialByte & 0x80))
	HandleResponse(pLgMaster,(sizeof(struct parse_basic_resp)-kCRCSize), 0);
      ResetFlexPlyCounter (  );
    }
  else
    {
      if (gAbortDisplay || (gPlysReceived < gPlysToDisplay))
	{
	  pResp->hdr.status = RESPGOOD;
	  if (!(pLgMaster->gHeaderSpecialByte & 0x80))
	    HandleResponse (pLgMaster,(sizeof(struct parse_basic_resp)-kCRCSize), 0);
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
	      ResetFlexPlyCounter (  );
	      return;
	    }
	  if ( (pLgMaster->gHeaderSpecialByte & 0x80))
	    {
	      PostCmdDispNoResp(pLgMaster, (struct displayData *)&dispData, 0);
	      ResetFlexPlyCounter (  );
	    }
	  else
	    {
	      PostCmdDisplay(pLgMaster, (struct displayData *)&dispData, 0);
	      ResetFlexPlyCounter();
	    }
	}
    }
  return;
}




void DoFlexQuickCheck ( struct lg_master *pLgMaster, struct parse_flexquickcheck_parms* data, uint32_t respondToWhom )
{
        uint32_t nTargets;

        nTargets = data->inp_numTargets;
#ifdef ZDEBUG
        fprintf( stderr, "\nDoQuickCheck\n" );
        for ( i=0; i<2*nTargets; i+=2 ) {
            fprintf( stderr, "DQC " );
            fprintf( stderr, " %8x", ((int *)(data))[1+i] );
            fprintf( stderr, " %8x", ((int *)(data))[1+i+1] );
            fprintf( stderr, "\n" );
        }
        fprintf( stderr, "\n" );
#endif
#ifdef ZDEBUG
        fprintf(stderr, "DFQC647 index %d data %x ptr %x\n", index,data,ptr );
#endif
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
#ifdef ZDEBUG
        fprintf( stderr, "\nDoThresholdQuickCheck\n" );
        for ( i=0; i<2*nTargets; i+=2 ) {
            fprintf( stderr, "DQC " );
            fprintf( stderr, " %8x", ((int *)(data))[2+i] );
            fprintf( stderr, " %8x", ((int *)(data))[2+i+1] );
            fprintf( stderr, "\n" );
        }
        fprintf( stderr, "\n" );
#endif
        index = 2 * sizeof(uint32_t);
        ptr = (char *)(&(data[index]));
#ifdef ZDEBUG
        fprintf(stderr, "DFQC647 index %d data %x ptr %x\n", index,data,ptr );
#endif
        PerformThresholdQuickCheck ( pLgMaster, ptr, nTargets, nThresh );
}

