#include <stdint.h>
//static char rcsid[] = "$Id: LaserCmds.c,v 1.18 2003/04/25 10:40:04 ags-sw Exp ags-sw $";

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
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
uint32_t		gTransmittedLengthSum = 0U;
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

void	ResetPlyCounter ( void )
{
	gPlysToDisplay = 0;
	gPlysReceived = 0;
}

static	unsigned char	CheckSourceAndMode ( uint32_t respondToWhom );

unsigned char CheckSourceAndMode ( uint32_t respondToWhom )
{
  respondToWhom = respondToWhom;
  // FIXME---PAH---Why is the function even used???
  //  char theRespBuff[sizeof ( uint32_t ) + kCRCSize];
  return true;
}

void ResetFlexPlyCounter(void)
{
  gPlysToDisplay = 0;
  gPlysReceived = 0;
}

void DoDisplayChunksStart (struct lg_master *pLgMaster, struct parse_chunkstart_parms *pInp, uint32_t respondToWhom )
{
  struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
  uint32_t                dataLength;

  dataLength = ntohl(pInp->apt_len);
  memset(pResp, 0, sizeof(struct parse_basic_resp));
  if (!dataLength || (dataLength > kMaxDataLength))
    {
      pResp->hdr.status = RESPFAIL;
      if (dataLength)
	pResp->hdr.errtype = htons(RESPDATATOOLARGE);
    }
  else
    {
      pLgMaster->gDataChunksLength = dataLength;
      gTransmittedLengthSum = 0U;
      pResp->hdr.status = RESPGOOD;
    }
  // If no-response flag is set, don't send response
  if (!(pLgMaster->gHeaderSpecialByte & 0x80))
    HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
  return;
}

void DoDisplayChunks(struct lg_master *pLgMaster, struct parse_chunksdo_parms *pInp, uint32_t respondToWhom)
{
  struct displayData  dispData;
  struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
  char *tmpPtr;
  char *tmpInp;
  double *p_transform;
  uint32_t *p_angles;
  int index;
  int i;
  double tmpDoubleArr[12];
  int numberOfTargets;
  int checkQC;
  int error;

  memset((char *)pResp, 0, sizeof(struct parse_basic_resp));
  memset((char *)&dispData, 0, sizeof(struct displayData));
  
  gAbortDisplay = false;
  
  if (!CheckSourceAndMode(respondToWhom))
    return;
	
  if (gTransmittedLengthSum < pLgMaster->gDataChunksLength)
    {
      pResp->hdr.status1 = RESPFAIL;
      pResp->hdr.errtype1 = RESPE1APTERROR;
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      return;
    }
	
  index  = gPlysReceived * kNumberOfRegPoints * 2 * sizeof(uint32_t);
  tmpPtr = (char *)&pLgMaster->gSensorBuffer[index];
  tmpInp = (char *)pInp;
  numberOfTargets = 6;
  gQuickCheckTargetNumber[gPlysReceived] = numberOfTargets;
  index = 0;
  memmove(tmpPtr, &tmpInp[index], (size_t)(kNumberOfRegPoints * 2 * sizeof(uint32_t)));

  //  KLUDGE to prevent quickcheck
  checkQC = 0;
  p_angles = (uint32_t *)&pInp->chunk_anglepairs[0];
  for (i = 0; i < numberOfTargets; i++)
    {
      if (p_angles[2*i] != 0)  // Check first uint32 of pair
	checkQC++;
      if (p_angles[2*(i+1)] != 0)  // Check second uint32 of pair
	checkQC++;
    }
  if (checkQC == 0)
    {
      pLgMaster->gQCcount = -1;
      initQCcounter(pLgMaster);
    }

  if (gPlysReceived++)
    {
      p_transform = (double *)&pInp->chunk_transform[0];
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
      ChangeTransform((double *)&tmpDoubleArr);
    }
  else
    {
      dispData.numberOfSensorSets = gPlysToDisplay;
      dispData.sensorAngles = (uint32_t *)pLgMaster->gSensorBuffer;
      index  = kNumberOfRegPoints * 2 * sizeof(uint32_t);
      p_transform = (double *)&pInp->chunk_transform[0];
      error = 0;
      for (i=0; i < 12 ; i++)
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
	  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp) - kCRCSize), respondToWhom);
	  ResetPlyCounter();
	  return;
	}
      SetUpLaserPattern(pLgMaster, tmpDoubleArr);
    }
  pResp->hdr.errtype = ProcessPatternData(pLgMaster, pLgMaster->gDataChunksBuffer, pLgMaster->gDataChunksLength);
  if (pResp->hdr.errtype)
    {
      pResp->hdr.status = RESPFAIL;
      pResp->hdr.errtype = pResp->hdr.errtype;
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      ResetPlyCounter (  );
      return;
    }
  
  if ( gPlysReceived < gPlysToDisplay )
    {
      pResp->hdr.errtype = (uint16_t)SetPenUp();
      if (!pResp->hdr.errtype)
	{
	  pResp->hdr.errtype = (uint16_t)PendPenDown();
	}
    }
  else
    {
      pResp->hdr.errtype = (uint16_t)FinishPattern(pLgMaster);
    }
  if (pResp->hdr.errtype)
    {
      pResp->hdr.status  = RESPFAIL;
      pResp->hdr.errtype = htons(pResp->hdr.errtype);
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      ResetPlyCounter();
      return;
    }
  else
    {
      if ( gAbortDisplay || ( gPlysReceived < gPlysToDisplay ) )
	{
	  pResp->hdr.status  = RESPGOOD;
	  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
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
	  PostCmdDisplay(pLgMaster, (struct displayData *)&dispData, respondToWhom);
	  ResetPlyCounter();
	}
    }
  return;
}
	
void AddDisplayChunksData ( struct lg_master *pLgMaster, uint32_t dataLength,
				uint32_t dataOffset, char * patternData,
					uint32_t respondToWhom )
{
  struct displayData     dispData;
  struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
  
  /*  wild_debug(); */

  memset((char *)pResp, 0, sizeof(struct parse_basic_resp));
  memset((char *)&dispData, 0, sizeof(struct displayData));
  
  if ( !CheckSourceAndMode ( respondToWhom ) ) return;
  if ( ( dataLength + dataOffset ) > pLgMaster->gDataChunksLength )
    {
      pResp->hdr.status = RESPFAIL;
      pResp->hdr.errtype = htons(RESPDATATOOLARGE);
      if (!(pLgMaster->gHeaderSpecialByte & 0x80))
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      return;
    }
  gTransmittedLengthSum += dataLength;
  memmove ( &pLgMaster->gDataChunksBuffer[dataOffset], patternData,
	    (size_t)dataLength );
  pResp->hdr.status = RESPGOOD;
  if (!(pLgMaster->gHeaderSpecialByte & 0x80))
    HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
}

void DoStop ( struct lg_master *pLgMaster, uint32_t respondToWhom )
{
  PostCommand ( pLgMaster, kStop, 0, respondToWhom );
}


void DoGoAngle (struct lg_master *pLgMaster, struct parse_goangle_parms *pInp, uint32_t respondToWhom )
{
  struct lg_xydata xydata;
  uint32_t theBuffer[2];
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
					     &theBuffer[0], &theBuffer[1]);
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
      xydata.xdata = theBuffer[0] & kMaxUnsigned;
      xydata.ydata = theBuffer[1] & kMaxUnsigned;
      PostCmdGoAngle(pLgMaster, (struct lg_xydata *)&xydata, respondToWhom );
    }
  return;
}

void DoEtherAngle (struct lg_master *pLgMaster, struct parse_ethangle_parms *pInp, uint32_t respondToWhom )
{
  struct lg_xydata xydata;
  uint32_t theBuffer[2];
  double   x, y;
  int      return_val;
  struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
	
  memset((char *)pResp, 0, sizeof(struct parse_basic_resp));
  memset((char *)&theBuffer[0], 0, sizeof(theBuffer));
  memset((char *)&xydata, 0, sizeof(struct lg_xydata));
  if ( !CheckSourceAndMode ( respondToWhom ) ) return;

  if (!pInp)
    return;
  x = pInp->xData;
  y = pInp->yData;
#ifdef PATDEBUG
  fprintf(stderr,"\nDOETHANGL: B4 CNVRT x%f, y%f", x, y);
#endif
  return_val =  ConvertExternalAnglesToBinary(pLgMaster, x, y, &theBuffer[0], &theBuffer[1]);
#ifdef PATDEBUG
  fprintf(stderr,"\nDOETHANGL: AFTER CNVRT x%d, y%d", theBuffer[0], theBuffer[1]);
#endif
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
      xydata.xdata = theBuffer[0] & kMaxUnsigned;
      xydata.ydata = theBuffer[1] & kMaxUnsigned;
      PostCmdEtherAngle(pLgMaster, (struct lg_xydata *)&xydata, respondToWhom);
    }
  return;
}
#if 0
// FIXME---PAH---IS THIS USED ANYMORE???  NOT ON LASERGUIDE LIST!
void DarkAngle (struct lg_master *pLgMaster, double x, double y, uint32_t respondToWhom )
{
        uint32_t theBuffer[2];
	int      return_val;
        struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;

	memset(theBuffer, 0, sizeof(theBuffer));
	memset(pResp, 0, sizeof(struct parse_basic_resp));
	
        if ( !CheckSourceAndMode ( respondToWhom ) ) return;

        return_val = ConvertExternalAnglesToBinary(pLgMaster, x, y, &theBuffer[0], &theBuffer[1]);

        if (return_val)
        {
	  pResp->hdr.status1 = RESPFAIL;
	  pResp->hdr.errtype1 = RESPE1INANGLEOUTOFRANGE;
	  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	  return;
        }
        else
        {
	  PostCommand (pLgMaster, kDarkAngle, (char *)theBuffer, respondToWhom );
        }
}

void DimAngle (struct lg_master *pLgMaster, char * parameters )
{
        double x;
        double y;
        int32_t   pulseoffvalue;
        int32_t   pulseonvalue;
        uint32_t xRaw;
        uint32_t yRaw;
        int index;
        char *ptr;
        

        index = 0;
        ptr = &(parameters[index]);
        x = ((double *)ptr)[0];
        y = ((double *)ptr)[1];
#ifdef SDEBUG
	fprintf( stderr,  "dim a xy %lf %lf\n", x, y );
#endif

        index = 2 * sizeof(double);
        ptr = &(parameters[index]);
        pulseoffvalue = ((int32_t *)ptr)[0];

        index = 2 * sizeof(double) + sizeof(int32_t);
        ptr = &(parameters[index]);
        pulseonvalue = ((int32_t *)ptr)[0];

	// FIXME---PAH---Supposed to check return & send back bad response if bad???
#if 0
        uint32_t  resp;
        resp = ConvertExternalAnglesToBinary(pLgMaster, x, y, &xRaw, &yRaw );
#else
        ConvertExternalAnglesToBinary(pLgMaster, x, y, &xRaw, &yRaw );
#endif
	
        GoToPulse (pLgMaster, &xRaw, &yRaw, pulseoffvalue, pulseonvalue );
}
#endif
void DoFullReg (struct lg_master *pLgMaster, 
		struct parse_dofullreg_parms *pInp, 
		uint32_t respondToWhom )
{
	double theCoordinateBuffer[kNumberOfRegPoints * 3];
	uint32_t theAngleBuffer[kNumberOfRegPoints * 2];
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
#if 0
// FIXME---PAH---IS THIS USED ANYMORE???  NOT ON LASERGUIDE LIST!
void DoDisplay (struct lg_master *pLgMaster,
		uint32_t dataLength
               , char * otherParameters
               , char * patternData
               )
{
	short i;
	double tmpDoubleArr[12];
	struct displayData dispData;
	double *currentData;
	uint32_t *currentAnglePair, anglesBuffer[2 * kNumberOfRegPoints];
	struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
        int index;
	int return_val;
	char * tmpPtr;
        int zeroTarget;
        double x, y, z;

	memset(pResp, 0, sizeof(struct parse_basic_resp));
	memset((char *)&tmpDoubleArr, 0, sizeof(tmpDoubleArr));
	memset((char *)&dispData, 0, sizeof(struct displayData));
	
	gAbortDisplay = false;

	currentData = (double *)otherParameters;
	currentAnglePair = anglesBuffer;

	index  = kNumberOfRegPoints * 3 * sizeof ( double );
	tmpPtr = &otherParameters[index];
	for( i=0; i<12; i++ ) {
		tmpDoubleArr[i] = (double)(((double *)tmpPtr)[i]);
	}
	ChangeTransform((double *)&tmpDoubleArr);

	i = 0;
        zeroTarget = 1;
	while ( i++ < kNumberOfRegPoints )
	{
                x = ( double )DoubleFromCharConv (
                         (unsigned char *)&currentData[0] );
                y = ( double )DoubleFromCharConv (
                         (unsigned char *)&currentData[1] );
                z = ( double )DoubleFromCharConv (
                         (unsigned char *)&currentData[2] );
	        return_val = Transform3DPointToBinary(pLgMaster,x, y, z,
						       &currentAnglePair[0],
						       &currentAnglePair[1] );
                if ( fabs(x) > 0.0001 ) { zeroTarget = 0; }
                if ( fabs(y) > 0.0001 ) { zeroTarget = 0; }
		SetHighBeam ( &currentAnglePair[0], &currentAnglePair[1] );
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
		  if ( !(pLgMaster->gHeaderSpecialByte & 0x80)  ) {
#ifdef ZDEBUG
		    fprintf( stderr
			     , "LaserCmds sending error response %x%x%x\n"
			     , pResp->hdr.status,pResp->hdr.errtype1,pResp->hdr.errtype2);
#endif
		    pResp->hdr.errtype2 = htons(pResp->hdr.errtype2);
		    HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), 0);
		  }
		  return;
		}
		SetHighBeam ( &currentAnglePair[0], &currentAnglePair[1] );
		currentData += 3;
		currentAnglePair += 2;
	}
	
        if ( zeroTarget ) {
            for ( i = 0; i < kNumberOfRegPoints; i++ ) {
	        anglesBuffer[2*i + 0] = 0;
	        anglesBuffer[2*i + 1] = 0;
            }
        }
	
	memmove ( &pLgMaster->gSensorBuffer[ gPlysReceived *
			kNumberOfRegPoints * 2 * sizeof ( uint32_t ) ],
                (char *)anglesBuffer,
		(size_t)( kNumberOfRegPoints * 2 * sizeof ( uint32_t ) ) );
	if ( gPlysReceived++ )
	{
#ifdef ZDEBUG
fprintf( stderr , "1038 plys rcvd %d  disp %d\n", gPlysReceived, gPlysToDisplay );
#endif
	}
	else
	{
#ifdef ZDEBUG
fprintf( stderr , "1046 plys rcvd %d  disp %d\n", gPlysReceived, gPlysToDisplay );
#endif
		dispData.numberOfSensorSets = gPlysToDisplay;
		dispData.sensorAngles = (uint32_t *)pLgMaster->gSensorBuffer;
	        index  = kNumberOfRegPoints * 3 * sizeof ( double );
	        tmpPtr = &otherParameters[index];
		for( i=0; i<12; i++ ) {
		        tmpDoubleArr[i] = (double)(((double *)tmpPtr)[i]);
	        }
		SetUpLaserPattern(pLgMaster, tmpDoubleArr);
	}
	
	pResp->hdr.errtype = ProcessPatternData(pLgMaster, patternData, dataLength);
	if (pResp->hdr.errtype)
	{
		pResp->hdr.status = RESPFAIL; 
                if ( !(pLgMaster->gHeaderSpecialByte & 0x80)  ) {
		  pResp->hdr.errtype = htons(pResp->hdr.errtype);
		  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), 0);
                }
		ResetPlyCounter (  );
		return;
	}
	
#ifdef ZDEBUG
fprintf( stderr , "1080 plys rcvd %d  disp %d\n", gPlysReceived, gPlysToDisplay );
#endif
	if ( gPlysReceived < gPlysToDisplay )
	  pResp->hdr.errtype = (uint16_t)SetPenUp (  );
	else
	  pResp->hdr.errtype = (uint16_t)FinishPattern (pLgMaster);

	if (pResp->hdr.errtype)
	{
		pResp->hdr.status = RESPFAIL; 
                if ( !(pLgMaster->gHeaderSpecialByte & 0x80)  ) {
		  pResp->hdr.errtype = htons(pResp->hdr.errtype);
		  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), 0);
                }
		ResetPlyCounter (  );
	}
	else
	{
		if ( gAbortDisplay || ( gPlysReceived < gPlysToDisplay ) )
		{
#ifdef ZDEBUG
		  fprintf( stderr , "1107 plys rcvd %d  disp %d\n", gPlysReceived, gPlysToDisplay );
#endif
		  pResp->hdr.status = RESPGOOD;
		  if ( !(pLgMaster->gHeaderSpecialByte & 0x80)  ) {
#ifdef ZDEBUG
		    fprintf( stderr
			     , "LaserCmds sending response %x\n"
			     , pResp->hdr.status);
#endif
		    HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), 0);
		  }
		}
		else
		{
                        gVideoCheck = 0;
			SearchBeamOff(pLgMaster);
                        if ( (CDRHflag(pLgMaster) )  ) {
			  pResp->hdr.status1 = RESPFAIL;
			  pResp->hdr.errtype1 = RESPE1BOARDERROR;
			  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), 0);
			  ResetPlyCounter (  );
			  return;
                        }
			PostCmdDisplay(pLgMaster, (struct displayData *)&dispData, 0);
			ResetPlyCounter (  );
		}
	}
}
#endif
void DoDisplayKitVideo (struct lg_master *pLgMaster,
			uint32_t dataLength,
			unsigned char *otherParameters,
			unsigned char *patternData,
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
fprintf( stderr, "gVideoPreDwell %d\n", gVideoPreDwell );
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
fprintf( stderr, "gVideoCount %d\n", gVideoCount );
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
    fprintf( stderr, "XYZ " );
    fprintf( stderr, " %.5lf", ((double *)tmpPtr)[0] );
    fprintf( stderr, " %.5lf", ((double *)tmpPtr)[1] );
    fprintf( stderr, " %.5lf", ((double *)tmpPtr)[2] );
    fprintf( stderr, "\n" );
#endif
    TransformPoint ( &CurrentTransform, inputPoint, outputPoint );
#if defined(SDEBUG) || defined(KITDEBUG)
    double Xpoint, Ypoint, Zpoint;
    Xpoint = outputPoint[0];
    Ypoint = outputPoint[1];
    Zpoint = outputPoint[2];
    fprintf( stderr, "check XYZ " );
    fprintf( stderr, " %.5lf", outputPoint[0] );
    fprintf( stderr, " %.5lf", outputPoint[1] );
    fprintf( stderr, " %.5lf", outputPoint[2] );
    fprintf( stderr, "\n" );
#endif

    PointToBinary(pLgMaster, outputPoint, &pLgMaster->gXcheck, &pLgMaster->gYcheck );

#if defined(SDEBUG) || defined(KITDEBUG)
    fprintf( stderr, "check XY raw " );
    fprintf( stderr, " %x", pLgMaster->gXcheck );
    fprintf( stderr, " %x", pLgMaster->gYcheck );
    fprintf( stderr, "\n" );
#endif

	gAbortDisplay = false;

	if ( !CheckSourceAndMode ( respondToWhom ) ) return;
	
	index  = 0;
	tmpPtr = &otherParameters[index];
	for (i=0; i<12; i++)
	  tmpDoubleArr[i] = (double)(((double *)tmpPtr)[i]);
	ChangeTransform((double *)&tmpDoubleArr);
	
	dispData.numberOfSensorSets = 0;
	dispData.sensorAngles = (uint32_t *)pLgMaster->gSensorBuffer;
	SetUpLaserPattern(pLgMaster, tmpDoubleArr);
	pResp->hdr.errtype = ProcessPatternData(pLgMaster, patternData, dataLength);	
 	if (pResp->hdr.errtype)
	  {
	    pResp->hdr.status = RESPFAIL;
	    pResp->hdr.errtype = htons(pResp->hdr.errtype);
	    HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	    ResetPlyCounter (  );
	    return;
	  }
 	
#if defined(ZDEBUG) || defined(KITDEBUG)
fprintf( stderr , "1555 plys rcvd %d  disp %d\n", gPlysReceived, gPlysToDisplay );
#endif
 	if ( gPlysReceived < gPlysToDisplay )
	  pResp->hdr.errtype = (uint16_t)SetPenUp (  );
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
	      fprintf( stderr , "1574 plys rcvd %d  disp %d\n", gPlysReceived, gPlysToDisplay );
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
			fprintf( stderr , "1634 getweb slen %d gVideoCount %d\n", slen, gVideoCount );
#else
			GetWebVideo(pLgMaster, ToVideo, count, FromVideo );
#endif

                        if ( (CDRHflag(pLgMaster) )  ) {
			  pResp->hdr.status1 = RESPFAIL;
			  pResp->hdr.errtype1 = RESPE1BOARDERROR;
			  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
			  ResetPlyCounter (  );
			  return;
                        }
#if defined(KITDEBUG)
			fprintf( stderr, "LC1651 setVideoCount %d\n", gVideoCount );
#endif
			SetQCcounter(pLgMaster, gVideoCount );
                        gQCtimer = -1;
 			PostCmdDisplay(pLgMaster, (struct displayData *)&dispData, respondToWhom );
 			ResetPlyCounter (  );
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
  if ( !CheckSourceAndMode ( respondToWhom ) ) return;
	
  memset((char *)&pLgMaster->gOutOfRange, 0, sizeof(struct k_header)); 
  ResetPlyCounter();
  gPlysToDisplay = (uint16_t)number;

  // Clear buffer for display data
  memset(pLgMaster->gSensorBuffer, 0xFF, (kNumberOfFlexPoints * 2 * gPlysToDisplay * sizeof(uint32_t)));
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
#ifdef ZDEBUG
        int i;
        fprintf( stderr, "\nDoQuickCheck" );
        for ( i=0; i<12; i++ ) {
            fprintf( stderr, " %8x", ((int *)(angles))[i] );
        }
        fprintf( stderr, "\n" );
#endif
        PerformAndSendQuickCheck (pLgMaster, angles, 6 );  // must now set target number
}

void CloseLaserCommands (void)
{
	ResetPlyCounter (  );
}

double DoubleFromCharConv ( unsigned char *theChar )
{
  double   return_val=0;

  memcpy((char *)&return_val, theChar, sizeof(double));
  return(return_val);
}

uint32_t LongFromCharConv ( unsigned char *theChar )
{
  uint32_t   return_val=0;

  memcpy((char *)&return_val, theChar, sizeof(uint32_t));
  return(ntohl(return_val));
}

unsigned short ShortFromCharConv ( unsigned char *theChar )
{
  uint16_t   return_val=0;

  memcpy((char *)&return_val, theChar, sizeof(uint16_t));
  return(ntohs(return_val));
}

void DoubleConv ( unsigned char *theChar )
{
#if GENERATING68K
	return;
#else
	unsigned char theThing [ sizeof ( double ) ];
	unsigned char *thing = theThing;
	unsigned short i;
	if ( gSameEndian ) return;
	memcpy(thing, theChar, sizeof(double));
#if 0
	*(double *)thing = *(double *)theChar;
#endif	
	i = sizeof ( double );
	thing += i;
	while ( i-- ) *(theChar++) = *(--thing);
	return;
#endif
}

void LongConv ( unsigned char *theChar )
{
#if GENERATING68K
	return;
#else
	unsigned char theThing [ sizeof ( uint32_t ) ];
	unsigned char *thing = theThing;
	unsigned short i;
	if ( gSameEndian ) return;
	memcpy(thing, theChar, sizeof(uint32_t));
#if 0
	*(uint32_t *)thing = *(uint32_t *)theChar;
#endif
	i = sizeof ( uint32_t );
	thing += i;
	while ( i-- ) *(theChar++) = *(--thing);
	return;
#endif
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
	ResetPlyCounter (  );
}
