/*
static char rcsid[] = "$Id: FullRegWithFeedback.c,v 1.1 2006/06/05 18:40:30 ags-sw Exp ags-sw $";

*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "AppCommon.h"
#include "comm_loop.h"
#include "LaserInterface.h"
#include "SensorRegistration.h"
#include "SensorSearch.h"
#include "FullRegManager.h"
#include "3DTransform.h"
#include "L3DTransform.h"
#include "BoardComm.h"

#define OLD_RETURN

#define	kNumberOfSensorSearchAttempts				3

#define OutputPadding   2
// FIXME---PAH---NEEDS CMD/RESP STRUCT FIXES
void FullRegWithFeedback (struct lg_master *pLgMaster,
			  char * parameters,
			  uint32_t respondToWhom )
{
    int16_t fndX, fndY;
    double XfoundAngles [ kFeedbackNumber ];
    double YfoundAngles [ kFeedbackNumber ];
    double XExternalAngles [ kFeedbackNumber ];
    double YExternalAngles [ kFeedbackNumber ];
    double foundAngles [ kFeedbackNumber * 2 ];
    int16_t Xarr[kFeedbackNumber];
    int16_t Yarr[kFeedbackNumber];
    int32_t target_status [ kFeedbackNumber ];
    int numberOfFoundTargets;
    int useTarget [ kFeedbackNumber ];
    char *RespBuff=(char *)pLgMaster->theResponseBuffer;
    uint32_t    resp_len=(sizeof ( uint32_t )
			  + 12 * ( sizeof ( double )  )
			  +        sizeof ( double ) 
			  +        sizeof ( double ) 
			  +        sizeof ( double ) 
			  +        sizeof ( int32_t ) 
			  +        sizeof ( int32_t ) 
			  +  2 * kFeedbackNumber * sizeof ( uint32_t )
			  +  2 * kFeedbackNumber * sizeof ( double )
			  +      kFeedbackNumber * sizeof ( int32_t )
			  +  OutputPadding
			  +  1024
			  +  kCRCSize);
    uint32_t    return_code=0;
    unsigned char *ucPtr;
    unsigned short i, j;
    uint32_t lostSensors;
    unsigned char theResult;
    int32_t numberOfTargets;
    int32_t RawGeomFlag;
    double theCoordinateBuffer[kFeedbackNumber * 3];
    int32_t theAngleBuffer[kFeedbackNumber * 2];
    double *currentData;
    struct lg_xydata *pCurXY;
    int index;
    double theTransformTolerance;
    transform foundTransform;
    uint32_t *pRawAngles;
    double *pGeometricAngles;
    int32_t offset;
    int searchResult;
    
    SlowDownAndStop(pLgMaster);
    theTransformTolerance  = pLgMaster->gArgTol;
    pLgMaster->gBestTargetNumber = 0;
    memset(pLgMaster->gBestTargetArray, 0, sizeof(pLgMaster->gBestTargetArray));
        for ( i = 0; i < kFeedbackNumber; i++ ) {
               target_status[i] = 0;
               pLgMaster->foundTarget[i]  = 0;
               savePoint[i] = 0;
               XExternalAngles[i] = 0;
               YExternalAngles[i] = 0;
               Xarr[i] = 0;
               Yarr[i] = 0;
               useTarget[i] = 1;
        }
        gWorstTolReg = 1.0;


	i = 0U;
	
        RawGeomFlag = *(int32_t *)(&(parameters)[0]);

          /*
           * for RawGeomFlag:
           *   1 -> raw binary angles
           *   2 -> Geometric angles
           */
        offset =    sizeof(int32_t); 
        numberOfTargets = *(int32_t *)(&(parameters)[offset]);

        offset =    sizeof(int32_t) 
                  + sizeof(int32_t)
                  + sizeof(double) * kFeedbackNumber * 3;
        pGeometricAngles = (double *)(&(parameters)[offset]);
        offset =    sizeof(int32_t) 
                  + sizeof(int32_t)
                  + sizeof(double) * kFeedbackNumber * 3
                  + sizeof(double) * kFeedbackNumber * 2;
        pRawAngles = (uint32_t *)(&(parameters)[offset]);

        if ( RawGeomFlag == 1)
	  {
	    for ( i = 0; i <  kFeedbackNumber ; i++)
	      {
		theAngleBuffer[2*i  ] = pRawAngles[2*i  ];
		theAngleBuffer[2*i+1] = pRawAngles[2*i+1];
	      }
	  }
	else if (RawGeomFlag == 2)
	  {
	    for ( i = 0; i <  kFeedbackNumber ; i++)
	      {
		pCurXY = (struct lg_xydata *)((char *)&theAngleBuffer[0] + (sizeof(struct lg_xydata) * i));
		return_code = ConvertExternalAnglesToBinary (pLgMaster,
							     pGeometricAngles[2*i],
							     pGeometricAngles[2*i+1],
							     &pCurXY->xdata,
							     &pCurXY->ydata);
		memcpy(RespBuff, &return_code, sizeof(uint32_t));
	      }
        }
        i = 0;
        index = sizeof(int32_t) + sizeof(int32_t);
        currentData = (double *)(&parameters[index]);
        while (i < (kFeedbackNumber * 3))
	  {
	    theCoordinateBuffer[i] = currentData[i];
	    i++;
	  }
        SaveFullRegCoordinates ( numberOfTargets, theCoordinateBuffer );

        for ( i = 0; i < numberOfTargets; i++ )
	  {
            for ( j = i; j < numberOfTargets; j++)
	      {
                if ( i != j )
		  {
                    if ( (gX[i]==gX[j]) && (gY[i]==gY[j]) && (gZ[i]==gZ[j]))
		      useTarget[j] = 0;
		  }
	      }
	  }
	lostSensors = 0;
	i = 0;
        if (return_code == 0)
	  {
	    while (i < numberOfTargets)
	      {
		j = gNumberOfSensorSearchAttempts;
		gSearchCurrentSensor = i;

		/*
		 *  allow for a variable speed search, in needed
		 */
		pLgMaster->gCoarse2Factor     = kCoarseFactorDef;
		pLgMaster->gCoarse2SearchStep = kCoarseSrchStpDef;
		while (j--)
		  {
		    pCurXY = (struct lg_xydata *)((char *)&theAngleBuffer[0] + (sizeof(struct lg_xydata) * i));
		    if (useTarget[i] == 1)
		      {
			searchResult = SearchForASensor ( pLgMaster, pCurXY->xdata, pCurXY->ydata,
							  &fndX, &fndY );
			if ( searchResult == kStopWasDone )
			  {
			    SearchBeamOff(pLgMaster);
			    fndX = 0;
			    fndY = 0;
			  }
		      }
		    else
		      {
			searchResult = 99;
			fndX = 0;
			fndY = 0;
		      }
		    Xarr[i] = fndX;
		    Yarr[i] = fndY;
		    if (searchResult == kStopWasDone)
		      return;
		    if (!searchResult)
		      break;
		    pLgMaster->gCoarse2SearchStep /= 2;
		    pLgMaster->gCoarse2Factor     /= 2; 
		    if (pLgMaster->gCoarse2SearchStep < 1)
		      {
			pLgMaster->gCoarse2SearchStep = 1;
			pLgMaster->gCoarse2Factor     = 1;
		      }
		  }
		pLgMaster->gCoarse2Factor     = kCoarseFactorDef;
		pLgMaster->gCoarse2SearchStep = kCoarseSrchStpDef;
		if (searchResult)
		  {
		    lostSensors += 1U << i;
		    XfoundAngles[i] = 0;
		    YfoundAngles[i] = 0;
		  }
		else
		  {
		    target_status[i] = 1;
		    pLgMaster->foundTarget[i]   = 1;
		    ConvertBinaryToGeometricAngles(pLgMaster, fndX, fndY,
						   &(XfoundAngles[i]),
						   &(YfoundAngles[i]) );
		    ConvertBinaryToExternalAngles(pLgMaster, fndX, fndY,
						  &(XExternalAngles[i]),
						  &(YExternalAngles[i]) );
		  }
		i++;
	      }
	    numberOfFoundTargets = 0;
	    for (i = 0; i < numberOfTargets ; i++)
	      {
		foundAngles[2*i  ] = XfoundAngles[i];
		foundAngles[2*i+1] = YfoundAngles[i];
		if (pLgMaster->foundTarget[i] == 1)
		  numberOfFoundTargets ++;
	      }
          if (numberOfFoundTargets >= 4)
	    {
	      theResult = FindTransformMatrix(pLgMaster, numberOfTargets, gDeltaMirror,
					      theTransformTolerance, foundAngles,
					      (double *)&foundTransform);
	    }
          for (i = 0; i < numberOfTargets ; i++)
	    {
              if (savePoint[i] > 0)
		target_status[i] = 2;
	    }
	  }
	else
          theResult = false;

        if (gTargetDrift)
	  InitDrift(Xarr, Yarr);
        /* 
         *  last desperate attempt to find transform
         *  provided that at least four targets have been found
         */
        if ((gSaved == 0) && (numberOfFoundTargets >= 4) && gForceTransform)
	  {
	    theResult = FindTransformMatrix(pLgMaster, numberOfTargets, gDeltaMirror, 0.001,
					    foundAngles, (double *)&foundTransform);
	  }

	/*
	 *   finally prepare and send response
	 */
	ucPtr = (unsigned char *)(RespBuff);
        memset( ucPtr, 0, resp_len);

        if (theResult == true)
	  {
	    return_code = kOK | ((0xFF & (uint32_t)GnOfTrans) << 8);
	    memcpy(RespBuff, &return_code, sizeof(uint32_t));
	  }
	else
	  {
            return_code = kFail;
	    memcpy(RespBuff, &return_code, sizeof(uint32_t));
            HandleResponse ( pLgMaster, sizeof(uint32_t), respondToWhom );
            return;
	  }

	index = sizeof(uint32_t);
	ucPtr = (unsigned char *)(RespBuff + index);

        TransformIntoArray( &foundTransform, (double *)ucPtr );

	index = sizeof(uint32_t)
                     + 12 * ( sizeof ( double ) )
                     ;
	ucPtr = (unsigned char *)(RespBuff + index);
        *(double *)ucPtr = gBestTolAll;

	index = sizeof(uint32_t)
                     + 12 * ( sizeof ( double ) )
                     +        sizeof ( double ) 
                     ;
	ucPtr = (unsigned char *)(RespBuff + index);
        *(double *)ucPtr = gWorstTolAll;

	index = sizeof(uint32_t)
                     + 12 * ( sizeof ( double ) )
                     +        sizeof ( double ) 
                     +        sizeof ( double ) 
                     ;
	ucPtr = (unsigned char *)(RespBuff + index);
        if ( gWorstTolReg > 0 ) {
            *(double *)ucPtr = gWorstTolReg;
        } else {
            *(double *)ucPtr = 1.0;
        }

	index = sizeof(uint32_t)
                     + 12 * ( sizeof ( double ) )
                     +        sizeof ( double ) 
                     +        sizeof ( double ) 
                     +        sizeof ( double ) 
                     ;
	ucPtr = (unsigned char *)(RespBuff + index);
        *(int32_t *)ucPtr = gSaved;

	index = sizeof(uint32_t)
                     + 12 * ( sizeof ( double ) )
                     +        sizeof ( double ) 
                     +        sizeof ( double ) 
                     +        sizeof ( double ) 
                     +        sizeof ( int32_t ) 
                     ;
	ucPtr = (unsigned char *)(RespBuff + index);
        *(int32_t *)ucPtr = numberOfTargets;

	index = sizeof(uint32_t)
                     + 12 * ( sizeof ( double ) )
                     +        sizeof ( double ) 
                     +        sizeof ( double ) 
                     +        sizeof ( double ) 
                     +        sizeof ( int32_t ) 
                     +        sizeof ( int32_t ) 
                     ;
	ucPtr = (unsigned char *)(RespBuff + index);

	for( i=0; i < kFeedbackNumber; i++ ) {
                        ((uint32_t *)ucPtr)[2*i  ] = Xarr[i];
                        ((uint32_t *)ucPtr)[2*i+1] = Yarr[i];
        }
	index = sizeof(uint32_t)
                     + 12 * ( sizeof ( double ) )
                     +        sizeof ( double ) 
                     +        sizeof ( double ) 
                     +        sizeof ( double ) 
                     +        sizeof ( int32_t ) 
                     +        sizeof ( int32_t ) 
		     +  2 * kFeedbackNumber * ( sizeof ( uint32_t ) )
                     ;
	ucPtr = (unsigned char *)(RespBuff + index);
	for( i=0; i < kFeedbackNumber; i++ ) {
                        ((double *)ucPtr)[2*i  ] = XExternalAngles[i];
                        ((double *)ucPtr)[2*i+1] = YExternalAngles[i];
        }
	index = sizeof(uint32_t)
                     + 12 * ( sizeof ( double ) )
                     +        sizeof ( double ) 
                     +        sizeof ( double ) 
                     +        sizeof ( double ) 
                     +        sizeof ( int32_t ) 
                     +        sizeof ( int32_t ) 
		     +  2 * kFeedbackNumber * ( sizeof ( uint32_t ) )
		     +  2 * kFeedbackNumber * ( sizeof ( double ) )
                     ;
	ucPtr = (unsigned char *)(RespBuff + index);
	for( i=0; i < kFeedbackNumber; i++ ) {
                        ((int32_t *)ucPtr)[i] = target_status[i];
        }

	HandleResponse (pLgMaster, resp_len, respondToWhom );
	return;
}
