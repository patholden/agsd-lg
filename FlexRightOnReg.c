#include <stdint.h>
static char rcsid[] = "$Id: FlexRightOnReg $";

#include <stdio.h>

#include "AppCommon.h"
#include "comm_loop.h"
#include "LaserInterface.h"
#include "SensorRegistration.h"
#include "SensorSearch.h"
#include "FullRegManager.h"
#include "3DTransform.h"
#include "Protocol.h"

#define	kNumberOfSensorSearchAttempts				2


// FIXME---PAH---NEEDS CMD/RESP FIXES
void FlexRightOnReg(char * parameters, uint32_t respondToWhom)
{
	uint32_t ptX, ptY;
	uint32_t fndX;
	uint32_t fndY;
	double XfoundAngles [ kNumberOfFlexPoints ];
	double YfoundAngles [ kNumberOfFlexPoints ];
	double XExternalAngles [ kNumberOfFlexPoints ];
	double YExternalAngles [ kNumberOfFlexPoints ];
	double foundAngles [ kNumberOfFlexPoints * 2 ];
	uint32_t Xarr [ kNumberOfFlexPoints ];
	uint32_t Yarr [ kNumberOfFlexPoints ];
        int numberOfFoundTargets;
        int useTarget [ kFeedbackNumber ];


 /*
  * set to maximum size
  */
	unsigned char theResponseBuffer [ sizeof ( uint32_t ) +
		12 * ( kSizeOldLongDouble  ) +
		2 * kNumberOfFlexPoints * sizeof ( uint32_t ) +
		2 * kNumberOfFlexPoints * sizeof ( double ) +
                kCRCSize ];
        unsigned char *ucPtr;
	unsigned short i, j;
	uint32_t lostSensors;
	unsigned char theResult;
        int32_t RawGeomFlag;
        double theCoordinateBuffer[kNumberOfFlexPoints * 3];
        uint32_t theAngleBuffer[kNumberOfFlexPoints * 2];
        uint32_t *currentAnglePair;
        double *currentData;
	int index;
	double theTransformTolerance;
	transform foundTransform;
        uint32_t *pRawAngles;
        double *pGeometricAngles;
        int32_t offset;
	
	int searchResult;
	
        SlowDownAndStop( );
	theTransformTolerance  = pLgMaster->gArgTol;

        for ( index = 0; index < kNumberOfFlexPoints; index++ ) {
               foundTarget[index] = 0;
               useTarget[index] = 1;
        }
        gWorstTolReg = 1.0;
        numberOfFoundTargets = 0;

	i = 0U;
	
        RawGeomFlag = *(int32_t *)(&(parameters)[0]);

          /*
           * for RawGeomFlag:
           *   1 -> raw binary angles
           *   2 -> Geometric angles
           */
        offset =    sizeof(int32_t) 
                  + sizeof(double) * kNumberOfFlexPoints * 3;
        pGeometricAngles = (double *)(&(parameters)[offset]);
        offset =    sizeof(int32_t) 
                  + sizeof(double) * kNumberOfFlexPoints * 3
                  + sizeof(double) * kNumberOfFlexPoints * 2;
        pRawAngles = (uint32_t *)(&(parameters)[offset]);
        if ( RawGeomFlag == 1 ) {
           for ( i = 0; i <  kNumberOfFlexPoints ; i++ ) {
               theAngleBuffer[2*i  ] = pRawAngles[2*i  ];
               theAngleBuffer[2*i+1] = pRawAngles[2*i+1];
           }
        } else if ( RawGeomFlag == 2 ) {
           for ( i = 0; i <  kNumberOfFlexPoints ; i++ ) {
                *(uint32_t *)theResponseBuffer =
                    ConvertExternalAnglesToBinary (
                          pGeometricAngles[2*i],
                          pGeometricAngles[2*i+1],
                          &(theAngleBuffer[2*i]),
                          &(theAngleBuffer[2*i+1]) );
           }
        } else {
        }
        i = 0;
        index = sizeof(int32_t);
        currentData = (double *)(&parameters[index]);
        while ( i < ( kNumberOfFlexPoints * 3 ) )
        {
          theCoordinateBuffer[i] = currentData[i];
          i++;
        }
        SaveFullRegCoordinates ( kNumberOfFlexPoints, theCoordinateBuffer );
        for ( i = 0; i < kNumberOfFlexPoints; i++)
	  {
            for (j = i; j < kNumberOfFlexPoints; j++)
	      {
                if (i != j)
		  {
                    if ((gX[i]==gX[j]) && (gY[i]==gY[j]) && (gZ[i]==gZ[j]))
                         useTarget[j] = 0;
		  }
	      }
	  }
	lostSensors = 0;
        i = 0;
        if ( *(uint32_t *)theResponseBuffer == 0 ) {
	  while ( i < kNumberOfFlexPoints)
	    {
	      j = gNumberOfSensorSearchAttempts;
	      gSearchCurrentSensor = i;

	      /*
	       *  allow for a variable speed search, in needed
	       */
	      gCoarse2Factor     = gCoarseFactor;
	      pLgMaster->gCoarse2SearchStep = gCoarseSearchStep;
	      while (j--)
		{
		  ptX = theAngleBuffer[2*i  ];
		  ptY = theAngleBuffer[2*i+1];
		  if (useTarget[i] == 1)
		    searchResult = SearchForASensor(ptX, ptY, &fndX, &fndY);
		  else
		    {
		      searchResult = 99;
		      fndX = 0;
		      fndY = 0;
		    }
		  Xarr[i] = fndX;
		  Yarr[i] = fndY;
		  if (searchResult == kStopWasDone)
		    {
		      SearchBeamOff(pLgMaster);
		      return;
		    }
		  if (!searchResult)
		    break;
		  pLgMaster->gCoarse2SearchStep /= 2;
		  gCoarse2Factor     /= 2; 
		  if (pLgMaster->gCoarse2SearchStep <= 0x00010000)
		    {
		      pLgMaster->gCoarse2SearchStep = 0x00010000;
		      gCoarse2Factor     = 1;
		    }
		}
	      gCoarse2Factor     = gCoarseFactor;
	      pLgMaster->gCoarse2SearchStep = gCoarseSearchStep;
	      if (searchResult)
		lostSensors += 1U << i;
	      else
		{
		  foundTarget[i] = 1;
		  ConvertBinaryToGeometricAngles(
						 fndX,
						 fndY,
						 &(XfoundAngles[i]),
						 &(YfoundAngles[i]) );
		  ConvertBinaryToExternalAngles(
						fndX,
						fndY,
						&(XExternalAngles[i]),
						&(YExternalAngles[i]) );
                }
	      i++;
	    }
	
          for ( i = 0; i < kNumberOfFlexPoints ; i++)
	    {
	      foundAngles[2*i  ] = XfoundAngles[i];
	      foundAngles[2*i+1] = YfoundAngles[i];
	      if (foundTarget[i] == 1)
		numberOfFoundTargets ++;
	    }
          if ( numberOfFoundTargets >= 4)
	    {
	      theResult = FindTransformMatrix(pLgMaster, kNumberOfFlexPoints,
					      gDeltaMirror, theTransformTolerance, foundAngles,
					      (double *)&foundTransform );
	    }
        }
	else
          theResult = 0;

	if (theResult)
	  {
	    *(uint32_t *)theResponseBuffer = kOK | 
	      ((0xFF & (uint32_t)GnOfTrans) << 8);
            if( gTargetDrift)
	      InitDrift( Xarr, Yarr );
	    
	    index = sizeof(uint32_t);
	    ucPtr = (unsigned char *)&( theResponseBuffer[index]);

	    memset( ucPtr, 0, 12 * (kSizeOldLongDouble) );
	    TransformIntoArray( &foundTransform, (double *)ucPtr );

	    index = sizeof(uint32_t) + 12 * ( kSizeOldLongDouble );
	    ucPtr = (unsigned char *)&( theResponseBuffer[index]);
	    for ( i=0; i < kNumberOfFlexPoints; i++ )
	      {
		((uint32_t *)ucPtr)[2*i  ] = Xarr[i];
		((uint32_t *)ucPtr)[2*i+1] = Yarr[i];
	      }
	    index = sizeof(uint32_t)
	      + 12 * ( kSizeOldLongDouble )
	      +  2 * kNumberOfFlexPoints * ( sizeof ( uint32_t ) ) ;
	    ucPtr = (unsigned char *)&( theResponseBuffer[index]);
	    for( i=0; i < kNumberOfFlexPoints; i++ )
	      {
		((double *)ucPtr)[2*i  ] = XExternalAngles[i];
		((double *)ucPtr)[2*i+1] = YExternalAngles[i];
	      }
	    HandleResponse ( (char *)theResponseBuffer,
			     ( sizeof ( uint32_t )
			       + 12 * ( kSizeOldLongDouble  )
			       +  2 * kNumberOfFlexPoints * sizeof ( uint32_t ) 
			       +  2 * kNumberOfFlexPoints * sizeof ( double ) ),
			     respondToWhom );
	  }
	else
	  {
	    *(uint32_t *)theResponseBuffer = kFail | lostSensors;
	    HandleResponse ( (char *)theResponseBuffer,
			     sizeof ( uint32_t ),
			     respondToWhom );
	    return;
	  }
	return;
}
