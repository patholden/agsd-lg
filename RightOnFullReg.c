/*
static char rcsid[] = "$Id: RightOnFullReg.c,v 1.2 2000/05/05 23:57:15 ags-sw Exp $";
*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "AppCommon.h"
#include "AppResponses.h"
#include "LaserInterface.h"
#include "SensorRegistration.h"
#include "SensorSearch.h"
#include "FullRegManager.h"
#include "3DTransform.h"
#include "Protocol.h"

#define	kNumberOfSensorSearchAttempts				3


void RightOnFullReg ( struct lg_master *pLgMaster,
		      char * parameters,
		      uint32_t respondToWhom )
{
	uint32_t ptX, ptY;
	uint32_t fndX;
	uint32_t fndY;
	double XfoundAngles [ kNumberOfRegPoints ];
	double YfoundAngles [ kNumberOfRegPoints ];
	double XExternalAngles [ kNumberOfRegPoints ];
	double YExternalAngles [ kNumberOfRegPoints ];
	double foundAngles [ kNumberOfRegPoints * 2 ];
	uint32_t Xarr [ kNumberOfRegPoints ];
	uint32_t Yarr [ kNumberOfRegPoints ];
        int numberOfFoundTargets;
        int useTarget [ kFeedbackNumber ];


 /*
  * set to maximum size
  */
	unsigned char *RespBuff=pLgMaster->theResponseBuffer;
	uint32_t       resp_len=(sizeof(uint32_t) + (12 * kSizeOldLongDouble) +
				 (2 * kNumberOfRegPoints * sizeof(uint32_t)) +
				 (2 * kNumberOfRegPoints * sizeof(double)) +
				 kCRCSize);
	uint32_t       return_code=0;
        unsigned char *ucPtr;
	unsigned short i, j;
	uint32_t lostSensors;
	unsigned char theResult;
        int32_t RawGeomFlag;
        double theCoordinateBuffer[kNumberOfRegPoints * 3];
        uint32_t theAngleBuffer[kNumberOfRegPoints * 2];
        double *currentData;
	int index;
	double theTransformTolerance;
	transform foundTransform;
        uint32_t *pRawAngles;
        double *pGeometricAngles;
        int32_t offset;
	
	int searchResult;

	memset(RespBuff, 0, resp_len);
        SlowDownAndStop(pLgMaster);
	theTransformTolerance  = pLgMaster->gArgTol;

        for (index = 0; index < kNumberOfRegPoints; index++)
	  {
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
                  + sizeof(double) * kNumberOfRegPoints * 3;
        pGeometricAngles = (double *)(&(parameters)[offset]);
        offset =    sizeof(int32_t) 
                  + sizeof(double) * kNumberOfRegPoints * 3
                  + sizeof(double) * kNumberOfRegPoints * 2;
        pRawAngles = (uint32_t *)(&(parameters)[offset]);
        if ( RawGeomFlag == 1 ) {
           for ( i = 0; i <  kNumberOfRegPoints ; i++ ) {
               theAngleBuffer[2*i  ] = pRawAngles[2*i  ];
               theAngleBuffer[2*i+1] = pRawAngles[2*i+1];
           }
        } else if ( RawGeomFlag == 2 ) {
           for ( i = 0; i <  kNumberOfRegPoints ; i++ ) {
                return_code = ConvertExternalAnglesToBinary (
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
        while ( i < ( kNumberOfRegPoints * 3 ) )
        {
          theCoordinateBuffer[i] = currentData[i];
          i++;
        }
        SaveFullRegCoordinates ( kNumberOfRegPoints, theCoordinateBuffer );
        for ( i = 0; i < kNumberOfRegPoints; i++ ) {
            for ( j = i; j < kNumberOfRegPoints; j++ ) {
                if ( i != j ) {
                    if ( (gX[i]==gX[j]) && (gY[i]==gY[j]) && (gZ[i]==gZ[j]) ) {
                         useTarget[j] = 0;
#ifdef ZDEBUG
                         fprintf( stderr, "target %d rejected %d\n", j, i );
                         fprintf( stderr, "  gX %lf %lf\n", gX[i], gX[j] );
                         fprintf( stderr, "  gY %lf %lf\n", gY[i], gY[j] );
                         fprintf( stderr, "  gZ %lf %lf\n", gZ[i], gZ[j] );
#endif
                    }
                }
            }
        }

	
	lostSensors = 0U;
	
        i = 0;
        if ( return_code == 0 ) {
	  while ( i < kNumberOfRegPoints ) {
		j = gNumberOfSensorSearchAttempts;
		gSearchCurrentSensor = i;

		  /*
		   *  allow for a variable speed search, in needed
		   */
		gCoarse2Factor     = gCoarseFactor;
		gCoarse2SearchStep = gCoarseSearchStep;
#ifdef ZDEBUG
fprintf( stderr, "i %d useTarget %d\n", i, useTarget[i] );
#endif
		while ( j-- ) {
                        ptX = theAngleBuffer[2*i  ];
                        ptY = theAngleBuffer[2*i+1];
                        if ( useTarget[i] == 1 ) {
			  searchResult = SearchForASensor ( pLgMaster, ptX, ptY,
                                                              &fndX, &fndY );
#ifdef ZDEBUG
fprintf( stderr, "finish search for sensor %d\n", i );
#endif
                        } else {
                            searchResult = 99;
                            fndX = 0;
                            fndY = 0;
                        }
                        Xarr[i] = fndX;
                        Yarr[i] = fndY;
			if ( searchResult == kStopWasDone ) {
                              return;
                        }
			if ( !searchResult ) break;
		        gCoarse2SearchStep /= 2;
		        gCoarse2Factor     /= 2; 
			if ( gCoarse2SearchStep <= 0x00010000 ) {
		        	gCoarse2SearchStep = 0x00010000;
		        	gCoarse2Factor     = 1;
			}
		}
		gCoarse2Factor     = gCoarseFactor;
		gCoarse2SearchStep = gCoarseSearchStep;
		
		if ( searchResult ) {
			lostSensors += 1U << i;
		} else {
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
#ifdef ZDEBUG
fprintf( stderr, "fndX %x ", fndX );
fprintf( stderr, "fndY %x ", fndY );
fprintf( stderr, "\n" );
fprintf( stderr, "curX %lf ", XfoundAngles[i] );
fprintf( stderr, "curY %lf ", YfoundAngles[i] );
fprintf( stderr, "\n" );
fprintf( stderr, "curX %lf ", XExternalAngles[i] );
fprintf( stderr, "curY %lf ", YExternalAngles[i] );
fprintf( stderr, "\n" );
fprintf( stderr, "reg  %d  "
                 " %.3lf %.3lf %.3lf "
                 " %.6lf %.6lf  "
                 "\n"
               , i
               , gX[i]
               , gY[i]
               , gZ[i]
               , XExternalAngles[i]
               , YExternalAngles[i]
               );
#endif
                }
		
		i++;
	  }
	
          for ( i = 0; i < kNumberOfRegPoints ; i++ ) {
             foundAngles[2*i  ] = XfoundAngles[i];
             foundAngles[2*i+1] = YfoundAngles[i];
             if ( foundTarget[i] == 1 )  numberOfFoundTargets ++;
          }
          if ( numberOfFoundTargets >= 4 ) {
	      theResult = FindTransformMatrix ( kNumberOfRegPoints,
                  gDeltaMirror, theTransformTolerance, foundAngles,
                  (double *)&foundTransform );
          }
        } else {
          theResult = 0;
        }

	if ( theResult ) {
	    return_code = kOK | ((0xFF & (uint32_t)GnOfTrans) << 8);
	    memcpy(RespBuff, &return_code, sizeof(uint32_t));
            if( gTargetDrift ) {
                  InitDrift( Xarr, Yarr );
            }

	    index = sizeof(uint32_t);
	    ucPtr = (unsigned char *)(RespBuff + index);

	    memset( ucPtr, 0, 12 * (kSizeOldLongDouble) );
	    TransformIntoArray( &foundTransform, (double *)ucPtr );

	    index = sizeof(uint32_t) + 12 * ( kSizeOldLongDouble );
	    ucPtr = (unsigned char *)(RespBuff + index);
	    for( i=0; i < kNumberOfRegPoints; i++ ) {
                        ((uint32_t *)ucPtr)[2*i  ] = Xarr[i];
                        ((uint32_t *)ucPtr)[2*i+1] = Yarr[i];
            }
	    index = sizeof(uint32_t)
                     + 12 * ( kSizeOldLongDouble )
		     +  2 * kNumberOfRegPoints * ( sizeof ( uint32_t ) ) ;
	    ucPtr = (unsigned char *)(RespBuff + index);
	    for( i=0; i < kNumberOfRegPoints; i++ ) {
                        ((double *)ucPtr)[2*i  ] = XExternalAngles[i];
                        ((double *)ucPtr)[2*i+1] = YExternalAngles[i];
            }
	    HandleResponse (pLgMaster, resp_len, respondToWhom );
	} else {
		return_code = kFail | lostSensors;
		memcpy(RespBuff, &return_code, sizeof(uint32_t));
		HandleResponse (pLgMaster, sizeof(uint32_t), respondToWhom );
		return;
	}
}
