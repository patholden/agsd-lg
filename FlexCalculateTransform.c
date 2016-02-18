/*
static char rcsid[] = "$Id$";

*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
#include "FlexCalculateTransform.h"
#include "3DTransform.h"
#include "Protocol.h"

//FIXME---PAH---NEEDS CMD/RESP STRUCTS
void FlexCalculateTransform ( struct lg_master *pLgMaster,
			      char * data,
			      uint32_t respondToWhom )
{
	double foundAngles [ kNumberOfFlexPoints * 2 ];
	uint32_t Xarr [ kNumberOfFlexPoints ];
	uint32_t Yarr [ kNumberOfFlexPoints ];
	double Xgeo;
	double Ygeo;
        int32_t Xbin;
        int32_t Ybin;
        double theCoordinateBuffer[kNumberOfFlexPoints * 3];
	uint32_t resp_len=( sizeof ( uint32_t )
			    + (12 * kSizeOldLongDouble)
			    + (2 * kNumberOfFlexPoints * sizeof(uint32_t))
			    + kCRCSize);
        unsigned char *ucPtr;
	unsigned short i, j;
	unsigned char theResult;
	int index;
        double *pExternalAngles;
        int32_t offset;
	double theTransformTolerance;
	transform foundTransform;
        double *currentData;
        double xMirror;
        double yMirror;
        double angX;
        double angY;
        int32_t nTargets;
        int32_t int32_tColinear;
        int32_t int32_tPlanar;


        memset(pLgMaster->theResponseBuffer, 0, resp_len);
	int32_tColinear = 0;
        int32_tPlanar = 0;
	
	theTransformTolerance  = pLgMaster->gArgTol;
		
        nTargets = *(int32_t *)(data);
	if ( (nTargets < 4) || (nTargets > kNumberOfFlexPoints) ) {
		*(uint32_t *)pLgMaster->theResponseBuffer = kFail;
		HandleResponse ( pLgMaster,
                        sizeof ( uint32_t ),
			respondToWhom );
		return;
        }
	
        offset =  sizeof(int32_t) + sizeof(double) * kNumberOfFlexPoints * 3;
        pExternalAngles = (double *)(&(data)[offset]);
        
        i = 0;
        while ( i < nTargets ) {
            gColinear[i] = 0;
            foundTarget[i] = 1;
            angX = pExternalAngles[2*i  ];
            angY = pExternalAngles[2*i+1];
	    ConvertExternalAnglesToMirror(angX, angY, &xMirror, &yMirror);
            Xgeo = xMirror;
            Ygeo = yMirror;
            ConvertMirrorToGeometricAngles ( &Xgeo, &Ygeo );
            foundAngles[2*i  ] = Xgeo;
            foundAngles[2*i+1] = Ygeo;
            ConvertExternalAnglesToBinary(pLgMaster, angX, angY, &Xbin, &Ybin);
            Xarr[i] = Xbin;
            Yarr[i] = Ybin;
#ifdef SDEBUG    
fprintf( stderr, "CalcTrans %2d ext ang %08x %08x\n", i, Xbin, Ybin );
#endif
            i++;
        } 

        index = sizeof(int32_t);
        currentData = (double *)(&data[index]);
        i = 0;
        while ( i < ( kNumberOfFlexPoints * 3 ) )
        {
          theCoordinateBuffer[i] = 0;
          i++;
        }
        i = 0;
        while ( i < ( nTargets * 3 ) )
        {
          theCoordinateBuffer[i] = currentData[i];
          i++;
        }
        SaveFullRegCoordinates ( nTargets, theCoordinateBuffer );

           //
           // if gHeaderSpecialByte is NOT 0x40,
           // then reject duplicate targets
           //
        if ( !(pLgMaster->gHeaderSpecialByte & 0x40)  ) {
          for ( i = 0; i < nTargets; i++ ) {
            for ( j = i; j < nTargets; j++ ) {
                if ( i != j ) {
                    if ( (gX[i]==gX[j]) && (gY[i]==gY[j]) && (gZ[i]==gZ[j]) ) {
                         foundTarget[j] = 0;
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
        }


        theResult = FindTransformMatrix ( nTargets
                                        , gDeltaMirror
                                        , theTransformTolerance
                                        , foundAngles
                                        , (double *)&foundTransform
                                         );

          /* desperate attempt to get something */
        if ( gSaved == 0 && gForceTransform ) {
            theResult = FindTransformMatrix ( nTargets
                                            , gDeltaMirror
                                            , 0.00001
                                            , foundAngles
                                            , (double *)&foundTransform
                                             );
            GnOfTrans = 0;
        }

        for ( i = 0; i < nTargets ; i++ ) {
            if ( gColinear[i] > 0 ) {
                 int32_tColinear = int32_tColinear | (1 << i);
            }
        }


	if ( theResult ) {
	        *(uint32_t *)pLgMaster->theResponseBuffer = kOK;
	} else {
		*(uint32_t *)pLgMaster->theResponseBuffer = kFlexFail;
        }

	index = sizeof(uint32_t);
	ucPtr = (unsigned char *)&(pLgMaster->theResponseBuffer[index]);

	memset( ucPtr, 0, 12 * (kSizeOldLongDouble) );
	TransformIntoArray( &foundTransform, (double *)ucPtr );

	index = sizeof(uint32_t) + 12 * ( kSizeOldLongDouble );
	ucPtr = (unsigned char *)&( pLgMaster->theResponseBuffer[index]);
	for( i=0; i < kNumberOfFlexPoints; i++ ) {
                        ((uint32_t *)ucPtr)[2*i  ] = 0xffffffff;
                        ((uint32_t *)ucPtr)[2*i+1] = 0xffffffff;
        }
	for( i=0; i < nTargets; i++ ) {
                        ((uint32_t *)ucPtr)[2*i  ] = Xarr[i];
                        ((uint32_t *)ucPtr)[2*i+1] = Yarr[i];
#ifdef ZDEBUG
fprintf( stderr, "FCT dac xy %08x %08x  i %d\n", Xarr[i], Yarr[i], i );
#endif
        }
	index = sizeof(uint32_t)
                  + 12 * ( kSizeOldLongDouble )
		  + 2 * kNumberOfFlexPoints * sizeof ( uint32_t )
                  ;
	ucPtr = (unsigned char *)&( pLgMaster->theResponseBuffer[index]);
        ((int32_t *)ucPtr)[0] = gSaved;
        ((int32_t *)ucPtr)[1] = nTargets;
        ((int32_t *)ucPtr)[2] = int32_tColinear;
        ((int32_t *)ucPtr)[3] = int32_tPlanar;
	HandleResponse ( pLgMaster,
			( sizeof ( uint32_t ) +
			12 * ( kSizeOldLongDouble  )
			+ 2 * kNumberOfFlexPoints * sizeof ( uint32_t ) )
                        + 4 * sizeof( int32_t )
                        ,
			respondToWhom );
}
