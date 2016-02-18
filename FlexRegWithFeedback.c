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
#include "FullRegManager.h"
#include "3DTransform.h"
#include "L3DTransform.h"
#include "FOM.h"

#define OLD_RETURN
#define OutputPadding   (2)

//FIXME---PAH---NEEDS CMD/RESP STRUCT
void FlexFullRegWithFeedback ( struct lg_master *pLgMaster,
			       char * parameters,
			       uint32_t respondToWhom )
{
	int32_t ptX, ptY;
	int32_t fndX;
	int32_t fndY;
	double XfoundAngles [ kNumberOfFlexPoints ];
	double YfoundAngles [ kNumberOfFlexPoints ];
	double XExternalAngles [ kNumberOfFlexPoints ];
	double YExternalAngles [ kNumberOfFlexPoints ];
	double foundAngles [ kNumberOfFlexPoints * 2 ];
	int32_t Xarr [ kNumberOfFlexPoints ];
	int32_t Yarr [ kNumberOfFlexPoints ];
        int32_t target_status [ kNumberOfFlexPoints ];
        int numberOfFoundTargets;
        int useTarget [ kNumberOfFlexPoints ];
	char *RespBuff=(char *)pLgMaster->theResponseBuffer;
	uint32_t   resp_len = (sizeof ( uint32_t )
			       + (12 * sizeof(double))
			       +        sizeof ( double )
			       +        sizeof ( double ) 
			       +        sizeof ( double ) 
			       +        sizeof ( int32_t ) 
			       +        sizeof ( int32_t ) 
			       +  (2 * kNumberOfFlexPoints * sizeof(uint32_t))
			       +  (2 * kNumberOfFlexPoints * sizeof(double))
			       +  (kNumberOfFlexPoints * sizeof(int32_t))
			       +  OutputPadding
			       +  sizeof ( int32_t ) 
			       +  sizeof ( int32_t ) 
			       +  sizeof ( int32_t ) 
			       +  sizeof ( int32_t ) 
			       +  sizeof ( int32_t ) 
			       +  sizeof ( int32_t ) 
			       +  1024
			       +  3 *  sizeof ( double ) 
			       +  kCRCSize);
	uint32_t      return_code=0;
        unsigned char *ucPtr;
	unsigned short i, j;
	uint32_t lostSensors;
	unsigned char theResult;
        int32_t numberOfTargets;
        int32_t RawGeomFlag;
        double theCoordinateBuffer[kNumberOfFlexPoints * 3];
        int32_t theAngleBuffer[kNumberOfFlexPoints * 2];
        double *currentData;
	int index;
	double theTransformTolerance;
	transform foundTransform;
        uint32_t *pRawAngles;
        double *pGeometricAngles;
        int32_t offset;
        int32_t int32_tColinear;
        int32_t int32_tPlanar;
        double useTol;
	
	int searchResult;

        unsigned char saveHeaderSpecialByte;

        saveHeaderSpecialByte = pLgMaster->gHeaderSpecialByte;

	memset(RespBuff, 0, resp_len);
        SlowDownAndStop(pLgMaster);
	
	theTransformTolerance  = pLgMaster->gArgTol;

        int32_tColinear = 0;
        int32_tPlanar = 0;


        gBestTargetNumber = 0;
        for ( i = 0; i < 128; i++ ) {
               gBestTargetArray[i] = 0;
        }

	// Initialize buffers
        for ( i = 0; i < kNumberOfFlexPoints; i++ ) {
               target_status[i] = 0;
               foundTarget[i]  = 0;
               savePoint[i] = 0;
               XExternalAngles[i] = 0;
               YExternalAngles[i] = 0;
               Xarr[i] = 0;
               Yarr[i] = 0;
               useTarget[i] = 1;
               gColinear[i] = 0;
               gCoplanar[i] = 0;
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
                  + sizeof(double) * kNumberOfFlexPoints * 3;
        pGeometricAngles = (double *)(&(parameters)[offset]);
        offset =    sizeof(int32_t) 
                  + sizeof(int32_t)
                  + sizeof(double) * kNumberOfFlexPoints * 3
                  + sizeof(double) * kNumberOfFlexPoints * 2;
        pRawAngles = (uint32_t *)(&(parameters)[offset]);

#ifdef ZDEBUG
fprintf( stderr, "RawGeomFlag %d\n", RawGeomFlag );
#endif
        if ( RawGeomFlag == 1 ) {
           for ( i = 0; i <  numberOfTargets ; i++ ) {
               theAngleBuffer[2*i  ] = pRawAngles[2*i  ];
               theAngleBuffer[2*i+1] = pRawAngles[2*i+1];
               if ( numberOfTargets == 4 ) {
                   theAngleBuffer[2*(i+4)  ] = pRawAngles[2*(i)  ];
                   theAngleBuffer[2*(i+4)+1] = pRawAngles[2*(i)+1];
               }
#ifdef ZDEBUG
fprintf( stderr
       , "raw angle %x %x\n"
       , theAngleBuffer[2*i  ]
       , theAngleBuffer[2*i+1]
       );
#endif
           }
        } else if ( RawGeomFlag == 2 ) {
           for ( i = 0; i <  numberOfTargets ; i++ ) {
	     return_code = ConvertExternalAnglesToBinary(pLgMaster,
							 pGeometricAngles[2*i],
							 pGeometricAngles[2*i+1],
							 &(theAngleBuffer[2*i]),
							 &(theAngleBuffer[2*i+1]));
               
                if ( numberOfTargets == 4 ) {
		  return_code = ConvertExternalAnglesToBinary(pLgMaster,
							      pGeometricAngles[2*(i)],
							      pGeometricAngles[2*(i)+1],
							      &(theAngleBuffer[2*(i+4)]),
							      &(theAngleBuffer[2*(i+4)+1]));
                }
#ifdef ANGDEBUG
fprintf( stderr
       , "geometric %lf %lf "
       , pGeometricAngles[2*i]
       , pGeometricAngles[2*i+1] 
       );
fprintf( stderr
       , "raw angle %x %x\n"
       , theAngleBuffer[2*i  ]
       , theAngleBuffer[2*i+1]
       );
#endif
           }
        } else {
        }
        i = 0;
        index = sizeof(int32_t) + sizeof(int32_t);
        currentData = (double *)(&parameters[index]);
        if ( numberOfTargets > 4 ) {
            while ( i < ( numberOfTargets * 3 ) )
            {
              theCoordinateBuffer[i] = currentData[i];
              i++;
            }
            SaveFullRegCoordinates ( numberOfTargets, theCoordinateBuffer );
        } else {
            while ( i < ( 12 ) )
            {
              theCoordinateBuffer[i]    = currentData[i];
              theCoordinateBuffer[i+12] = currentData[i];
              i++;
            }
            SaveFullRegCoordinates ( 8, theCoordinateBuffer );
        }

        // *****************************************************
        // if numberOfTargets is 4, change it to 8 at this point
        // *****************************************************

        if ( numberOfTargets == 4 ) {
               numberOfTargets = 8;
        }

	lostSensors = 0U;
	
        i = 0;
        if (!return_code) {
	  while ( i < numberOfTargets ) {
		j = gNumberOfSensorSearchAttempts;
		gSearchCurrentSensor = i;

		  /*
		   *  allow for a variable speed search, in needed
		   */
		gCoarse2Factor     = gCoarseFactor;
		pLgMaster->gCoarse2SearchStep = gCoarseSearchStep;
#ifdef ZDEBUG
fprintf( stderr, "i %d useTarget %d\n", i, useTarget[i] );
#endif
		while ( j-- ) {
                        ptX = theAngleBuffer[2*i  ];
                        ptY = theAngleBuffer[2*i+1];
#ifdef ZDEBUG
			fprintf( stderr, "197 about to search i %d j %d\n", i, j );
#endif
#ifdef ZPAUSE
			printf( "PAUSE: ");
			itemp = getchar();
#endif
                        if ( useTarget[i] == 1 ) {
#ifdef ZDEBUG
			  fprintf( stderr, "201 about to search i %d j %d\n", i, j );
#endif
			  searchResult = SearchForASensor ( pLgMaster, ptX,
							    ptY, &fndX, &fndY );
			  if ( searchResult == kStopWasDone ) {
                                fndX = 0;
                                fndY = 0;
                            }
                        } else {
                            searchResult = 99;
                            fndX = 0;
                            fndY = 0;
                        }
                        Xarr[i] = fndX;
                        Yarr[i] = fndY;
			if ( searchResult == kStopWasDone ) {
#ifdef ZDEBUG
fprintf( stderr, "kStopWasDone %d %d\n", gSearchFlag, gStopFlag );
#endif
                              return;
                        }
			if ( !searchResult ) break;
		        pLgMaster->gCoarse2SearchStep /= 2;
		        gCoarse2Factor     /= 2; 
			if (pLgMaster->gCoarse2SearchStep <= 0x00010000 ) {
		        	pLgMaster->gCoarse2SearchStep = 0x00010000;
		        	gCoarse2Factor     = 1;
			}
		}
		gCoarse2Factor     = gCoarseFactor;
		pLgMaster->gCoarse2SearchStep = gCoarseSearchStep;
		
		if ( searchResult ) {
			lostSensors += 1U << i;
                        XfoundAngles[i] = 0;
                        YfoundAngles[i] = 0;
		} else {
                        target_status[i] = 1;
                        foundTarget[i]   = 1;
			ConvertBinaryToGeometricAngles(pLgMaster, fndX, fndY,
						       &(XfoundAngles[i]),
						       &(YfoundAngles[i]) );
			ConvertBinaryToExternalAngles(pLgMaster, fndX, fndY,
						      &(XExternalAngles[i]),
						      &(YExternalAngles[i]));
#ifdef ZDEBUG
fprintf( stderr, "fndX %x ", fndX );
fprintf( stderr, "fndY %x ", fndY );
fprintf( stderr, "\n" );
fprintf( stderr, "curX %lf", XfoundAngles[i] );
fprintf( stderr, "curY %lf", YfoundAngles[i] );
fprintf( stderr, "\n" );
fprintf( stderr, "curX %lf", XExternalAngles[i] );
fprintf( stderr, "curY %lf", YExternalAngles[i] );
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
	
	  numberOfFoundTargets = 0;
          for ( i = 0; i < numberOfTargets ; i++ ) {
             foundAngles[2*i  ] = XfoundAngles[i];
             foundAngles[2*i+1] = YfoundAngles[i];
             if ( foundTarget[i] == 1 )  numberOfFoundTargets ++;
          }
          if ( numberOfFoundTargets >= 4 ) {
	     theResult = FindTransformMatrix ( numberOfTargets
                                          , gDeltaMirror
                                          , theTransformTolerance
                                          , foundAngles
                                          , (double *)&foundTransform
                                          );
          }
          for ( i = 0; i < numberOfTargets ; i++ ) {
              if ( savePoint[i] > 0 ) {
                   target_status[i] = 2;
              }
          }
        } else {
          theResult = false;
        }

             // zero out response buffer
	ucPtr = (unsigned char *)(RespBuff);
       if ( theResult == true ) {
	    return_code = kOK ;
        } else {
            gWorstTolReg = 0.0;
            return_code = kFlexFail;
        }

#ifdef SDEBUG
 for ( i = 0; i < numberOfTargets ; i++ ) {
    printf( "save %d point %d   status %d\n",i, savePoint[i],target_status[i]);
 }
#endif

        // if( gTargetDrift ) {
        //           InitDrift( Xarr, Yarr );
        // }

        /* 
         *  last desperate attempt to find transform
         *  provided that at least four targets have been found
         */
        if ( gSaved == 0 && numberOfFoundTargets >= 4 && gForceTransform ) {
            return_code = kFlexFail;
            theResult = FindTransformMatrix ( numberOfTargets
                                            , gDeltaMirror
                                            , 0.001
                                            , foundAngles
                                            , (double *)&foundTransform
                                            );
               // redo with the suggested tolerance
               // now in variable gWorstTolReg
            if ( theResult == true ) {
                useTol = gWorstTolReg;
                theResult = FindTransformMatrix ( numberOfTargets
                                            , gDeltaMirror
                                            , useTol
                                            , foundAngles
                                            , (double *)&foundTransform
                                            );
            }
#ifdef ZDEBUG
fprintf( stderr, "FullRegWithFeedback last desperate attempt %d\n", theResult );
#endif
            for ( i = 0; i < numberOfTargets ; i++ ) {
              if ( savePoint[i] > 0 ) {
                   target_status[i] = 2;
              }
            }
        }
        if ( gSaved == 0 && numberOfFoundTargets >= 4 && (saveHeaderSpecialByte & 0x20) ) {
            return_code = kFlexFail;
            theResult = FindTransformMatrix ( numberOfTargets
                                            , gDeltaMirror
                                            , 0.001
                                            , foundAngles
                                            , (double *)&foundTransform
                                            );
               // redo with the suggested tolerance
               // now in variable gWorstTolReg
            if ( theResult == true ) {
                useTol = gWorstTolReg;
                theResult = FindTransformMatrix ( numberOfTargets
                                            , gDeltaMirror
                                            , useTol
                                            , foundAngles
                                            , (double *)&foundTransform
                                            );
            }
#ifdef ZDEBUG
fprintf( stderr, "FlexRegWithFeedback last desperate attempt %d\n", theResult );
#endif
            for ( i = 0; i < numberOfTargets ; i++ ) {
              if ( savePoint[i] > 0 ) {
                   target_status[i] = 2;
              }
            }
        }

/*
 *   finally prepare and send response
 */
        for ( i = 0; i < numberOfTargets ; i++ ) {
            if ( savePoint[i] > 0 && gColinear[i] > 0 ) {
                 int32_tColinear = int32_tColinear | (1 << i);
            }
            if ( gCoplanarCount > 0 && gCoplanar[i] > 0 ) {
                 int32_tPlanar = int32_tPlanar | (1 << i);
            }
        }


        if ( theResult != true ) {
            return_code = kFlexFail;
            gWorstTolReg = 0.0;
        }


#ifdef ZDEBUG
fprintf( stderr, "l436 Special %x  result %x\n", saveHeaderSpecialByte, theResult);
#endif

        if ( (saveHeaderSpecialByte & 0x20) != 0x20 ) {

#ifdef ZDEBUG
fprintf( stderr, "l440 Special %x  result %x\n", saveHeaderSpecialByte, theResult);
#endif
            if ( theResult == true ) {
	        return_code = kOK ;
            } else {

#ifdef ZDEBUG
fprintf( stderr, "l446 Special %x  result %x\n", saveHeaderSpecialByte, theResult);
#endif
                return_code = kFlexFail;
                gWorstTolReg = 0.0;
	        index = sizeof(uint32_t);
	        ucPtr = (unsigned char *)(RespBuff + index);
                *(uint32_t *)ucPtr = int32_tColinear;
	        index = 2 * sizeof(uint32_t);
	        ucPtr = (unsigned char *)(RespBuff + index);
                *(uint32_t *)ucPtr = int32_tPlanar;

                        // zero out unused bytes
	        index = 3 * sizeof(uint32_t);
	        ucPtr = (unsigned char *)(RespBuff + index);
                *(uint32_t *)ucPtr = 0;
	        index = 4 * sizeof(uint32_t);
	        ucPtr = (unsigned char *)(RespBuff + index);
                *(uint32_t *)ucPtr = 0;
	        index = 5 * sizeof(uint32_t);
	        ucPtr = (unsigned char *)(RespBuff + index);
                *(uint32_t *)ucPtr = 0;
	        index = 6 * sizeof(uint32_t);
	        ucPtr = (unsigned char *)(RespBuff + index);
                *(uint32_t *)ucPtr = 0;
#ifdef ZDEBUG
		fprintf( stderr, "FRWF397  Colin %x  Plan %x\n", int32_tColinear, int32_tPlanar );
#endif
 
		memcpy(RespBuff, &return_code, sizeof(uint32_t));
		HandleResponse (pLgMaster,
				(sizeof(uint32_t)
				 + sizeof(uint32_t)
				 + sizeof(uint32_t)
				 + sizeof(uint32_t)
				 + sizeof(uint32_t)
				 + sizeof(uint32_t)
				 + sizeof(uint32_t)),
				respondToWhom);
                return;

            }
        }


	index = sizeof(uint32_t);
	ucPtr = (unsigned char *)(RespBuff + index);

        TransformIntoArray( &foundTransform, (double *)ucPtr );

        if ( gSaved == 0 ) {
            gBestTolAll = 0.0;
            gWorstTolAll = 0.0;
            gWorstTolReg = 0.0;
        }

	index = sizeof(uint32_t)
                     + 12 * ( sizeof ( double ) )
                     ;
	ucPtr = (unsigned char *)(RespBuff + index);
        *(double *)ucPtr = gBestTolAll;

#ifdef ZDEBUG
fprintf( stderr, "FRWF520 tol %.9lf %.9lf %.9lf\n", gWorstTolAll, gWorstTolReg, gBestTolAll );
#endif
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
            *(double *)ucPtr = 0.0;
        }

	index = sizeof(uint32_t)
                     + 12 * ( sizeof ( double ) )
                     +        sizeof ( double ) 
                     +        sizeof ( double ) 
                     +        sizeof ( double ) 
                     ;
	ucPtr = (unsigned char *)(RespBuff + index);
        *(int32_t *)ucPtr = GnOfTrans;

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

	for( i=0; i < kNumberOfFlexPoints; i++ ) {
                        ((uint32_t *)ucPtr)[2*i  ] = Xarr[i];
                        ((uint32_t *)ucPtr)[2*i+1] = Yarr[i];
#ifdef ZDEBUG
fprintf( stderr, "FRWF dac xy %08x %08x  i %d\n", Xarr[i], Yarr[i], i );
#endif
        }

	index = sizeof(uint32_t)
                     + 12 * ( sizeof ( double ) )
                     +        sizeof ( double ) 
                     +        sizeof ( double ) 
                     +        sizeof ( double ) 
                     +        sizeof ( int32_t ) 
                     +        sizeof ( int32_t ) 
		     +  2 * kNumberOfFlexPoints * ( sizeof ( uint32_t ) )
                     ;
	ucPtr = (unsigned char *)(RespBuff + index);
	for( i=0; i < kNumberOfFlexPoints; i++ ) {
                        ((double *)ucPtr)[2*i  ] = XExternalAngles[i];
                        ((double *)ucPtr)[2*i+1] = YExternalAngles[i];
#ifdef ZDEBUG
fprintf( stderr, "FRWF extern xy %lf %lf  i %d\n", XExternalAngles[i], YExternalAngles[i], i );
#endif
        }

	index = sizeof(uint32_t)
                     + 12 * ( sizeof ( double ) )
                     +        sizeof ( double ) 
                     +        sizeof ( double ) 
                     +        sizeof ( double ) 
                     +        sizeof ( int32_t ) 
                     +        sizeof ( int32_t ) 
		     +  2 * kNumberOfFlexPoints * ( sizeof ( uint32_t ) )
		     +  2 * kNumberOfFlexPoints * ( sizeof ( double ) )
                     ;
	ucPtr = (unsigned char *)(RespBuff + index);
	for( i=0; i < kNumberOfFlexPoints; i++ ) {
                        ((int32_t *)ucPtr)[i] = target_status[i];
#ifdef ZDEBUG
printf( "FCWF239 save %d point %d   status %d\n",i, savePoint[i],target_status[i]);
#endif

        }

	index = sizeof(uint32_t)
                     + 12 * ( sizeof ( double ) )
                     +        sizeof ( double ) 
                     +        sizeof ( double ) 
                     +        sizeof ( double ) 
                     +        sizeof ( int32_t ) 
                     +        sizeof ( int32_t ) 
		     +  2 * kNumberOfFlexPoints * ( sizeof ( uint32_t ) )
		     +  2 * kNumberOfFlexPoints * ( sizeof ( double ) )
		     +  kNumberOfFlexPoints * ( sizeof ( int32_t ) )
                     ;
	ucPtr = (unsigned char *)(RespBuff + index);
        ((int32_t *)ucPtr)[0] = int32_tColinear;
        ((int32_t *)ucPtr)[1] = 0;
            // for kOK response, only report the coplanar targets
            // if the average error is greater than 0.025
        if ( gFOMavg > 0.025 ) {
            ((int32_t *)ucPtr)[1] = int32_tPlanar;
        }

#ifdef ZDEBUG
	fprintf( stderr, "FRWF617  response %x\n", return_code);
#endif
	memcpy(RespBuff, &return_code, sizeof(uint32_t));
	HandleResponse (pLgMaster, resp_len, respondToWhom );
	return;
}
