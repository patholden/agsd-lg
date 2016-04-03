/*
static char rcsid[] = "$Id$";

*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
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
    int16_t ptX, ptY, fndX, fndY;
    double XfoundAngles [ kNumberOfFlexPoints ];
    double YfoundAngles [ kNumberOfFlexPoints ];
    double XExternalAngles [ kNumberOfFlexPoints ];
    double YExternalAngles [ kNumberOfFlexPoints ];
    double foundAngles [ kNumberOfFlexPoints * 2 ];
    int16_t Xarr [ kNumberOfFlexPoints ];
    int16_t Yarr [ kNumberOfFlexPoints ];
    int16_t target_status [ kNumberOfFlexPoints ];
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
    uint16_t      i, j;
    uint32_t      lostSensors;
    unsigned char theResult;
    uint32_t      numberOfTargets;
    int32_t       RawGeomFlag;
    double theCoordinateBuffer[kNumberOfFlexPoints * 3];
    int32_t theAngleBuffer[kNumberOfFlexPoints * 2];
    double *currentData;
    struct lg_xydata *pCurXY;
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

    syslog(LOG_DEBUG, "PMA: in FlexFullRegWithFeedback");
    
    // Stop display by driver that may be in progress
    SlowDownAndStop(pLgMaster);

    // Initialize variables
    saveHeaderSpecialByte = pLgMaster->gHeaderSpecialByte;
    theTransformTolerance  = pLgMaster->gArgTol;
    int32_tColinear = 0;
    int32_tPlanar = 0;
    pLgMaster->gBestTargetNumber = 0;
    gWorstTolReg = 1.0;
    i = 0;

    //Initialize buffers
    memset(RespBuff, 0, resp_len);
    memset(pLgMaster->gBestTargetArray, 0, sizeof(pLgMaster->gBestTargetArray));
    memset((char *)&target_status, 0, sizeof(target_status));
    memset((char *)&foundTarget, 0, sizeof(foundTarget));
    memset((char *)&savePoint, 0, sizeof(savePoint));
    memset((char *)&XExternalAngles, 0, sizeof(XExternalAngles));
    memset((char *)&YExternalAngles, 0, sizeof(YExternalAngles));
    memset((char *)&Xarr, 0, sizeof(Xarr));
    memset((char *)&Yarr, 0, sizeof(Yarr));
    memset((char *)&useTarget, 0, sizeof(useTarget));
    memset((char *)&gColinear, 0, sizeof(gColinear));
    memset((char *)&gCoplanar, 0, sizeof(gCoplanar));


    //FIXME---PAH---NEED TO DO CMD/RESP HERE	
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
    
    if ( RawGeomFlag == 1 ) {
      for ( i = 0; i <  numberOfTargets ; i++ ) {
	theAngleBuffer[2*i  ] = pRawAngles[2*i  ];
	theAngleBuffer[2*i+1] = pRawAngles[2*i+1];
	if ( numberOfTargets == 4 ) {
	  theAngleBuffer[2*(i+4)  ] = pRawAngles[2*(i)  ];
	  theAngleBuffer[2*(i+4)+1] = pRawAngles[2*(i)+1];
	}
      }
    } else if ( RawGeomFlag == 2 ) {
      for ( i = 0; i <  numberOfTargets ; i++ ) {
	pCurXY = (struct lg_xydata *)((char *)&theAngleBuffer[0] + (sizeof(struct lg_xydata) * i));
	return_code = ConvertExternalAnglesToBinary(pLgMaster,
							 pGeometricAngles[2*i],
							 pGeometricAngles[2*i+1],
							 &pCurXY->xdata,
							 &pCurXY->ydata);
               
	if ( numberOfTargets == 4 ) {
	  pCurXY = (struct lg_xydata *)((char *)&theAngleBuffer[0] + (sizeof(struct lg_xydata) * 4));
	  
	  return_code = ConvertExternalAnglesToBinary(pLgMaster,
						      pGeometricAngles[2*(i)],
						      pGeometricAngles[2*(i)+1],
						      &pCurXY->xdata,
						      &pCurXY->ydata);
	}
      }
    } else {
    }
    i = 0;
    index = sizeof(int32_t) + sizeof(int32_t);
    currentData = (double *)(&parameters[index]);
    syslog(LOG_DEBUG, "PMA: numberOfTargets:  %d", numberOfTargets);
    if ( numberOfTargets > 4 ) {
      while ( i < ( numberOfTargets * 3 ) )
	{
	  theCoordinateBuffer[i] = currentData[i];
	  i++;
	  syslog(LOG_DEBUG, "PMA: target data: %f", theCoordinateBuffer[i]);
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
    
    if ( numberOfTargets == 4 )
      numberOfTargets = 8;

    lostSensors = 0;
    i = 0;
    if (!return_code) {
      while ( i < numberOfTargets ) {
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
                        ptX = pCurXY->xdata;
                        ptY = pCurXY->ydata;
                        if ( useTarget[i] == 1 ) {
			  searchResult = SearchForASensor ( pLgMaster, ptX,
							    ptY, &fndX, &fndY );
			  if (searchResult == kStopWasDone)
			    {
			      SearchBeamOff(pLgMaster);
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
			if (searchResult == kStopWasDone)
			  return;
			if (!searchResult)
			  break;
		        pLgMaster->gCoarse2SearchStep /= 2;
		        pLgMaster->gCoarse2Factor     /= 2; 
			if (pLgMaster->gCoarse2SearchStep < kCoarseSrchStpMin) {
		        	pLgMaster->gCoarse2SearchStep = kCoarseSrchStpMin;
		        	pLgMaster->gCoarse2Factor     = kCoarseFactorMin;
			}
		}
		pLgMaster->gCoarse2Factor     = kCoarseFactorDef;
		pLgMaster->gCoarse2SearchStep = kCoarseSrchStpDef;
		
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
	    theResult = FindTransformMatrix(pLgMaster, numberOfTargets,
					    gDeltaMirror, theTransformTolerance,
					    foundAngles, (double *)&foundTransform);
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

        /* 
         *  last desperate attempt to find transform
         *  provided that at least four targets have been found
         */
        if ( gSaved == 0 && numberOfFoundTargets >= 4 && gForceTransform ) {
            return_code = kFlexFail;
            theResult = FindTransformMatrix(pLgMaster, numberOfTargets, gDeltaMirror, 0.001,
                                            foundAngles, (double *)&foundTransform);
               // redo with the suggested tolerance
               // now in variable gWorstTolReg
            if ( theResult == true ) {
                useTol = gWorstTolReg;
                theResult = FindTransformMatrix(pLgMaster, numberOfTargets, gDeltaMirror,
						useTol, foundAngles, (double *)&foundTransform);
            }
            for ( i = 0; i < numberOfTargets ; i++ ) {
              if ( savePoint[i] > 0 ) {
                   target_status[i] = 2;
              }
            }
        }
        if ( gSaved == 0 && numberOfFoundTargets >= 4 && (saveHeaderSpecialByte & 0x20) ) {
            return_code = kFlexFail;
            theResult = FindTransformMatrix(pLgMaster, numberOfTargets, gDeltaMirror, 0.001,
                                            foundAngles, (double *)&foundTransform);
               // redo with the suggested tolerance
               // now in variable gWorstTolReg
            if ( theResult == true ) {
                useTol = gWorstTolReg;
                theResult = FindTransformMatrix(pLgMaster, numberOfTargets, gDeltaMirror, useTol,
                                            foundAngles, (double *)&foundTransform);
            }
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

        if ( (saveHeaderSpecialByte & 0x20) != 0x20 )
	  {
            if (theResult == true)
	        return_code = kOK;
            else
	      {
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

	memcpy(RespBuff, &return_code, sizeof(uint32_t));
	HandleResponse (pLgMaster, resp_len, respondToWhom );
	return;
}
