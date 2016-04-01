/*
static char rcsid[] = "$Id: FullRegManager.c,v 1.10 2006/06/15 18:34:22 pickle Exp pickle $";
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
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "AppCommon.h"
#include "comm_loop.h"
#include "LaserInterface.h"
#include "SensorRegistration.h"
#include "SensorSearch.h"
#include "FullRegManager.h"
#include "3DTransform.h"
#include "Protocol.h"

#define	kNumberOfSensorSearchAttempts				5
//FIXME---PAH---NEED TO FIX DEREFERENCING
void PerformAndSendFullReg(struct lg_master *pLgMaster, struct lg_xydata *pXYdata,
			   uint32_t respondToWhom)
{
    struct lg_xydata *pCurXY;
    char *RespBuff=(char *)pLgMaster->theResponseBuffer;
    uint32_t   resp_len=(sizeof(uint32_t) + (12 * kSizeOldLongDouble) +
			 (2 * kNumberOfRegPoints * sizeof(uint32_t)) + kCRCSize);
    uint32_t   return_code=0;
    int16_t ptX, ptY;
    int16_t fndX;
    int16_t fndY;
    double foundAngles [ kNumberOfRegPoints * 2 ];
    int16_t Xarr [ kNumberOfRegPoints ];
    int16_t Yarr [ kNumberOfRegPoints ];
    double *curX, *curY;
    unsigned char *ucPtr;
    unsigned short i, j;
    uint32_t lostSensors;
    unsigned char theResult;
    int index;
    int numberOfFoundTargets;
    double theTransformTolerance;
    transform foundTransform;
    int searchResult;

    memset(RespBuff, 0, resp_len);
    memset((char *)&Xarr, 0, sizeof(Xarr));
    memset((char *)&Yarr, 0, sizeof(Yarr));

    SlowDownAndStop(pLgMaster);
    theTransformTolerance  = pLgMaster->gArgTol;

    for ( index = 0; index < kNumberOfRegPoints; index++)
      foundTarget[index] = 0;

    gWorstTolReg = 1.0;
    numberOfFoundTargets = 0;
    i = 0;
    curX = foundAngles;
    curY = curX;
    curY++;
    lostSensors = 0;

    while (i < kNumberOfRegPoints)
      {
	j = gNumberOfSensorSearchAttempts;
	gSearchCurrentSensor = i;
	/*
	 *  allow for a variable speed search, in needed
	 */
	while (j--)
	  {
	    pCurXY = (struct lg_xydata *)((char *)pXYdata + (sizeof(struct lg_xydata) * i));
	    ptX = pCurXY->xdata;
	    ptY = pCurXY->ydata;
	    searchResult = SearchForASensor(pLgMaster, ptX, ptY, &fndX, &fndY);
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
	    pLgMaster->gCoarse2Factor /= 2; 
	    if (pLgMaster->gCoarse2SearchStep <= 1)
	      {
		pLgMaster->gCoarse2SearchStep = 1;
		pLgMaster->gCoarse2Factor     = 1;
	      }
	  }
	pLgMaster->gCoarse2SearchStep = kCoarseSrchStpDef;
	if (searchResult)
	  lostSensors += 1 << i;
	else
	  {
	    foundTarget[i] = 1;
	    ConvertBinaryToGeometricAngles(pLgMaster,fndX,fndY,curX,curY);
	  }
	
	i++;
	curX++; curX++; curY++, curY++;
      }

    for (i = 0; i < kNumberOfRegPoints; i++)
      {
	if (foundTarget[i] == 1)
	  numberOfFoundTargets++;
      }

    if (numberOfFoundTargets >= 4)
      {
	theResult = FindTransformMatrix(pLgMaster, kNumberOfRegPoints, gDeltaMirror, theTransformTolerance,
					foundAngles, (double *)&foundTransform);
      }
    else
      theResult = 0;

    if (!lostSensors && theResult)
      {
	return_code = kOK | ((0xFF & (uint32_t)GnOfTrans) << 8);
	memcpy(RespBuff, &return_code, sizeof(uint32_t));
	if (gTargetDrift)
	  InitDrift(Xarr, Yarr);

	index = sizeof(uint32_t);
	ucPtr = (unsigned char *)(RespBuff + index);

	memset(ucPtr, 0, 12 * (kSizeOldLongDouble));
	TransformIntoArray(&foundTransform, (double *)ucPtr);

	index = sizeof(uint32_t) + (12 * kSizeOldLongDouble);
	ucPtr = (unsigned char *)(RespBuff + index);
	i = 0;
	for (i = 0; i < kNumberOfRegPoints; i++)
	  {
	    ((uint32_t *)ucPtr)[2*i  ] = Xarr[i];
	    ((uint32_t *)ucPtr)[2*i+1] = Yarr[i];
	  }
	HandleResponse(pLgMaster, resp_len, respondToWhom);
      }
    else
      {
	return_code = kFail | lostSensors;
	memcpy(RespBuff, &return_code, sizeof(uint32_t));
	HandleResponse(pLgMaster, sizeof(uint32_t), respondToWhom);
      }
    return;
}
