/*
static char rcsid[] = "$Id$";

*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <stddef.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include <assert.h>
#include <sys/io.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <sys/ioctl.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "AppCommon.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "LaserInterface.h"
#include "SensorRegistration.h"
#include "SensorSearch.h"
#include "CalculateTransform.h"
#include "3DTransform.h"
#include "Protocol.h"

void CalculateTransform(struct lg_master *pLgMaster,
		        struct parse_clcxfrm_parms *pInp,
		        uint32_t respondToWhom)
{
    struct k_xyz_double    theCoordinateBuffer[MAX_TARGETSOLD];
    struct k_xy_double     foundAngles[MAX_TARGETSOLD];
    struct k_xy_anglepair  XYarr[MAX_TARGETSOLD];
    transform              foundTransform;
    struct parse_clcxfrm_resp *pResp;
    double Xgeo;
    double Ygeo;
    double angX;
    double angY;
    double theTransformTolerance;
    int16_t  xout, yout;
    uint16_t i;
    uint8_t  theResult;

    syslog(LOG_DEBUG, "Entered Routine: CalcTransform");

    LogCalculateTransformCommand(pInp, pLgMaster);
    
    theTransformTolerance  = pLgMaster->gArgTol;
    pResp = (struct parse_clcxfrm_resp *)pLgMaster->theResponseBuffer;
    memset((char *)pResp, 0, (sizeof(struct parse_clcxfrm_resp)));
    memset((char *)&theCoordinateBuffer, 0, sizeof(theCoordinateBuffer));
    memset((char *)&foundAngles, 0, sizeof(foundAngles));
    memset((char *)&XYarr, 0, sizeof(XYarr));
    memset((char *)&foundTransform, 0, sizeof(foundTransform));

    for (i=0; i < MAX_TARGETSOLD; i++)
      {
	pLgMaster->foundTarget[i] = 1;
	angX = pInp->tgtAngle[i].Xangle;
	angY = pInp->tgtAngle[i].Yangle;
	ConvertExternalAnglesToMirror(angX, angY, &Xgeo, &Ygeo );
	ConvertMirrorToGeometricAngles(&Xgeo, &Ygeo);
	foundAngles[i].Xangle = Xgeo;
	foundAngles[i].Yangle = Ygeo;
	ConvertExternalAnglesToBinary(pLgMaster, Xgeo, Ygeo, &xout, &yout);
	XYarr[i].xangle = xout;
	XYarr[i].yangle = yout;
      } 
    for (i=0; i < MAX_TARGETSOLD; i++)
      {
	theCoordinateBuffer[i].Xtgt = pInp->target[i].Xtgt;
	theCoordinateBuffer[i].Ytgt = pInp->target[i].Ytgt;
	theCoordinateBuffer[i].Ztgt = pInp->target[i].Ztgt;
      }
    SaveFullRegCoordinates(MAX_TARGETSOLD, (double *)&theCoordinateBuffer);
    theResult = FindTransformMatrix(pLgMaster, MAX_TARGETSOLD, gDeltaMirror,
				    theTransformTolerance, (double *)&foundAngles,
				    (double *)&foundTransform);

    /* desperate attempt to get something */
    if (gSaved == 0)
      {
	theResult = FindTransformMatrix(pLgMaster, MAX_TARGETSOLD, gDeltaMirror,
					0.00001, (double *)&foundAngles, (double *)&foundTransform);
	GnOfTrans = 0;
      }
    if (theResult)
      {
	pResp->hdr.status = RESPGOOD;
	pResp->hdr.errtype = htons((uint16_t)((GnOfTrans & 0xFF) << 8));
	TransformIntoArray( &foundTransform, (double *)&pResp->transform[0]);
	memcpy((char *)&pResp->anglepairs[0], (char *)&XYarr[0], OLDANGLEPAIRSLEN);
	LogCalculateTransformResponse(pResp, (sizeof(struct parse_clcxfrm_resp)-kCRCSize));
	HandleResponse(pLgMaster, (sizeof(struct parse_clcxfrm_resp)-kCRCSize), respondToWhom );
      }
    else
      {
	pResp->hdr.status = RESPFAIL;
	LogCalculateTransformResponse(pResp, (sizeof(struct parse_basic_resp)-kCRCSize));
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
      }
    return;
}


void LogCalculateTransformCommand(struct parse_clcxfrm_parms *param, struct lg_master *pLgMaster)
{
    int32_t          i;
   
    syslog(LOG_DEBUG, "CMD: HeaderSpecialByte: %02x", pLgMaster->gHeaderSpecialByte);

    for (i = 0; i < MAX_TARGETSOLD; i++)
      {
        syslog(LOG_DEBUG, "CMD: target #%d: target[%d].Xtgt: %f   target[%d].Ytgt: %f   target[%d].Ztgt: %f",
               i + 1,
	       i,
  	       param->target[i].Xtgt,
	       i,
	       param->target[i].Ytgt,
	       i,
	       param->target[i].Ztgt);
	}

    for (i = 0; i < MAX_TARGETSOLD; i++)
      {
        syslog(LOG_DEBUG, "CMD: target #%d: tgtAngle[%d].Xangle: %f   tgtAngle[%d].Yangle: %f",
    	       i + 1,
	       i,
	       param->tgtAngle[i].Xangle,
	       i,
	       param->tgtAngle[i].Yangle);
      }
      
    return;    
}


void LogCalculateTransformResponse(struct parse_clcxfrm_resp *pRespBuf, uint32_t respLen)
{
    double           transform[MAX_NEW_TRANSFORM_ITEMS];
    int32_t          i;
    
    syslog(LOG_DEBUG, "RSP: hdr: %08x", pRespBuf->hdr.hdr);

    if (respLen <= sizeof(pRespBuf->hdr.hdr))
      return;

    memcpy(transform, pRespBuf->transform, sizeof(transform));
    for (i = 0; i < MAX_NEW_TRANSFORM_ITEMS; i++)
      {
	syslog(LOG_DEBUG, "RSP: transform[%d]: %f", i, transform[i]);
      }

    for (i = 0; i < MAX_TARGETSOLD; i++)
      {
        syslog(LOG_DEBUG, "RSP: target #%d: anglepairsX[%d]: %d   anglepairsY[%d]: %d",
	       i + 1,
	       2*i+0,
	       ((uint32_t *)(&(pRespBuf->anglepairs[0])))[2*i+0],
	       2*i+1,
  	       ((uint32_t *)(&(pRespBuf->anglepairs[0])))[2*i+1]);
      }

    return;
}
