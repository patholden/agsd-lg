/*
static char rcsid[] = "$Id: DoFindOneTarget.c,v 1.1 2001/01/22 19:51:54 ags-sw Exp $";
*/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <syslog.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "AppCommon.h"
#include "LaserInterface.h"
#include "SensorRegistration.h"
#include "SensorSearch.h"
#include "DoFindOneTarget.h"
#include "3DTransform.h"

#define	kNumberOfSensorSearchAttempts				5

void DoFindOneTarget(struct lg_master *pLgMaster,
		     struct parse_findonetgt_parms *pInp,
		     uint32_t respondToWhom)
{
    struct parse_findonetgt_resp *pResp;
    double XfoundAngle=0;
    double YfoundAngle=0;
    double XExternalAngle=0;
    double YExternalAngle=0;
    double steerX,steerY;
    int16_t ptX=0, ptY=0, fndX=0, fndY=0;
    int rc;

    pResp = (struct parse_findonetgt_resp *)pLgMaster->theResponseBuffer;
    memset(pResp, 0, sizeof(struct parse_findonetgt_resp));

    // Shift input angle to be positive so it falls within 0->0xFFFF.
    steerX = pInp->steerX;
    steerY = pInp->steerY;
    rc = ConvertExternalAnglesToBinary (pLgMaster, steerX, steerY, &ptX, &ptY);
#ifdef AGS_DEBUG
    syslog(LOG_DEBUG,"DOFIND1TGT: XY angle-in X=%f,Y=%f, x=%x,y=%x,rc=%x",steerX,steerY,ptX,ptY,rc);
#endif
    if (rc)
      {
	pResp->hdr.status1 = RESPFAIL;
	pResp->hdr.errtype = RESPE1INANGLEOUTOFRANGE;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	return;
      }

    ClearSensorBuffers();  // Make sure all the buffers used by SearchForASensor are zeroed to start
    rc = SearchForASensor(pLgMaster, ptX, ptY, &fndX, &fndY);
    doLGSTOP(pLgMaster);
#ifdef AGS_DEBUG
    syslog(LOG_DEBUG,"DOFIND1TGT: SEARCHSENSOR x=%x,y=%x,findX %x, findY %x, rc %x",ptX,ptY,fndX,fndY,rc);
#endif
    if (rc == kStopWasDone)
      {
	pResp->hdr.status = RESPFAIL;
	HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
	return;
      }
		
    ConvertBinaryToGeometricAngles(pLgMaster, fndX, fndY, &XfoundAngle, &YfoundAngle);
    ConvertBinaryToExternalAngles(pLgMaster, fndX, fndY, &XExternalAngle, &YExternalAngle);
    if (rc  == 0)
      {
	pResp->hdr.status = RESPGOOD;
	pResp->rawX = fndX;
	pResp->rawY = fndY;
	pResp->geoX = XExternalAngle;
	pResp->geoY = YExternalAngle;
#ifdef AGS_DEBUG
	syslog(LOG_NOTICE,"DOFIND1TGT: FOUND XY TARGET: fndX %x,fndY %x, Angles XY X=%f,Y=%f",fndX,fndY,XExternalAngle,YExternalAngle);
#endif
	HandleResponse (pLgMaster, (sizeof(struct parse_findonetgt_resp)-kCRCSize), respondToWhom );
      }
    else
      {
#ifdef AGS_DEBUG
	syslog(LOG_NOTICE,"DOFIND1TGT: ERROR %x, Bin XY fndX %x,fndY %x",rc,fndX,fndY);
#endif
	pResp->hdr.status = RESPFAIL;
	HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
      }
    return;
}
