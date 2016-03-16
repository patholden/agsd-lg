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
  int16_t ptX=0, ptY=0, fndX=0, fndY=0;
  int rc;
	
  pResp = (struct parse_findonetgt_resp *)pLgMaster->theResponseBuffer;
  memset(pResp, 0, sizeof(struct parse_findonetgt_resp));
  rc = ConvertExternalAnglesToBinary (pLgMaster, pInp->steerX, pInp->steerY, &ptX, &ptY);
#ifdef AGS_DEBUG
  syslog(LOG_NOTICE,"DOFIND1TGT: XY angle x=%x,y=%x",ptX,ptY);
#endif
  pResp->hdr.errtype = htons((uint16_t)rc);
  rc = SearchForASensor(pLgMaster, ptX, ptY, &fndX, &fndY);
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
      syslog(LOG_NOTICE,"DOFIND1TGT: Resp XY angle x=%f,y=%f",XExternalAngle,YExternalAngle);
#endif
      HandleResponse (pLgMaster, (sizeof(struct parse_findonetgt_resp)-kCRCSize), respondToWhom );
    }
  else
    {
      pResp->hdr.status = RESPFAIL;
      HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
    }
  return;
}
