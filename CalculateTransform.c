/*
static char rcsid[] = "$Id$";

*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
#include "parse_data.h"
#include "AppResponses.h"
#include "LaserInterface.h"
#include "SensorRegistration.h"
#include "SensorSearch.h"
#include "CalculateTransform.h"
#include "3DTransform.h"
#include "Protocol.h"

void CalculateTransform ( struct lg_master *pLgMaster,
			  struct parse_clctrnsfrm_parms* pInp,
			  uint32_t respondToWhom )
{
        double theCoordinateBuffer[kNumberOfRegPoints * 3];
	double foundAngles [ kNumberOfRegPoints * 2 ];
	uint32_t Xarr [ kNumberOfRegPoints ];
	uint32_t Yarr [ kNumberOfRegPoints ];
	struct parse_clctrnsfrm_resp *pResp;
	uint32_t *pOut;
	double Xgeo;
	double Ygeo;
	unsigned short i;
	unsigned char theResult;
	double theTransformTolerance;
	transform foundTransform;
        double angX;
        double angY;
	
	theTransformTolerance  = pLgMaster->gArgTol;
	pResp = (struct parse_clctrnsfrm_resp *)pLgMaster->theResponseBuffer;
	memset((char *)pResp, 0, (sizeof(struct parse_clctrnsfrm_resp)));

        i = 0;
        while ( i < kNumberOfRegPoints ) {
            foundTarget[i] = 1;
            angX = pInp->tgtAngle[2*i  ];
            angY = pInp->tgtAngle[2*i+1];
            ConvertExternalAnglesToMirror(angX, angY, &Xgeo, &Ygeo );
            ConvertMirrorToGeometricAngles(&Xgeo, &Ygeo);
            foundAngles[2*i  ] = Xgeo;
            foundAngles[2*i+1] = Xgeo;
            ConvertExternalAnglesToBinary(pLgMaster, Xgeo, Ygeo, &Xarr[i], &Yarr[i]);
            i++;
        } 

        i = 0;
        while (i < (kNumberOfRegPoints * 3))
	  {
	    theCoordinateBuffer[i] = pInp->target[i];
	    i++;
	  }
        SaveFullRegCoordinates ( kNumberOfRegPoints, theCoordinateBuffer );

        theResult = FindTransformMatrix ( kNumberOfRegPoints
                                        , gDeltaMirror
                                        , theTransformTolerance
                                        , foundAngles
                                        , (double *)&foundTransform
                                         );

          /* desperate attempt to get something */
        if ( gSaved == 0 ) {
            theResult = FindTransformMatrix ( kNumberOfRegPoints
                                            , gDeltaMirror
                                            , 0.00001
                                            , foundAngles
                                            , (double *)&foundTransform
                                             );
            GnOfTrans = 0;
        }

	if (theResult)
	  {
	    pResp->hdr.status = RESPGOOD;
	    pResp->hdr.errtype = htons((uint16_t)((GnOfTrans & 0xFF) << 8));
	    TransformIntoArray( &foundTransform, (double *)&pResp->transform[0]);
	    pOut = (uint32_t *)&pResp->anglepair[0];
	    for (i=0; i < kNumberOfRegPoints; i++)
	      {
		pOut[2*i] = Xarr[i];
		pOut[2*i+1] = Yarr[i];
	      }
	    HandleResponse (pLgMaster, (sizeof(struct parse_clctrnsfrm_resp)-kCRCSize),
			    respondToWhom );
	  }
	else
	  {
	    pResp->hdr.status = RESPFAIL;
	    HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
	  }
	return;

}
