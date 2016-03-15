/*
static char rcsid[] = "$Id: RightOnCert.c,v 1.2 2000/05/05 23:54:44 ags-sw Exp pickle $";
*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <syslog.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "comm_loop.h"
#include "AppCommon.h"
#include "parse_data.h"
#include "3DTransform.h"
#include "AngleCorrections.h"
#include "SensorSearch.h"
#include "AutoCert.h"
#include "Protocol.h"
#include "LaserInterface.h"

static double uptime(void);

void RightOnCert(struct lg_master *pLgMaster,
		 char *parameters,
		 uint32_t respondToWhom )
{
   double Xin;
   double Yin;
   double Zin;
   transform theTransform;
   double XgeomAngle;
   double YgeomAngle;
   int32_t XrawAngle;
   int32_t YrawAngle;
   double Xexpect;
   double Yexpect;
   double Xfound;
   double Yfound;
   int status;
   double inPt[3];
   double outPt[3];
   double Xtr, Ytr, Ztr;
   int32_t RawGeomFlag;
   struct parse_basic_resp *pRespErr=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
   struct parse_rtoncert_resp *pResp=(struct parse_rtoncert_resp *)parameters;
   struct parse_rtoncert_parms *pInp=(struct parse_rtoncert_parms *)parameters;
   double aXd, aYd;
   double aXe, aYe;
   double aXf, aYf;
   double aXg, aYg;
   double Xe, Ye;
   double Xf, Yf;
   int16_t bXe, bYe, bXf, bYf;
   double Xexternal; 
   double Yexternal; 
   double time_now, time_diff, time_start;
 
   memset(pResp, 0, sizeof(struct parse_rtoncert_resp));
   time_start = uptime();

   RawGeomFlag = pInp->inp_angle_flag;
   ArrayIntoTransform( (double *)pInp->inp_transform, &theTransform );
   Xin = pInp->inp_Xin;
   Yin = pInp->inp_Yin;
   Zin = pInp->inp_Zin;
   inPt[0] = Xin; inPt[1] = Yin; inPt[2] = Zin; 
   TransformPoint( &theTransform, inPt, outPt );
   Xtr = (double)outPt[0];
   Ytr = (double)outPt[1];
   Ztr = (double)outPt[2];
   XrawAngle = pInp->inp_XrawAngle;
   YrawAngle = pInp->inp_YrawAngle;
   XgeomAngle = pInp->inp_XgeomAngle;
   YgeomAngle = pInp->inp_YgeomAngle;
   GeometricAnglesFrom3D(pLgMaster, Xtr, Ytr, Ztr, &aXd, &aYd );
   ConvertGeometricAnglesToMirror( &aXd, &aYd );
   if (pLgMaster->gCALIBFileOK)
     {
       aXe = aXd;
       aYe = aYd;
       ApplyCorrection(pLgMaster, &aXe, &aYe);
     }
   else
     {
       aXe = aXd;
       aYe = aYd;
     }
   aXg = aXe;
   aYg = aYe;
   ConvertMirrorToGeometricAngles( &aXg, &aYg );
   XYFromGeometricAnglesAndZ(pLgMaster, aXg, aYg, Ztr, &Xe, &Ye );
   ConvertMirrorAnglesToBinary( aXe, aYe, &bXe, &bYe );
   if (RawGeomFlag == 1)
     {
       bXe = XrawAngle;
       bYe = YrawAngle;
   }
   else if (RawGeomFlag == 2)
     ConvertExternalAnglesToBinary(pLgMaster, XgeomAngle, YgeomAngle, &bXe, &bYe );
   switch(pLgMaster->gHeaderSpecialByte)
     {
     case 0:
       gCoarse2Factor     = gCoarseFactor;
       pLgMaster->gCoarse2SearchStep = gCoarseSearchStep;
       break;
     case 1:
       gCoarse2Factor     = 1;
       pLgMaster->gCoarse2SearchStep = gCoarse2Factor *  0x00010000;
       break;
     case 2:
       gCoarse2Factor     = 2;
       pLgMaster->gCoarse2SearchStep = gCoarse2Factor *  0x00010000;
       break;
     case 3:
       gCoarse2Factor     = 4;
       pLgMaster->gCoarse2SearchStep = gCoarse2Factor *  0x00010000;
       break;
     case 4:
       gCoarse2Factor     = 8;
       pLgMaster->gCoarse2SearchStep = gCoarse2Factor *  0x00010000;
       break;
     default:
       gCoarse2Factor     = 1;
       pLgMaster->gCoarse2SearchStep = gCoarse2Factor *  0x00010000;
       break;
     }
   status = SearchForASensor(pLgMaster, bXe, bYe, &bXf, &bYf);
   if (status == kStopWasDone)
     {
       SearchBeamOff(pLgMaster);
       pRespErr->hdr.status = RESPGOOD;
       HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
       return;
     }
   ConvertBinaryToMirrorAngles( bXf, bYf, &aXf, &aYf );
   ConvertMirrorToGeometricAngles( &aXf, &aYf );
   XYFromGeometricAnglesAndZ(pLgMaster, aXf, aYf, Ztr, &Xf, &Yf );
   Xexpect = Xe;
   Yexpect = Ye;
   Xfound  = Xf;
   Yfound  = Yf;
   time_now = uptime( );
   time_diff = time_now - time_start;
   syslog(LOG_NOTICE, "RightOnCert time diff %lf  end/start %lf %lf",
	  time_diff, time_now, time_start);

   if (status)
     {
       pRespErr->hdr.status = RESPFAIL;
       HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
       return;
     }
   else
     {
       pResp->hdr.status = RESPGOOD;
       pResp->Xtr = Xtr; 
       pResp->Ytr = Ytr; 
       pResp->Ztr = Ztr; 
       pResp->Xfound = Xfound; 
       pResp->Yfound = Yfound; 
       pResp->Xexpect = Xexpect; 
       pResp->Yexpect = Yexpect; 
       pResp->bXf = bXf;
       pResp->bYf = bYf;
       ConvertBinaryToExternalAngles(pLgMaster, bXf, bYf, &Xexternal, &Yexternal);
       pResp->Xexternal = Xexternal;
       pResp->Yexternal = Yexternal;
       HandleResponse (pLgMaster, (sizeof(struct parse_rtoncert_resp)-kCRCSize), respondToWhom);
       return;
     }
   return;
}

static double uptime(void)
{
    int up_fd;
    int up_size;
    char up_buff[1024];
    double dtime, dummy;

    up_fd = open( "/proc/uptime", O_RDONLY );
    up_size = read( up_fd, up_buff, 1024 );
    if ( up_size == 1024 ) up_size = 1023;
    up_buff[up_size] = 0;
    close( up_fd );

    sscanf( up_buff, "%lf %lf", &dtime, &dummy );

    if ( dtime < 0.0 && dtime > 1000000.0 ) {
         dtime = -1.0;
    } 
    return dtime;
}
