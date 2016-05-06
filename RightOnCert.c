/*
static char rcsid[] = "$Id: RightOnCert.c,v 1.2 2000/05/05 23:54:44 ags-sw Exp pickle $";
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
#include "comm_loop.h"
#include "AppCommon.h"
#include "parse_data.h"
#include "3DTransform.h"
#include "AngleCorrections.h"
#include "SensorSearch.h"
#include "Protocol.h"
#include "LaserInterface.h"
#include "RightOnCert.h"

static double uptime(void);

void RightOnCert(struct lg_master *pLgMaster,
		 struct parse_rtoncert_parms *param,
		 uint32_t respondToWhom )
{
   struct parse_rtoncert_resp *pRespBuf;
   transform theTransform;
   double    aXd;
   double    aXe;
   double    aXf;
   double    aXg;
   double    aYd;
   double    aYe;
   double    aYf;
   double    aYg;
   double    inPt[3];
   double    outPt[3];
   double    time_diff;
   double    time_now;
   double    time_start;
   double    Xe;
   double    Xexpect;
   double    Xexternal;
   double    Xf;
   double    Xfound;
   double    XgeomAngle;
   double    Xin;
   double    Xtr;
   double    Ye;
   double    Yexpect;
   double    Yexternal; 
   double    Yf;
   double    Yfound;
   double    YgeomAngle;
   double    Yin;
   double    Ytr;
   double    Zin;
   double    Ztr;
   int32_t   RawGeomFlag;
   int32_t   XrawAngle;
   int32_t   YrawAngle;
   int       status;
   uint32_t  respLen = (sizeof(struct parse_rtoncert_resp) - kCRCSize);
   int16_t   bXe;
   int16_t   bXf;
   int16_t   bYe;
   int16_t   bYf;

   pRespBuf = (struct parse_rtoncert_resp *)pLgMaster->theResponseBuffer;
   memset(pRespBuf, 0, sizeof(respLen));

   time_start = uptime();

   RawGeomFlag = param->inp_angle_flag;

   ArrayIntoTransform((double *)param->inp_transform, &theTransform);

   Xin = param->inp_Xin;
   Yin = param->inp_Yin;
   Zin = param->inp_Zin;

   inPt[0] = Xin;
   inPt[1] = Yin;
   inPt[2] = Zin; 

   TransformPoint(&theTransform, inPt, outPt);

   Xtr = (double)outPt[0];
   Ytr = (double)outPt[1];
   Ztr = (double)outPt[2];

   XrawAngle = param->inp_XrawAngle;
   YrawAngle = param->inp_YrawAngle;

   XgeomAngle = param->inp_XgeomAngle;
   YgeomAngle = param->inp_YgeomAngle;

   GeometricAnglesFrom3D(pLgMaster, Xtr, Ytr, Ztr, &aXd, &aYd);

   ConvertGeometricAnglesToMirror(&aXd, &aYd);

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

   ConvertMirrorToGeometricAngles(&aXg, &aYg);

   XYFromGeometricAnglesAndZ(pLgMaster, aXg, aYg, Ztr, &Xe, &Ye );

   ConvertMirrorAnglesToBinary(aXe, aYe, &bXe, &bYe);

   if (RawGeomFlag == 1)
     {
       bXe = XrawAngle;
       bYe = YrawAngle;
     }
   else if (RawGeomFlag == 2)
     ConvertExternalAnglesToBinary(pLgMaster, XgeomAngle, YgeomAngle, &bXe, &bYe);

   switch(pLgMaster->gHeaderSpecialByte)
     {
       case 0:
         pLgMaster->gCoarse2Factor     = kCoarseFactorDef;
         pLgMaster->gCoarse2SearchStep = kCoarseSrchStpDef;
         break;
       case 1:
         pLgMaster->gCoarse2Factor     = kCoarseFactorMin;
         pLgMaster->gCoarse2SearchStep = kCoarseFactorDef;
         break;
       case 2:
         pLgMaster->gCoarse2Factor     = 2;
         pLgMaster->gCoarse2SearchStep = pLgMaster->gCoarse2Factor;
         break;
       case 3:
         pLgMaster->gCoarse2Factor     = 4;
         pLgMaster->gCoarse2SearchStep = pLgMaster->gCoarse2Factor;
         break;
       case 4:
         pLgMaster->gCoarse2Factor     = 8;
         pLgMaster->gCoarse2SearchStep = pLgMaster->gCoarse2Factor;
         break;
       default:
         pLgMaster->gCoarse2Factor     = kCoarseFactorMin;
         pLgMaster->gCoarse2SearchStep = pLgMaster->gCoarse2Factor;
         break;
     }

   status = SearchForASensor(pLgMaster, bXe, bYe, &bXf, &bYf);

   if (status == kStopWasDone)
     {
       SearchBeamOff(pLgMaster);
       pRespBuf->hdr.status = RESPGOOD;
       HandleResponse(pLgMaster, sizeof(pRespBuf->hdr.hdr), respondToWhom);
       return;
     }

   ConvertBinaryToMirrorAngles(bXf, bYf, &aXf, &aYf);

   ConvertMirrorToGeometricAngles(&aXf, &aYf);

   XYFromGeometricAnglesAndZ(pLgMaster, aXf, aYf, Ztr, &Xf, &Yf);

   Xexpect = Xe;
   Yexpect = Ye;

   Xfound  = Xf;
   Yfound  = Yf;

   time_now = uptime( );
   time_diff = time_now - time_start;

   syslog(LOG_NOTICE, "RightOnCert time diff %lf  end/start %lf %lf", time_diff, time_now, time_start);

   if (status)
     {
       pRespBuf->hdr.status = RESPFAIL;
       HandleResponse(pLgMaster, sizeof(pRespBuf->hdr.hdr), respondToWhom);
       return;
     }
   else
     {
       pRespBuf->hdr.status = RESPGOOD;
       pRespBuf->Xtr = Xtr; 
       pRespBuf->Ytr = Ytr; 
       pRespBuf->Ztr = Ztr; 
       pRespBuf->Xfound = Xfound; 
       pRespBuf->Yfound = Yfound; 
       pRespBuf->Xexpect = Xexpect; 
       pRespBuf->Yexpect = Yexpect; 
       pRespBuf->bXf = bXf;
       pRespBuf->bYf = bYf;
       ConvertBinaryToExternalAngles(pLgMaster, bXf, bYf, &Xexternal, &Yexternal);
       pRespBuf->Xexternal = Xexternal;
       pRespBuf->Yexternal = Yexternal;
       HandleResponse(pLgMaster, respLen, respondToWhom);
       return;
     }
   
   return;
}

static double uptime(void)
{
    double dtime;
    double dummy;
    int    up_fd;
    int    up_size;
    char   up_buff[1024];

    up_fd = open("/proc/uptime", O_RDONLY);

    up_size = read(up_fd, up_buff, 1024);

    if (up_size == 1024) up_size = 1023;

    up_buff[up_size] = 0;

    close(up_fd);

    sscanf(up_buff, "%lf %lf", &dtime, &dummy);

    if (dtime < 0.0 && dtime > 1000000.0)
      {
         dtime = -1.0;
      }
    
    return dtime;
}
