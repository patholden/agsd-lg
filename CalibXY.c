#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "LaserInterface.h"
#include "CalibXY.h"
#include "Protocol.h"
#include "comm_loop.h"
// FIXME---PAH---NEEDS CMD/RESP STRUCTS
void
CalibXY(struct lg_master *pLgMaster,
	char * parameters,
	uint32_t respondToWhom)
{
  char *RespBuff=(char *)pLgMaster->theResponseBuffer;
  uint32_t    resp_len=(sizeof(int32_t) + (2 * sizeof(double)) + kCRCSize);
  uint32_t    return_code=0;
  double xgotoang,  ygotoang;
  int16_t xbinary, ybinary;
  double aXf, aYf;
  double Ztr, Xf, Yf;
  char *tmpPtr;
  uint32_t theError;
  int index;

  memset (RespBuff, 0, resp_len);
  
    index    = 0;
    tmpPtr   = &(parameters[index]);
    xgotoang = *((double *)tmpPtr);

    index    = sizeof(double);
    tmpPtr   = &(parameters[index]);
    ygotoang = *((double *)tmpPtr);

    index    = 2 * sizeof(double);
    tmpPtr   = &(parameters[index]);
    Ztr      = *((double *)tmpPtr);


    theError  = ConvertExternalAnglesToBinary(pLgMaster, xgotoang,
                                              ygotoang, &xbinary,
                                              &ybinary);
    if (theError)
      {
	return_code = kFail + kInputAngleOutOfRange;
	memcpy(RespBuff, &return_code, sizeof(uint32_t));
	HandleResponse(pLgMaster, sizeof(uint32_t), kRespondExtern);
	return;
      }
    ConvertBinaryToMirrorAngles(xbinary, ybinary, &aXf, &aYf);
    ConvertMirrorToGeometricAngles( &aXf, &aYf );
    XYFromGeometricAnglesAndZ(pLgMaster, aXf, aYf, Ztr, &Xf, &Yf );
    index    = 0;
    tmpPtr = (char *)(RespBuff + index);
    *((uint32_t *) tmpPtr) = kOK;

    index    = sizeof(uint32_t);
    tmpPtr = (char *)(RespBuff + index);
    *((double *) tmpPtr) = Xf;

    index    = sizeof(uint32_t) + sizeof(double);
    tmpPtr = (char *)(RespBuff + index);
    *((double *) tmpPtr) = Yf;

    HandleResponse (pLgMaster 
                   , sizeof ( uint32_t ) + 2 * sizeof(double)
                   , kRespondExtern
                   );
    return;


}
