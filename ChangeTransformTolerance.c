/*
static char rcsid[] = "$Id$";

*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>

#include "BoardComm.h"
#include "Protocol.h"
#include "comm_loop.h"
#include "AppCommon.h"
#include "ChangeTransformTolerance.h"

// FIXME---PAH NEEDS CMD/RESP STRUCTS
void DoChangeTransformTolerance (struct lg_master *pLgMaster, char * parameters )
{
  char        *RespBuff=(char *)pLgMaster->theResponseBuffer;
  uint32_t    resp_len=(sizeof(uint32_t) + sizeof(double) + kCRCSize);
  uint32_t    return_code=0;
  double      newTol;
  
  memset(RespBuff, 0, resp_len);
  newTol = *((double *)&(parameters[0]));
  fprintf(stderr,"change display  %f\n", newTol );

  if ((newTol >= 1.0e-10) && (newTol < 1.0))
    {
      return_code = kOK;
      pLgMaster->gArgTol = newTol;
    }
  else if (fabs(newTol+1.0) < 0.00001)
    return_code = kOK;
  else if (fabs(newTol) < 1.0e-30)
    {
      pLgMaster->gArgTol = GARGTOL_DEFAULT;
      return_code = kOK;
    }
  else
    return_code = kFail;
  memcpy(RespBuff, &return_code, sizeof(uint32_t));
  memcpy((char *)(RespBuff+4), &pLgMaster->gArgTol, sizeof(double));
  HandleResponse(pLgMaster, resp_len, kRespondExtern);
  return;
}
