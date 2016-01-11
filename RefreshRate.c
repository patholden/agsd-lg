/*
static char rcsid[] = "$Id: FOM.c,v 1.2 1999/05/04 15:32:35 ags-sw Exp $";

*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>

#include "BoardComm.h"
#include "parse_data.h"
#include "Protocol.h"
#include "comm_loop.h"
#include "SensorRegistration.h"

void DoRefreshRate (struct lg_master *pLgMaster, uint32_t respondToWhom )
{
  struct parse_rfrshrt_resp *pResp;

  pResp = (struct parse_rfrshrt_resp *)pLgMaster->theResponseBuffer;
  memset(pResp, 0, sizeof(struct parse_rfrshrt_resp));
  pResp->hdr.status = RESPGOOD;
  pResp->num_points = 0;
  pResp->laser_period = pLgMaster->gPeriod;
  HandleResponse(pLgMaster, (sizeof(struct parse_rfrshrt_resp)-kCRCSize), respondToWhom);
  return;
}
