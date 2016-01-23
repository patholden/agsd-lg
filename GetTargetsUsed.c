/*
static char rcsid[] = "$Id: GetTargetsUsed.c ";

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
#include "comm_loop.h"
#include "parse_data.h"
#include "GetTargetsUsed.h"
#include "L3DTransform.h"
#include "Protocol.h"

void GetTargetsUsed(struct lg_master *pLgMaster, uint32_t respondToWhom)
{
  struct parse_tgtsused_resp *pResp;
  int i;

  pResp = (struct parse_tgtsused_resp *)pLgMaster->theResponseBuffer;
  pResp->hdr.status = RESPGOOD;
  memset((char *)pResp, 0, sizeof(struct parse_tgtsused_resp));
  
  pResp->tgtCount = gBestTargetNumber;
  for (i=0; i < 4; i++)
    pResp->tgtNumber[i] = gBestTargetArray[i];

  HandleResponse(pLgMaster, (sizeof(struct parse_tgtsused_resp)-kCRCSize), respondToWhom);
  return;
}
