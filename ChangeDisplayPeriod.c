/*
static char rcsid[] = "$Id$";

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
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "parse_data.h"
#include "Protocol.h"
#include "comm_loop.h"
#include "AppCommon.h"
#include "ChangeDisplayPeriod.h"

void DoChangeDisplayPeriod (struct lg_master *pLgMaster, struct parse_chngdisp_parms *pInp)
{
  struct parse_chngdisp_resp *pResp;
  uint32_t newPeriod;

  pResp = (struct parse_chngdisp_resp *)pLgMaster->theResponseBuffer;
  memset(pResp, 0, sizeof(struct parse_chngdisp_resp));

  newPeriod = pInp->displayPeriod;
  pResp->hdr.status = RESPGOOD;
  fprintf(stderr,"change display  %d\n", newPeriod );
  if ((newPeriod == -1) || ((newPeriod >= 50) && (newPeriod <= 150)))
    pLgMaster->gPeriod = newPeriod;
  else if (newPeriod == 0)
    pLgMaster->gPeriod = KETIMER_75U;
  else
    pResp->hdr.status =RESPFAIL;

  pResp->displayPeriod = pLgMaster->gPeriod;
  HandleResponse(pLgMaster, (sizeof(struct parse_chngdisp_resp)-kCRCSize),
		 kRespondExtern);
  return;
}
