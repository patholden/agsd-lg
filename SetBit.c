#include <stdint.h>
#include <time.h>
#include <stdio.h> 
#include <stdlib.h>
#include <string.h>
#include <sys/io.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <sys/ioctl.h>
#include <linux/laser_ioctl.h>
#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "Protocol.h"
#include "SetBit.h"
#include "CRCHandler.h"
#include "laserguide.h"

void DoSetBit(struct lg_master *pLgMaster, char * data, uint32_t respondToWhom )
{
      int index;
      struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
      char * tmpPtr;
      unsigned int bitID;
      unsigned int bitValue;
      struct lg_info  loc_info;
      uint32_t last_xpos;
      uint32_t last_ypos;

      memset((char *)&loc_info, 0, sizeof(loc_info));
      
      index      = 0 * sizeof( unsigned int );
      tmpPtr     = &(data[index]);
      bitID      = *(unsigned int *)tmpPtr;

      index      = 1 * sizeof( unsigned int );
      tmpPtr     = &(data[index]);
      bitValue   = *(unsigned int *)tmpPtr;

      pResp->hdr.status = RESPGOOD;
      switch ( bitID )
	{
	case 1:
	  // unblank intensity
	  doLGSTOP(pLgMaster);
	  doStopPulse(pLgMaster);
	  ioctl( pLgMaster->fd_laser, LGGETANGLE, &loc_info );
	  last_xpos = loc_info.ptns.xpos;
	  last_ypos = loc_info.ptns.ypos;
	  pLgMaster->optic_status &= 0xBF;
	  outb(pLgMaster->optic_status, LG_IO_OPTIC);
	  if (bitValue == 0)
	    doWriteDevPoints(pLgMaster, (last_xpos & 0xFFFFFF00), last_ypos);
	  else if ( bitValue == 1 )
	    doWriteDevPoints(pLgMaster, (last_xpos | 0x40), last_ypos);
	  else
	    pResp->hdr.status = RESPFAIL;
	  break;
	case 2:
	  // search intensity
          doLGSTOP(pLgMaster);
	  doStopPulse(pLgMaster);
	  ioctl( pLgMaster->fd_laser, LGGETANGLE, &loc_info );
	  last_xpos = loc_info.ptns.xpos;
	  last_ypos = loc_info.ptns.ypos;

	  if (bitValue == 0)
	    {
	      pLgMaster->optic_status &= 0xBF;
	      outb(pLgMaster->optic_status, LG_IO_OPTIC);
	      doWriteDevPoints(pLgMaster, (last_xpos & 0xFFFFFF00), last_ypos);
	    }
	  else if ( bitValue == 1)
	    {
	      pLgMaster->optic_status |= 0x40;
	      outb(pLgMaster->optic_status, LG_IO_OPTIC);
	      doWriteDevPoints(pLgMaster, (last_xpos | 0x40), last_ypos);
	    }
	  else {
	    pResp->hdr.status = RESPFAIL;
	  }
	  break;
	default:
	  pResp->hdr.status = RESPFAIL;
	  break;
	}
      HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp) -kCRCSize), respondToWhom);
      return;
}
