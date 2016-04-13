#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <syslog.h>
#include <sys/io.h>
#include <string.h>
#include <ctype.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "DoAutoFocusCmd.h"
#include "Protocol.h"
#include "RemoteSerial.h"

void DoAutoFocusCmd(struct lg_master *pLgMaster, unsigned char *buffer)
{
     int AFtoWrite;
     int af_index;
     int write_count=0;
     int read_count=0;
     int num_read=0;
     char cdata_in=0;
     char cdata_out=0;
     
     struct parse_autofocus_parms *pInp = (struct parse_autofocus_parms *)buffer;
     struct parse_autofocus_resp *pResp = (struct parse_autofocus_resp *)pLgMaster->theResponseBuffer;

     // Init response buffer
     memset(pResp, 0, sizeof(struct parse_autofocus_resp));

     // Get length of input buffer & validate length
     AFtoWrite = strlen((const char *)pInp->inp_data );
     if ((AFtoWrite <= 0) || (AFtoWrite > 32) || (pLgMaster->af_serial < 0))
       {
	 syslog(LOG_ERR, " Bad AFWriteLen %d", AFtoWrite);
	 pResp->hdr.status = RESPFAIL;
	 HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp) - kCRCSize), kRespondExtern);
	 return;
       }

     // for LASER mode
     // check if remote computer is to be used
     syslog(LOG_NOTICE, "af special header %x", pLgMaster->gHeaderSpecialByte);
     if ( (pLgMaster->gHeaderSpecialByte & 0x01) && (pLgMaster->projector_mode == PROJ_LASER) ) {

           RemoteSerial( pLgMaster
                       , (char *)buffer
                       , kRespondExtern
                       );

           return;
     }

     // Write buffer out to ttyS2
     // Need to read one byte at a time due to FPGA limitations for serial port
     // design
     syslog(LOG_NOTICE, "AFtoWrite %d  fd %d ", AFtoWrite, pLgMaster->af_serial );
     if ( pLgMaster->af_serial > 0 ) {
       for (af_index = 0; af_index < AFtoWrite; af_index++)
       {
	 cdata_out = pInp->inp_data[af_index];
	 write_count = write(pLgMaster->af_serial, (void *)&cdata_out, 1);
#ifdef AGS_DEBUG
if (isalnum(cdata_out) ) {
syslog(LOG_DEBUG, "afwrite %2x  %d index %d   %c ", cdata_out, write_count, af_index, cdata_out );
} else {
syslog(LOG_DEBUG, "afwrite %2x  %d index %d  ", cdata_out, write_count, af_index );
}
#endif

	 if (write_count != 1)
	   {
	     syslog(LOG_ERR, "AFWRITE: WRITE fail error/count %x, syserr %x, err-count %x", write_count, errno, write_count);
	     pResp->hdr.status = RESPFAIL;
	     HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp) - kCRCSize), kRespondExtern);
	     return;
	   }
       }
     }
     syslog(LOG_NOTICE, "AFtoWrite2 %d ", AFtoWrite );
       usleep(1000);   // Wait 1msec to get data back
       // Need to read one byte at a time due to FPGA limitations for serial port
       // design
       for (af_index = 0; af_index < MAX_DATA; af_index++)
	 {
	   read_count = read(pLgMaster->af_serial, &cdata_in, 1);
#ifdef ZDEBUG
if ( isalnum( cdata_in ) ) {
syslog( LOG_NOTICE, "af read %3d %2x  %c ", af_index, 0xff & cdata_in, cdata_in );
} else {
syslog( LOG_NOTICE, "af read %3d %2x  ", af_index, 0xff & cdata_in );
}
#endif

	   if (read_count == 1)
	     {
	       pLgMaster->gAFInputBuffer[af_index] = cdata_in;
	       num_read++;
	       usleep( 100 );    // We can only read 1 byte at a time with FPGA comms
	     }
	   else
	     {
	       if ((cdata_in == -1) && (errno == EOF))
		 break;
	       if (read_count < 0)
		 {
		   if ( errno != EAGAIN && errno != EWOULDBLOCK )
		     {
		       syslog(LOG_ERR,"AFREAD: FAIL: total_bytes %x, err %x, val %2x",read_count,num_read, (int)cdata_in);
		       pResp->hdr.status = RESPFAIL;
		       HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp) - kCRCSize), kRespondExtern);
		       return;
		     }
		 }
	     }
	 }

     syslog(LOG_NOTICE, "AF num_read %d ", num_read );
       if ((num_read > 0) && (num_read <= AUTOFOCUS_SIZE))
	 {
	   memcpy(pResp->resp_data, pLgMaster->gAFInputBuffer, num_read);
	   pResp->hdr.status = kAutoFocusCmd;
	   HandleResponse(pLgMaster, (sizeof(struct parse_autofocus_resp) - kCRCSize), kRespondExtern);
	 }
       return;
}

