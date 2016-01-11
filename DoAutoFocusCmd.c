#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/io.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>

#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "DoAutoFocusCmd.h"
#include "Protocol.h"

void
DoAutoFocusCmd(struct lg_master *pLgMaster, unsigned char * buffer )
{
     int AFtoWrite;
     int af_index;
     int write_count=0;
     int read_count=0;
     int num_read=0;
     char cdata_in=0;
     struct parse_autofocus_parms *pInp = (struct parse_autofocus_parms *)buffer;
     struct parse_autofocus_resp *pResp = (struct parse_autofocus_resp *)pLgMaster->theResponseBuffer;

     // Init response buffer
     memset(pResp, 0, sizeof(struct parse_autofocus_resp));

     // Get length of input buffer & validate length
     AFtoWrite = strlen((const char *)pInp->inp_data );
     if ((AFtoWrite <= 0) || (AFtoWrite > 32))
       {
	 fprintf(stderr, " Bad AFWriteLen %d\n", AFtoWrite);
	 // Flush ttyS2 to prepare for next operation
	 if (tcflush(pLgMaster->af_serial, TCIOFLUSH))
	   perror("CantFlushttyS2");
	 return;
       }
     // Write buffer out to ttyS2
     write_count = write(pLgMaster->af_serial, pInp->inp_data, AFtoWrite);
     if (write_count != AFtoWrite)
       {
	 fprintf(stderr, "\nAFWRITE: WRITE fail error/count %x, syserr %x, err-count %x", write_count, errno, write_count);
	 if (tcflush(pLgMaster->af_serial, TCIOFLUSH))
	   perror("AFWRITE:CantFlushttyS2");
	 return;
       }
       usleep(1000);   // Wait 1msec to get data back
       
       for (af_index = 0; af_index < MAX_DATA; af_index++)
	 {
	   read_count = read(pLgMaster->af_serial, &cdata_in, 1);
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
		       fprintf(stderr,"\nAFREAD: FAIL: total_bytes %x, err %x, val %2x",read_count,num_read, (int)cdata_in);
		       if (tcflush(pLgMaster->af_serial, TCIOFLUSH))
			 perror("AFWRITE:CantFlushttyS2");
		       return;
		     }
		 }
	     }
	 }

       if ((num_read > 0) && (num_read <= AUTOFOCUS_SIZE))
	 {
	   memcpy(pResp->resp_data, pLgMaster->gAFInputBuffer, num_read);
	   pResp->hdr.status = kAutoFocusCmd;
	   HandleResponse(pLgMaster, (sizeof(struct parse_autofocus_resp) - kCRCSize), kRespondExtern);
	 }

       // Flush ttyS2 to prepare for next operation
       if (tcflush(pLgMaster->af_serial, TCIOFLUSH))
	 perror("CantFlushttyS2");      
       return;
}

