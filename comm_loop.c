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
#include <linux/laser_api.h>
#include <syslog.h>
#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "Protocol.h"
#include "LaserCmds.h"
#include "SystemMaint.h"
#include "CRCHandler.h"

#define MAXPENDING  5   // Max # connection requests
#define AGS_PORT  1234

extern int
pingClient( char* peerAddr );

int CommConfigSockfd(struct lg_master *pLgMaster)
{    
  int                sockaddr_len = sizeof(struct sockaddr_in);
  int                error;
  
  // Initialize buffers to 0
  memset(&pLgMaster->webhost_addr, 0, sockaddr_len);
  
  // Just in case this is a re-config, check for open socketfd & close first
  if ((pLgMaster->socketfd >= 0) || (pLgMaster->datafd >= 0))
    {
      if (pLgMaster->socketfd >= 0)
	close(pLgMaster->socketfd);
      if (pLgMaster->datafd >= 0)
	close(pLgMaster->datafd);
      pLgMaster->socketfd = -1;
      pLgMaster->datafd = -1;
    }

  // Set up comm interface with PC Host
  pLgMaster->socketfd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (pLgMaster->socketfd < 0)
    {
      perror("server socket: ");
      return(-1);
    }

  pLgMaster->webhost_addr.sin_family = AF_INET;
  pLgMaster->webhost_addr.sin_port = htons(AGS_PORT);
  pLgMaster->webhost_addr.sin_addr.s_addr = INADDR_ANY;
  error = bind(pLgMaster->socketfd, (struct sockaddr *)&pLgMaster->webhost_addr, sockaddr_len);
  if (error < 0)
    {
      perror("COMMCFGSCK: Bind failed ");
      return(-2);
    }
  error = listen(pLgMaster->socketfd, MAXPENDING);
  if (error < 0)
    {
      perror("COMMCFGSCK: connect failed ");
      return(-3);
    }

  pLgMaster->enet_retry_count++;
  pLgMaster->serial_ether_flag = 2;
  return(0);
}

int CommInit(struct lg_master *pLgMaster)
{
  struct termios term;
  int            baud;
  int            error=0;
  
  // Always check integrity of master struct
  if (!pLgMaster)
    {
      perror("Need master data configured!");
      return(-1);
    }

  // Try to open Comms interface to PC host
  if ((error = CommConfigSockfd(pLgMaster)) < 0)
    {
      perror("opensock");
      return(error);
    }
  syslog(LOG_NOTICE, "\nPC host comm port initialized\n");

  // Try to open AutoFocus serial port
  pLgMaster->af_serial = open("/dev/ttyS2", O_RDWR | O_NONBLOCK | O_NOCTTY);
  if (pLgMaster->af_serial <= 0)
    {
      syslog(LOG_ERR,"open serial port 2 failed");
      return(-5);
    }
  // Need to zero struct, driver may not return all parms
  memset(&term, 0, sizeof(term));
  if( tcgetattr( pLgMaster->af_serial, &term) < 0 ){
    syslog(LOG_ERR,"Unable to get serial port 2 attributes");
    return(-6);
  }
  // Set up device to our settings
  term.c_cflag = (term.c_cflag & ~CSIZE) | CS8; // 8 bits
  term.c_cflag &= ~(PARENB | PARODD | CMSPAR);  // no parity 
  term.c_cflag &= ~CSTOPB;                      // One stop bit
  term.c_cflag &= ~(CRTSCTS);                   // Flow control off
  term.c_iflag &= ~(IXON | IXOFF | IXANY);
  term.c_cflag |= CLOCAL;                       // ignore modem status lines  
  baud  = B115200;

  // Disable canonical mode, and set buffer size to 1 byte
  term.c_lflag &= ~ICANON;
  term.c_lflag &= ~ECHO;
  term.c_lflag &= ~ISIG;
  term.c_cc[VMIN] = 1;
  term.c_cc[VTIME] = 0;
	
  // Set input & output baud rate
  cfsetispeed( &term, baud );
  cfsetospeed( &term, baud );

  // Enable the receiver and set local mode...
  term.c_cflag |= ( CLOCAL | CREAD );
  term.c_cflag &= ~PARENB;
  term.c_cflag &= ~CSTOPB;
  term.c_cflag &= ~CSIZE;
  term.c_cflag |= CS8;

  //Attempt to flush IO
  if (tcflush(pLgMaster->af_serial, TCIOFLUSH))
    {
      perror("CantFlushttyS2");
      close(pLgMaster->af_serial);
      return(-7);
    }

  //Set new term attributes
  cfmakeraw(&term);
  if( tcsetattr( pLgMaster->af_serial, TCSANOW, &term) < 0 )
    {
      perror("tcsetattr2");
      close(pLgMaster->af_serial);
      return(-8);
    }

  syslog(LOG_NOTICE, "\nAutoFocus port initialized, baud %d\n", baud);
  // This sets the correct data presentation on send/recv from webhost
  pLgMaster->gHEX = 1;
  return(0);
}

static int ProcEnetPacketsFromHost(struct lg_master *pLgMaster)
{
  unsigned char *recv_data = 0;
  int       data_len = 0;
  int       error = 0;
  int       parsed_count = 0;
  
  if (!pLgMaster)
    return(-1);

  if (pLgMaster->serial_ether_flag != 2)
    return(-2);

  recv_data = (unsigned char *)malloc(COMM_RECV_MAX_SIZE);   // We only read in up to 8k at a time.
  if (!recv_data)
    return(-3);

  // Prep buffers for accepting/receiving new packet
  memset(recv_data, 0, COMM_RECV_MAX_SIZE);

  // We've got our socket fd for client, now look for data
  data_len = recv(pLgMaster->datafd, recv_data, COMM_RECV_MAX_SIZE, 0);
  if (data_len <= 0)
    {
      if (data_len < 0)
	{
	  if (data_len == EOF)
	    {
	      fprintf(stderr,".");
	      free(recv_data);
	      return(0);
	    }
	  else
	    {
	      syslog(LOG_ERR,"\nRecv data error:");
	      free(recv_data);
	      return(-5);
	    }
	}
      else
	{
	  free(recv_data);
	  return(0);
	}
    }

  // Process incoming data
  error = parse_data(pLgMaster, recv_data, data_len, &parsed_count );
  if (error < 0)
    {
      syslog(LOG_ERR, "Bad parse, error %d",error);
      free(recv_data);
      return(error);
    }
  free(recv_data);
  return(data_len);
}

int SendConfirmation(struct lg_master *pLgMaster, unsigned char theCommand)
{
  unsigned char    gOutputBuffer[sizeof(struct send_cnfm)];
  unsigned char    gOutputHex[(sizeof(struct send_cnfm) * 2)];
  int i=0, count=0;
  int sent=0, remain=0;
  struct send_cnfm *pOut=(struct send_cnfm *)&gOutputBuffer[0];;

  sent = 0;
  // Need to initialize buffers to be used
  memset(gOutputBuffer, 0, sizeof(struct send_cnfm));
  memset(gOutputHex, 0, (sizeof(struct send_cnfm) * 2));

  pOut->cmd   = theCommand;
  pOut->flags = pLgMaster->gHeaderSpecialByte;
  pOut->seq_num = htons(pLgMaster->seqNo);
  AppendCRC((char *)gOutputBuffer, (sizeof(struct send_cnfm)-kCRCSize));

  if (pLgMaster->gHEX == 1 ) {
    count = 0; 
    for (i=0; i<COMM_CONFIRM_LEN; i++)
      {
	if (gOutputBuffer[i] >= 0x80)
	  {
	    gOutputHex[count] = 0x80;
	    count++;
	    gOutputHex[count] = gOutputBuffer[i] - 0x80;
	    count++;
	  }
	else
	  {
	    gOutputHex[count] = gOutputBuffer[i];
	    count++;
	  }
      }
    i = 0;
    remain = count;
    sent = 0;
    while(count && (i < count) && (sent >= 0) && (pLgMaster->datafd >= 0))
      {
	sent = send(pLgMaster->datafd, gOutputHex, count, MSG_NOSIGNAL);
	i += sent;
	remain -= sent;
      }
  }

  if (pLgMaster->gHEX == 0)
    {
      if ((pLgMaster->serial_ether_flag == 2) && (pLgMaster->datafd >= 0)) 
	sent = send(pLgMaster->datafd, gOutputBuffer, COMM_CONFIRM_LEN, MSG_NOSIGNAL);
      if (sent == -1)
	perror("\nBad data-send: ");
    }
  return(0);
}

void SendA3(struct lg_master *pLgMaster)
{
  struct k_header gOutputBuffer;
  int sent=0;

  memset ((char *)&gOutputBuffer, 0, sizeof(struct k_header));
  gOutputBuffer.status = 0xA3;

  if ((pLgMaster->serial_ether_flag == 2)  && (pLgMaster->datafd >= 0))
    sent = send(pLgMaster->datafd, (char *)&gOutputBuffer, sizeof(struct k_header), MSG_NOSIGNAL );
  if (sent <= 0)
    syslog(LOG_ERR,"\nUnable to send data, errno %d, sent %d", errno, sent);
  return;
}


void HandleResponse (struct lg_master *pLgMaster, int32_t lengthOfResponse, uint32_t gRespondToWhom )
{
  char     *gOutputHEX;
  int i, count, remain, sent;

  // FIXME--PAH--Here for compilation but should it be removed?
  // gRespondToWhom isn't used atm.
  gRespondToWhom = gRespondToWhom;
  gOutputHEX = malloc((lengthOfResponse * 2));
  if (!gOutputHEX)
    {
      perror("\nHANDLERESP: Bad Malloc():");
      return;
    }
  memset(gOutputHEX, 0, (lengthOfResponse * 2));
  
  AppendCRC((char *) pLgMaster->theResponseBuffer, lengthOfResponse);
  if (pLgMaster->gHEX == 1)
    {
      count = 0; 
      for (i=0; i<(lengthOfResponse+kCRCSize); i++)
	{
	  if (pLgMaster->theResponseBuffer[i] >= 0x80)
	    {
	      gOutputHEX[count] = 0x80;
	      count++;
	      gOutputHEX[count] = pLgMaster->theResponseBuffer[i] - 0x80;
	      count++;
	    }
	  else
	    {
	      gOutputHEX[count] = pLgMaster->theResponseBuffer[i];
	      count++;
	    }
	}
      i = 0;
      remain = count;
      sent = 0;
      if (pLgMaster->serial_ether_flag == 2)
	{
	  while ((i < count) && (sent >= 0) && (pLgMaster->datafd >= 0))
	    {
	      sent = send( pLgMaster->datafd, gOutputHEX, remain, MSG_NOSIGNAL );
	      i += sent;
	      remain -= sent;
	    }
	}
    }

  else if (pLgMaster->gHEX == 0)
    {
      i = 0;
      count = lengthOfResponse+2;
      remain = count;
      if (pLgMaster->serial_ether_flag == 2)
	{
	  while((i < count) && (pLgMaster->datafd >= 0))
	    {
	      sent = send(pLgMaster->datafd, pLgMaster->theResponseBuffer, remain, MSG_NOSIGNAL);
	      i += sent;
	      remain -= sent;
	    }
	}
    }
  free(gOutputHEX);
  return;
}

int
pingClient(char* peerAddr )
{
  char buffer[256];
  int return_code;
  uint32_t inaddr;

  // Make sure our PC webhost has been configured!
  if ((inaddr = inet_addr(peerAddr)) == INADDR_NONE)
    {
      perror("\nBad WebHost IP: ");
      return(-1);
    }
   
   memset(buffer, 0, sizeof(buffer));
#if 0
   char c[20];
   memset(c, 0, sizeof(c));

   sprintf( buffer, "/myping %s count 1 | grep loss | grep -c 100", peerAddr );
   FILE *p = popen( buffer, "r" );
   fgets( c,5,p );
   pclose(p);
   if ( strcmp( c,"0\n" ) == 0 )
     {
       return( 1 );
     }
   else
     {
       syslog(LOG_ERR, "NO Reply from %s", peerAddr );
       return( 0 );
     }
#else
   sprintf(buffer, "ping %s -c 1 > /dev/null 2>&1", peerAddr);
   return_code = system(buffer);
   if (return_code != 0) {
     syslog(LOG_ERR, "Ping to %s FAILED", peerAddr );
     return(-1);
   }
    
#endif
   return(0);
}

int CheckInput(struct lg_master *pLgMaster)
{
        int read2socks;
        struct timeval timeout;
        fd_set socks;

	// FIXME--PAH.  MAY NEED TO REVISIT FOR SERIAL COMM
	if (!pLgMaster || (pLgMaster->socketfd < 0))
	  return(-1);
        timeout.tv_sec  = 0;
        timeout.tv_usec = 0;
        FD_ZERO( &socks );
        if (pLgMaster->socketfd > 0)
	  FD_SET(pLgMaster->socketfd, &socks);
        if (pLgMaster->socketfd)
	  read2socks = select(pLgMaster->socketfd+1, &socks, NULL, NULL, &timeout);
        return( read2socks );
}
int DoProcEnetPackets(struct lg_master *pLgMaster)
{
  struct sockaddr_in client_addr;
  struct sockaddr_in haddr;
  socklen_t          sockaddr_len = sizeof(struct sockaddr_in);
  long               comm_loop_count=0;
  int                error=0;

  // Initialize buffers
  memset(&client_addr, 0, sockaddr_len);
  memset(&haddr, 0, sockaddr_len);

  // Accept connection with host
  pLgMaster->datafd = accept(pLgMaster->socketfd, (struct sockaddr *)&client_addr, (socklen_t *)&sockaddr_len);
  if (pLgMaster->datafd < 0)
    {
      syslog(LOG_ERR,"\nCOMMLOOP:  unable to accept data from %s", pLgMaster->webhost);
      return(-1);
    }
  // Get host's IP address
    error = getpeername(pLgMaster->datafd, (struct sockaddr *)&haddr, &sockaddr_len);
  if (error < 0)
    {
      syslog(LOG_ERR,"COMMCFGSCK: getpeername failed");
      return(-2);
    }
  strcpy(pLgMaster->webhost, inet_ntoa(haddr.sin_addr));

  syslog(LOG_NOTICE, "receiving data from %s, recvfd %x", pLgMaster->webhost, pLgMaster->datafd);
  while (pLgMaster->datafd >= 0)
    {
      error = ProcEnetPacketsFromHost(pLgMaster);
      if (error<=0)
	{
	  // Honor ping-heartbeat so partner knows we're alive
	  // FIXME--PAH.  NEED TO PUT PING INTO PTHREAD & JUST CHECK STATUS.
	  if ((pingClient(pLgMaster->webhost)) != 0)
	    pLgMaster->ping_count = 0;
	  else
	    pLgMaster->ping_count++;
	  return(0);
	}
      else
	{
	  comm_loop_count++;
	}
    }
  
  return(0);
}

