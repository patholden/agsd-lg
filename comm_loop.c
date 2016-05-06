#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#define _GNU_SOURCE   // Needed for polling conditions
#include <signal.h>
#include <poll.h>
#include <sys/io.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/ip_icmp.h>
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
#include "SystemSpecifics.h"

#define MAXPENDING  5   // Max # connection requests
#define AGS_PORT  1234
#define PACKETSIZE  64
#define POLL_SIZE 4
#define MAXTRIES 100


extern int
pingClient( char* peerAddr );

extern int
DoOnePing( char * hostip, uint8_t * buf, size_t bufsize );

uint16_t checksum(void *b, int len);

struct packet
{
    struct icmphdr hdr;
    char msg[PACKETSIZE-sizeof(struct icmphdr)];
};

int CommReinit(struct lg_master *pLgMaster)
{    
    int                sockaddr_len = sizeof(struct sockaddr_in);
    int                error;
    int                on = 1;

    syslog(LOG_NOTICE, "Re-initializing PC host comm connection");

    // Initialize buffers to 0
    memset(&pLgMaster->webhost_addr, 0, sockaddr_len);

    if (pLgMaster->datafd >=0)
      {
	close(pLgMaster->datafd);
	pLgMaster->datafd = -1;
      }
    if (pLgMaster->socketfd >=0)
      {
	close(pLgMaster->socketfd);
	pLgMaster->socketfd = -1;
      }
    sleep(3);
    // Set up comm interface with PC Host
    pLgMaster->socketfd = socket(PF_INET, SOCK_STREAM, IPPROTO_IP);
    if (pLgMaster->socketfd < 0)
      {
	syslog(LOG_ERR, "REINIT server socket failed");
	return(-1);
      }
    error = setsockopt(pLgMaster->socketfd, SOL_SOCKET, TCP_NODELAY, &on, sizeof(on));
    if (error < 0)
      {
	syslog(LOG_ERR,"COMMCFGSCK: REINIT setsockopt failed for socket %d",pLgMaster->socketfd);
	close(pLgMaster->socketfd);
	pLgMaster->socketfd = 0;
	return(-2);
      }
    pLgMaster->webhost_addr.sin_family = AF_INET;
    pLgMaster->webhost_addr.sin_port = htons(AGS_PORT);
    pLgMaster->webhost_addr.sin_addr.s_addr = INADDR_ANY;
    error = bind(pLgMaster->socketfd, (struct sockaddr *)&pLgMaster->webhost_addr, sockaddr_len);
    if (error < 0)
      {
	syslog(LOG_ERR, "COMMCFGSCK: REINIT bind failed");
	close(pLgMaster->socketfd);
	pLgMaster->socketfd = 0;
	return(-2);
      }
    error = listen(pLgMaster->socketfd, MAXPENDING);
    if (error < 0)
      {
	syslog(LOG_ERR, "COMMCFGSCK: REINIT connect failed ");
	close(pLgMaster->socketfd);
	pLgMaster->socketfd = 0;
	return(-3);
      }

    pLgMaster->enet_retry_count++;
    pLgMaster->serial_ether_flag = 2;
    pLgMaster->gResetComm = 0;
    syslog(LOG_ERR, "COMMCFGSCK: REINIT SUCCESSFUL");
    return(0);
}

int CommConfigSockfd(struct lg_master *pLgMaster)
{    
  int                sockaddr_len = sizeof(struct sockaddr_in);
  int                error;
  int                on = 1;
  
  // Initialize buffers to 0
  memset(&pLgMaster->webhost_addr, 0, sockaddr_len);

  // Just in case this is a re-config, check for open socketfd & close first
  if ((pLgMaster->socketfd >= 0) || (pLgMaster->datafd >= 0))
    {
      if (pLgMaster->datafd >= 0)
	close(pLgMaster->datafd);
      if (pLgMaster->socketfd >= 0)
	close(pLgMaster->socketfd);
      pLgMaster->socketfd = -1;
      pLgMaster->datafd = -1;
    }

  // Set up comm interface with PC Host
  pLgMaster->socketfd = socket(PF_INET, SOCK_STREAM, IPPROTO_IP);
  if (pLgMaster->socketfd < 0)
    {
      syslog(LOG_ERR, "INIT server socket: ");
      return(-1);
    }
  error = setsockopt(pLgMaster->socketfd, SOL_SOCKET, TCP_NODELAY, &on, sizeof(on));
  if (error < 0)
    {
      syslog(LOG_ERR,"COMMCFGSCK: INIT setsockopt failed for socket %d",pLgMaster->socketfd);
      return(-2);
    }
  pLgMaster->webhost_addr.sin_family = AF_INET;
  pLgMaster->webhost_addr.sin_port = htons(AGS_PORT);
  pLgMaster->webhost_addr.sin_addr.s_addr = INADDR_ANY;
  error = bind(pLgMaster->socketfd, (struct sockaddr *)&pLgMaster->webhost_addr, sockaddr_len);
  if (error < 0)
    {
      syslog(LOG_ERR, "COMMCFGSCK: INIT Bind failed ");
      return(-2);
    }
  error = listen(pLgMaster->socketfd, MAXPENDING);
  if (error < 0)
    {
      syslog(LOG_ERR, "COMMCFGSCK: INIT connect failed ");
      return(-3);
    }

  pLgMaster->enet_retry_count++;
  pLgMaster->serial_ether_flag = 2;
  return(0);
}

int CommInit(struct lg_master *pLgMaster)
{
  int            error=0;
  
  // Always check integrity of master struct
  if (!pLgMaster)
    {
      syslog(LOG_ERR, "Need master data configured!");
      return(-1);
    }

  // Try to open Comms interface to PC host
  if ((error = CommConfigSockfd(pLgMaster)) < 0)
    {
      syslog(LOG_ERR, "opensock");
      return(error);
    }
  syslog(LOG_NOTICE, "PC host comm ethernet port initialized");

  // Try to open front end PC serial port
  pLgMaster->pc_serial = open("/dev/lgttyS1", O_RDWR | O_NONBLOCK);
  if (pLgMaster->pc_serial <= 0)
    {
      syslog(LOG_ERR,"open PC front end serial port /dev/lgttyS1 failed");
      return(-5);
    }
  syslog(LOG_NOTICE, "PC host comm serial port lgttyS1 initialized");
  // Try to open AutoFocus serial port
  pLgMaster->af_serial = open("/dev/lgttyS2", O_RDWR | O_NONBLOCK);
  if (pLgMaster->af_serial <= 0)
    {
      syslog(LOG_ERR,"open Auto Focus serial port /dev/lgttyS2 failed");
      return(-5);
    }

  syslog(LOG_NOTICE, "AutoFocus port lgttyS2 initialized");
  // This sets the correct data presentation on send/recv from webhost
  pLgMaster->gHEX = 1;
  return(0);
}
static int ProcEnetPacketsFromHost(struct lg_master *pLgMaster)
{
    struct pollfd  poll_fd;
    unsigned char *recv_data = 0;
    unsigned char *parse_buff = 0;
    unsigned char *last_buff = 0;
    int       data_len = 0;
    int       total_count = 0;
    int       error = 0;
    uint32_t  parsed_count = 0;

    if (!pLgMaster)
      return(-1);

    if (pLgMaster->serial_ether_flag != 2)
      return(-2);

    memset((char *)&poll_fd, 0, sizeof(struct pollfd));
    poll_fd.fd = pLgMaster->datafd;
    poll_fd.events = POLLIN | POLLHUP;

    // error = poll((struct pollfd *)&poll_fd, 1, 10000);
    error = poll((struct pollfd *)&poll_fd, 1, 300);  // try old .3 seconds
    if (error < 1)
      return(0);
    if (poll_fd.revents == 0)
      return(0);

    last_buff = (unsigned char *)malloc(COMM_RECV_MAX_SIZE);
    if (!last_buff)
      return(-4);
    recv_data = (unsigned char *)malloc(COMM_RECV_MAX_SIZE);
    if (!recv_data)
      return(-3);

    // Prep buffers for accepting/receiving new packet
    memset(recv_data, 0, COMM_RECV_MAX_SIZE);
    memset(last_buff, 0, COMM_RECV_MAX_SIZE);
    for (;;)
      {
	// We've got our socket fd for client, now look for data
	parse_buff = (unsigned char *)(recv_data + total_count);
	data_len = recv(pLgMaster->datafd, parse_buff, COMM_RECV_MAX_SIZE, 0);
	if (data_len <= 0)
	  break;
	total_count+= data_len;
	if (!total_count || (total_count >= COMM_RECV_MAX_SIZE))
	  {
	    syslog(LOG_NOTICE, "Data not ready on receive, count %d, errno %d", total_count,errno);
	    break;
	  }
	error = poll((struct pollfd *)&poll_fd, 1, 3);
	if (error < 1)
	  break;
	if (!(poll_fd.revents & POLLIN))
	  break;
      }
    // Try one more time
    error = poll((struct pollfd *)&poll_fd, 1, 3);
    if (error == 1)
      {
	if ((poll_fd.revents & POLLIN))
	  {
	    // Try reading in one more buffer of data
	    data_len = recv(pLgMaster->datafd, last_buff, COMM_RECV_MAX_SIZE, 0);
	    if (data_len > 0)
	      {
		parse_buff = (unsigned char *)(recv_data + total_count);
		memcpy(parse_buff, last_buff, data_len);
		total_count+= data_len;
	      }
	  }
      }

    if (total_count == 0)
      {
	free(last_buff);
	free(recv_data);
	return(-1);
      }
    // Process incoming data
    error = parse_data(pLgMaster, recv_data, total_count, &parsed_count);
    if (error < 0)
      {
	syslog(LOG_ERR, "Bad parse, error %d",error);
	free(recv_data);
	free(last_buff);
	return(error);
      }
    free(last_buff);
    free(recv_data);
    return(0);
}
static int IsOkToSend(struct lg_master *pLgMaster)
{
    struct pollfd  poll_fd;

    memset((char *)&poll_fd, 0, sizeof(struct pollfd));
    poll_fd.fd = pLgMaster->datafd;
    poll_fd.events = POLLOUT;

    if (poll_fd.revents | POLLOUT)
      return(0);
    return(-1);
}
void SendConfirmation(struct lg_master *pLgMaster, unsigned char theCommand)
{
    unsigned char    gOutputBuffer[sizeof(struct send_cnfm)];
    unsigned char    gOutputHex[(sizeof(struct send_cnfm) * 2)];
    int              i, count, sent;
    struct           send_cnfm *pOut=(struct send_cnfm *)&gOutputBuffer[0];;

    if (IsOkToSend(pLgMaster))
      return;
  
    // Need to initialize buffers to be used
    memset(gOutputBuffer, 0, sizeof(struct send_cnfm));
    memset(gOutputHex, 0, (sizeof(struct send_cnfm) * 2));

    pOut->cmd   = theCommand;
    pOut->flags = pLgMaster->gHeaderSpecialByte;
    pOut->seq_num = htons(pLgMaster->seqNo);
    AppendCRC((unsigned char *)gOutputBuffer, (sizeof(struct send_cnfm)-kCRCSize));

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
      sent = send(pLgMaster->datafd, gOutputHex, count, MSG_NOSIGNAL);
      if (sent <=0)
	syslog(LOG_ERR,"bad send on confirmation: errno %d", errno);
    }

    if (pLgMaster->gHEX == 0)
      {
	if ((pLgMaster->serial_ether_flag == 2) && (pLgMaster->datafd >= 0)) 
	  sent = send(pLgMaster->datafd, gOutputBuffer, COMM_CONFIRM_LEN, MSG_NOSIGNAL);
	if (sent <=0)
	  syslog(LOG_ERR,"Bad send on confirmation: errno %d", errno);
      }
    return;
}

void SendA3(struct lg_master *pLgMaster)
{
    struct k_header gOutputBuffer;
    int sent=0;

    if (IsOkToSend(pLgMaster))
      return;
  
    memset ((char *)&gOutputBuffer, 0, sizeof(struct k_header));
    gOutputBuffer.status = 0xA3;

    if ((pLgMaster->serial_ether_flag == 2)  && (pLgMaster->datafd >= 0))
      sent = send(pLgMaster->datafd, (char *)&gOutputBuffer, sizeof(struct k_header), MSG_NOSIGNAL );
      syslog(LOG_ERR,"Sending A3: errno %d", errno);
    if (sent <= 0)
      syslog(LOG_ERR,"Bad send on A3: errno %d", errno);
    return;
}


void HandleResponse (struct lg_master *pLgMaster, int32_t lengthOfResponse, uint32_t gRespondToWhom )
{
    unsigned char  *gOutputHEX;
    int            i, count, remain, sent;

    if (IsOkToSend(pLgMaster))
      return;
  
    // FIXME--PAH--Here for compilation but should it be removed?
    // gRespondToWhom isn't used atm.
    gRespondToWhom = gRespondToWhom;
    gOutputHEX = malloc((lengthOfResponse * 2));
    if (!gOutputHEX)
      {
	syslog(LOG_ERR, "HANDLERESP: Bad Malloc():");
	return;
      }
    memset(gOutputHEX, 0, (lengthOfResponse * 2));
  
    AppendCRC(pLgMaster->theResponseBuffer, lengthOfResponse);
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
	remain = count;
	if (pLgMaster->serial_ether_flag == 2)
	  {
	    while (remain > 0)
	      {
		sent = send( pLgMaster->datafd, gOutputHEX, remain, MSG_NOSIGNAL );
		if (sent <= 0)
		  {
		    syslog(LOG_ERR,"Bad send on Response: errno %d", errno);
		    if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
		      break;
		  }
		remain -= sent;
	      }
	  }
      }

    else if (pLgMaster->gHEX == 0)
      {
	remain = lengthOfResponse+2;
	if (pLgMaster->serial_ether_flag == 2)
	  {
	    while(remain > 0)
	      {
		sent = send(pLgMaster->datafd, pLgMaster->theResponseBuffer, remain, MSG_NOSIGNAL);
		if (sent <= 0)
		  {
		    syslog(LOG_ERR,"Bad send on Response: errno %d", errno);
		    if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
		      break;
		  }
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
  uint8_t buffer[256];
  // int return_code;
  uint32_t inaddr;
  int  bytes;

  // Make sure our PC webhost has been configured!
  if ((inaddr = inet_addr(peerAddr)) == INADDR_NONE)
    {
      syslog(LOG_ERR, "Bad WebHost IP: ");
      // ***debug*** return(-1);
    }
   
   bytes = DoOnePing( peerAddr, buffer, sizeof(buffer) ); 
   if (bytes <= 0) {
     syslog(LOG_ERR, "Ping to %s FAILED return %d", peerAddr, bytes );
     return(-1);
   }
   return(0);
}


int DoProcEnetPackets(struct lg_master *pLgMaster)
{
    struct sockaddr_in client_addr;
    struct sockaddr_in haddr;
    socklen_t          sockaddr_len = sizeof(struct sockaddr_in);
    int                error=0;
    int                on=1;

    // Initialize buffers
    memset(&client_addr, 0, sockaddr_len);
    memset(&haddr, 0, sockaddr_len);

    // Accept connection with host
    if (pLgMaster->datafd >= 0)
      close(pLgMaster->datafd);
    pLgMaster->datafd = accept(pLgMaster->socketfd, (struct sockaddr *)&client_addr, (socklen_t *)&sockaddr_len);
    if (pLgMaster->datafd < 0)
      {
	syslog(LOG_ERR,"COMMLOOP:  unable to accept data from %s", pLgMaster->webhost);
	return(-1);
      }
    error = setsockopt(pLgMaster->datafd, SOL_SOCKET, TCP_NODELAY, &on, sizeof(on));
    if (error < 0)
      {
	syslog(LOG_ERR,"COMMCFGSCK: setsockopt failed for socket %d",pLgMaster->socketfd);
	return(-2);
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
    doSetLinkLED(pLgMaster);
    while (pLgMaster->datafd >= 0)
      {
	error = ProcEnetPacketsFromHost(pLgMaster);
	if (error < 0)
	  return(error);
	else if (pLgMaster->gResetComm)
	  {
	    if ((CommReinit(pLgMaster)) < 0)
	      {
		  // It takes a while for eth0 to free up bound socket.
		  // Try to open Comms interface to PC host once a second
		  sleep(3);
		  return(-1);
	      }
	    // Return to caller to decide what to do
	  }
	else
	  {
	    // Honor ping-heartbeat so partner knows we're alive
	    // FIXME--PAH.  NEED TO PUT PING INTO PTHREAD & JUST CHECK STATUS.
	    if ((pingClient(pLgMaster->webhost)) == 0)
	      {
		pLgMaster->ping_count = 0;
	      }
	    else
             {
	        pLgMaster->ping_count++;
	     }
             // wait for three failed pings
             if ( pLgMaster->ping_count >= 3 ) 
		return(-1);
	    
	  }
	DoSystemPeriodic(pLgMaster);
      }
    // Should never get here
    return(0);
}


int
DoOnePing( char * hostip, uint8_t * buf, size_t bufsize )
{
/*
 *  DoOnePing just sends one ICMP ping packet
 *  and tried to receive any return message
 */

        const int            val  =   2;
        int                  i, sd;
        struct packet        pckt;
        socklen_t            len;
        struct sockaddr_in   addr;
        struct protoent      *protocol;
        int                  bytes = 0;
        in_addr_t            inaddr;

        int try;
        struct pollfd poll_set[POLL_SIZE];
        int numfds = 0;
        int status;

        inaddr = inet_addr( hostip );

        len   =  sizeof(struct sockaddr_in);

        addr.sin_family        =  AF_INET;
        addr.sin_port          =  0;

        memcpy( (void *)&(addr.sin_addr.s_addr)
              , (void *)&inaddr
              , sizeof(inaddr));


        protocol = getprotobyname("ICMP");

        // open a socket for an ICMP ping packet
        sd    =  socket(PF_INET, SOCK_RAW, protocol->p_proto);
        if (sd < 0)
	  return(-1);

        // set Time-To-Live
        if (setsockopt(sd, SOL_IP, IP_TTL, &val, sizeof(val)) != 0)
        {
                close( sd );
                return(-1);
        }

        // set socket status flag
        if (fcntl(sd, F_SETFL, O_NONBLOCK) != 0)
        {
                close( sd );
                return(-1);
        }

        // assemble ICMP ping packet
        //
        memset( (void *)(&pckt), 0, sizeof(pckt) );
        pckt.hdr.type               =  ICMP_ECHO;
        pckt.hdr.un.echo.id         =  321;  // should be set to something better
        for (i = 0; i < sizeof(pckt.msg)-1; i++)
                        pckt.msg[i] = i+'0';
        pckt.msg[i]                 =  0;
        pckt.hdr.un.echo.sequence   =  1;
        pckt.hdr.checksum           =  checksum(&pckt, sizeof(pckt));

        // send off ICMP ping packet
        //
        if (sendto(sd, &pckt, sizeof(pckt), 0, (struct sockaddr*)&addr, sizeof(addr)) <= 0) {
                close( sd );
                return(-1);
        }


        len  =  sizeof(struct sockaddr_in);

        memset( buf, 0, bufsize );


        // set up a poll for response
        numfds = 0;
        poll_set[0].fd = sd;
        poll_set[0].events = POLLIN;
        numfds++;
         
        try = 0;
        bytes = 0;
        
        while ( try < MAXTRIES && bytes == 0 ) {
            try++;
            status = poll( poll_set, numfds, 2 );
            if ( (poll_set[0].revents & POLLIN) && status >= 1 ) {
               usleep(1000);
               bytes = recvfrom(sd, buf, bufsize, 0, (struct sockaddr*)&addr, &len);
            }
       }

       close( sd );
       return ( bytes );
}


uint16_t checksum(void *b, int len)
{
        uint16_t   *buf = b;
        uint32_t     sum=0;

        for(sum=0; len>1; len-=2)
        {
                sum += *buf++;
        }
        if (len == 1)
        {
                sum += *(uint8_t*)buf;
        }
        sum     =  (sum >> 16) + (sum & 0xFFFF);
        sum     += (sum >> 16);
        return(~sum);
}

