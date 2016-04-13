#include <stdint.h>
#include <stdlib.h>
#include <syslog.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <ctype.h>
#include <string.h>
#include <linux/laser_api.h>



#include <unistd.h>
#include <netdb.h>

#include <stdio.h>
#include <sys/time.h>

#include "BoardComm.h"
#include "AppCommon.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "LaserInterface.h"
#include "3DTransform.h"
#include "Protocol.h"
#include "CRCHandler.h"
#include "DoAutoFocusCmd.h"

#include "RemoteSerial.h"

struct sockaddr_in     tcp_srv_addr;
struct servent         tcp_serv_info;
struct hostent         tcp_host_info;

#define MAXLINE 1024
#define BUFFSIZE  (2048+3*512*512)

static char inbuff[BUFFSIZE];
static char hexbuff[BUFFSIZE];
static char outbuff[BUFFSIZE];


// char  visionhost[128] = "10.1.3.1";

void RemoteSerial ( struct lg_master *pLgMaster
                  , char * buffer
                  , uint32_t respondToWhom
                  )
{

        int index, i;
        char * tmpPtr;
        int             fd;
        uint32_t   inaddr;
        int             port = 1234;
        int             n, count, count0;
        char            linebuff[MAXLINE];
        int length;
        int incount;
        int total;
        fd_set socks;
        int readsocks;
        int highsock = 0;
        struct timeval timeout;
        int hexcount;
        struct parse_autofocus_resp *pResp = (struct parse_autofocus_resp *)pLgMaster->theResponseBuffer;



#ifdef AGS_DEBUG
     for ( i = 0; i < 32 ; i++ ) {
          if ( isalnum( buffer[i] ) ) {
          syslog(LOG_DEBUG, "remoteserial %2d %02x   %c ", i, 0xff&buffer[i], buffer[i] );
          } else {
          syslog( LOG_DEBUG, "remoteserial %2d %02x ", i, 0xff&buffer[i] );
          }
     }
#endif



        index  = 0;
        tmpPtr = &(buffer[index]);

  

        bzero((char *) &tcp_srv_addr, sizeof(tcp_srv_addr));
        tcp_srv_addr.sin_family = AF_INET; 

        tcp_srv_addr.sin_port = htons(port);

#ifdef AGS_DEBUG
syslog(LOG_DEBUG, "about to inet_addr" );
#endif

        if( (inaddr = inet_addr(pLgMaster->visionhost)) != INADDR_NONE) {
            bcopy((char *) &inaddr, (char *) &tcp_srv_addr.sin_addr, sizeof(inaddr));
            tcp_host_info.h_name = NULL;
        } else {
                pResp->hdr.status = RESPFAIL;
                HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );

                return;
        }

#ifdef AGS_DEBUG
syslog(LOG_DEBUG, "about socket" );
#endif

  if( (fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
                syslog( LOG_ERR, "can't create TCP socket\n");
                close (fd);

                pResp->hdr.status = RESPFAIL;
                HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );

                return;
  }

#ifdef AGS_DEBUG
syslog(LOG_DEBUG, "about connect" );
#endif
   
        if( connect(fd,(struct sockaddr *)&tcp_srv_addr,sizeof(tcp_srv_addr)) < 0 ) {
                syslog( LOG_NOTICE, "can't connect to server\n");
                close (fd);
                pResp->hdr.status = RESPFAIL;
                HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );

                return;
  }

  memset( inbuff, 0, BUFFSIZE );
  memset( hexbuff, 0, BUFFSIZE );

        inbuff[0] = 0x64;
        inbuff[1] = 0xff & pLgMaster->gHeaderSpecialByte;
        index  = sizeof(int32_t);
        tmpPtr = &(inbuff[index]);

        for( i=0; i<32; i++ ) {
                tmpPtr[i] = buffer[i];
        }

        length = sizeof(int32_t) + 32;
        AppendCRC( (unsigned char *)inbuff, length );

        incount = length + kCRCSize;
        hexcount = 0;
        for ( i=0; i<incount; i++ ) {
            if ( (unsigned char)(inbuff[i]) >= 0x80 ) {
               hexbuff[hexcount] = 0x80;
               hexcount++;
               hexbuff[hexcount] = inbuff[i] - 0x80;
               hexcount++;
            } else {
               hexbuff[hexcount] = inbuff[i];
               hexcount++;
            }
        }
        for ( i=0; i<hexcount; i++ ) {
#ifdef AGS_DEBUG
syslog( LOG_DEBUG, "about write %x  hex %d", 0xff&hexbuff[i],i );
#endif
        }


        total = 0;
        n = 1;
        while ( total < hexcount && n > 0 ) {
#ifdef AGS_DEBUG
syslog( LOG_NOTICE, "about write %d  total %d", hexcount, total );
#endif
            n = write( fd, &(hexbuff[total]), (hexcount-total) );
            if ( n < 0 ) {
                close( fd );

                pResp->hdr.status = RESPFAIL;
                HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );

                return;
            }
            total += n;
        }
        count = 0;
#ifdef AGS_DEBUG
syslog( LOG_DEBUG, "about read" );
#endif
        FD_ZERO( &socks );
        FD_SET( fd, &socks );
        highsock = fd;
        timeout.tv_sec  = 1;
        timeout.tv_usec = 200000;

        readsocks = select( highsock+1
                          , &socks
                          , (fd_set *)0
                          , (fd_set *)0
                          , &timeout
                          );

#ifdef AGS_DEBUG
syslog( LOG_DEBUG, "gHeaderSpecialByte  %02x", 0xff & pLgMaster->gHeaderSpecialByte );
#endif
        memset( linebuff, 0, MAXLINE );
        memset( hexbuff, 0, MAXLINE );
        if ( pLgMaster->gHeaderSpecialByte & 0x02 ) {
          usleep( 200000 );

          count = 0;
          memset( linebuff, 0, MAXLINE );
               // try first read, which may be just a response
          if ( readsocks > 0 && FD_ISSET( fd, &socks ) ) {
#ifdef AGS_DEBUG
syslog( LOG_DEBUG, "about readsocks %d", readsocks );
#endif
            n = read(fd,linebuff,MAXLINE);
            memcpy( &(outbuff[count]), linebuff, n );
            count += n;
#ifdef AGS_DEBUG
syslog( LOG_DEBUG, "1st read %d bytes  count %d", n, count );
#endif
          }

          if ( n <= 20 ) {
              FD_ZERO( &socks );
              FD_SET( fd, &socks );
              highsock = fd;
              timeout.tv_sec  = 1;
              timeout.tv_usec = 200000;

              readsocks = select( highsock+1
                                , &socks
                                , (fd_set *)0
                                , (fd_set *)0
                                , &timeout
                                );
              if ( readsocks > 0 && FD_ISSET( fd, &socks ) ) {
                  memset( linebuff, 0, MAXLINE );
                  // too few bytes received
                  usleep( 200000 );
                  n = read(fd,linebuff,MAXLINE);
                  memcpy( &(outbuff[count]), linebuff, n );
                  count += n;
              }
#ifdef AGS_DEBUG
syslog( LOG_DEBUG, "2nd read %d bytes   count %d", n, count );
#endif
          }



          count0 = 0;
          for ( i = 0; i < count; i++ ) {
#ifdef AGS_DEBUG
if ( isalnum( outbuff[i] ) ) {
syslog( LOG_DEBUG, "hexread %3d %2x  %c ", i, 0xff & outbuff[i], outbuff[i] );
} else {
syslog( LOG_DEBUG, "hexread %3d %2x  ", i, 0xff & outbuff[i] );
}
#endif
             if ( (unsigned char)(outbuff[i]) == 0x80 ) {
                  hexbuff[count0] = outbuff[i+1] + 0x80;
                  i++;
#ifdef AGS_DEBUG
if ( isalnum( outbuff[i] ) ) {
syslog( LOG_DEBUG, "hexread %3d %2x  %c ", i, 0xff & outbuff[i], outbuff[i] );
} else {
syslog( LOG_DEBUG, "hexread %3d %2x  ", i, 0xff & outbuff[i] );
}
#endif
             } else {
                  hexbuff[count0] = outbuff[i];
             }
             count0++;
          } 

#ifdef AGS_DEBUG
          for ( i = 0; i < count0; i++ ) {
if ( isalnum( hexbuff[i] ) ) {
syslog( LOG_DEBUG, "unhexread %3d %2x  %c ", i, 0xff & hexbuff[i], hexbuff[i] );
} else {
syslog( LOG_DEBUG, "unhexread %3d %2x  ", i, 0xff & hexbuff[i] );
}
          } 
#endif
            // skip over the six byte confirmation in hexbuff
            // and copy over just 32 bytes
          memcpy( &(outbuff[0]), &(hexbuff[6]), 32 );

#ifdef AGS_DEBUG
syslog( LOG_DEBUG, "about close  read %d   count %d", n, count );
for ( i = 0; i < 40; i++ ) {
syslog( LOG_DEBUG, "outbuff %3d %2x", i, 0xff & outbuff[i] );
}
#endif
        }

        close (fd);


       if ( pLgMaster->gHeaderSpecialByte & 0x02 ) {

                pResp->hdr.status  = kAutoFocusCmd;
                            // skip over header
                memcpy( &(pLgMaster->theResponseBuffer[4]), &(outbuff[4]), 32 );

#ifdef AGS_DEBUG
syslog( LOG_DEBUG, "about close  read %d   count %d", n, count );
for ( i = 0; i < 40; i++ ) {
syslog( LOG_DEBUG, "response %3d %2x", i, 0xff & pLgMaster->theResponseBuffer[i] );
}
#endif

                HandleResponse( pLgMaster
                              , (sizeof(struct parse_autofocus_resp) - kCRCSize)
                              , respondToWhom
                              );

       }
       return;

       //  PostCommand( kDarkAngle, (char *)data, kRespondExtern );


}
