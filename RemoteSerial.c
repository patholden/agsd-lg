#include <stdint.h>
#include <stdlib.h>
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
#include "DoTakePicture.h"

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
                  )
{

        int index, i;
        char * tmpPtr;
        double partPoint[3];
        int             fd;
        uint32_t   inaddr;
        int             port = 1234;
        int             n, count;
        char            linebuff[MAXLINE];
        int length;
        int incount;
        int total;
        fd_set socks;
        int readsocks;
        int highsock = 0;
        struct timeval timeout;
        int hexcount;


#ifdef ZDEBUG
     for ( i = 0; i < 32 ; i++ ) {
          fprintf( stderr, "remoteserial %2d %02x ", i, 0xff&buffer[i] );
          if ( isalnum( buffer[i] ) )
             fprintf( stderr, " %c ",  buffer[i] );
          fprintf( stderr, "\n" );
     }
#endif



        index  = 0;
        tmpPtr = &(buffer[index]);

  

  bzero((char *) &tcp_srv_addr, sizeof(tcp_srv_addr));
  tcp_srv_addr.sin_family = AF_INET; 

  tcp_srv_addr.sin_port = htons(port);

#ifdef ZDEBUG
fprintf(stderr, "about to inet_addr\n" );
#endif

  if( (inaddr = inet_addr(pLgMaster->visionhost)) != INADDR_NONE) {
      bcopy((char *) &inaddr, (char *) &tcp_srv_addr.sin_addr, sizeof(inaddr));
      tcp_host_info.h_name = NULL;
  } else {
                *(uint32_t *)(pLgMaster->theResponseBuffer) =
                        kFail;
                HandleResponse ( pLgMaster,
                        sizeof ( uint32_t ), kRespondExtern );
                return;
  }

#ifdef ZDEBUG
fprintf(stderr, "about socket\n" );
#endif

  if( (fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
      fprintf( stderr, "can't create TCP socket\n");
      close (fd);
                *(uint32_t *)(pLgMaster->theResponseBuffer) =
                        kFail;
                HandleResponse ( pLgMaster,
                        sizeof ( uint32_t ), kRespondExtern );
                return;
  }

#ifdef ZDEBUG
fprintf(stderr, "about connect\n" );
#endif
   
        if( connect(fd,(struct sockaddr *)&tcp_srv_addr,sizeof(tcp_srv_addr)) < 0 ) {
      fprintf( stderr, "can't connect to server\n");
      close (fd);
                *(uint32_t *)(pLgMaster->theResponseBuffer) =
                        kFail;
                HandleResponse ( pLgMaster,
                        sizeof ( uint32_t ), kRespondExtern );
                return;
  }

  memset( inbuff, 0, BUFFSIZE );
  memset( hexbuff, 0, BUFFSIZE );

        inbuff[0] = 0x64;
        index  = sizeof(int32_t);
        tmpPtr = &(inbuff[index]);

        for( i=0; i<32; i++ ) {
                tmpPtr[i] = buffer[i];
        }
        *(double *)(&(tmpPtr[16])) = partPoint[2];

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
#ifdef ZDEBUG
fprintf(stderr, "about write %x  hex %d\n", 0xff&hexbuff[i],i );
#endif
        }


        total = 0;
        n = 1;
        while ( total < hexcount && n > 0 ) {
#ifdef ZDEBUG
fprintf(stderr, "about write %d  total %d\n", hexcount, total );
#endif
            n = write( fd, &(hexbuff[total]), (hexcount-total) );
            if ( n < 0 ) {
                close( fd );
                *(uint32_t *)(pLgMaster->theResponseBuffer) =
                        kFail;
                HandleResponse ( pLgMaster,
                        sizeof ( uint32_t ), kRespondExtern );
                return;
            }
            total += n;
        }
        count = 0;
#ifdef ZDEBUG
fprintf(stderr, "about read \n" );
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

#ifdef ZDEBUG
fprintf(stderr, "gHeaderSpecialByte  %02x \n", 0xff & pLgMaster->gHeaderSpecialByte );
#endif
        memset( linebuff, 0, MAXLINE );
        memset( hexbuff, 0, MAXLINE );
        if ( pLgMaster->gHeaderSpecialByte & 0x02 ) {
          usleep( 200000 );


          if ( readsocks > 0 && FD_ISSET( fd, &socks ) ) {
#ifdef ZDEBUG
fprintf(stderr, "about readsocks %d \n", readsocks );
#endif
            n = read(fd,linebuff,MAXLINE);
            // strncpy( &(outbuff[count]), linebuff, n );
            count += n;
          }
          count = 0;
          for ( i = 0; i < n; i++ ) {
#ifdef ZDEBUG
fprintf(stderr, "hexread %3d %2x  ", i, 0xff & linebuff[i] );
if ( isalnum( linebuff[i] ) )
             fprintf( stderr, " %c ",  linebuff[i] );
fprintf(stderr, "\n" );
#endif
             if ( (unsigned char)(linebuff[i]) == 0x80 ) {
                  hexbuff[count] = linebuff[i+1] + 0x80;
                  i++;
#ifdef ZDEBUG
fprintf(stderr, "hexread %3d %2x  ", i, 0xff & linebuff[i] );
if ( isalnum( linebuff[i] ) )
             fprintf( stderr, " %c ",  linebuff[i] );
fprintf(stderr, "\n" );
#endif
             } else {
                  hexbuff[count] = linebuff[i];
             }
             count++;
          } 

#ifdef ZDEBUG
          for ( i = 0; i < count; i++ ) {
fprintf(stderr, "unhexread %3d %2x  ", i, 0xff & hexbuff[i] );
          if ( isalnum( hexbuff[i] ) )
             fprintf( stderr, " %c ",  hexbuff[i] );
fprintf(stderr, "\n" );
          } 
#endif
            // skip over the six byte confirmation in hexbuff
            // and copy over just 32 bytes
          memcpy( &(outbuff[0]), &(hexbuff[6]), 32 );

#ifdef ZDEBUG
fprintf(stderr, "about close  read %d   count %d\n", n, count );
for ( i = 0; i < 40; i++ ) {
fprintf(stderr, "outbuff %3d %2x  ", i, 0xff & outbuff[i] );
fprintf(stderr, "\n" );
}
#endif
        }

        close (fd);


       if ( pLgMaster->gHeaderSpecialByte & 0x02 ) {

                *(uint32_t *)(pLgMaster->theResponseBuffer) =
                        kAutoFocusCmd;
// skip over header
          memcpy( &(pLgMaster->theResponseBuffer[4]), &(outbuff[4]), 32 );
#ifdef ZDEBUG
fprintf(stderr, "about close  read %d   count %d\n", n, count );
for ( i = 0; i < 40; i++ ) {
fprintf(stderr, "response %3d %2x  ", i, 0xff & theResponseBuffer[i] );
fprintf(stderr, "\n" );
}
#endif
          HandleResponse ( pLgMaster,
                        sizeof ( uint32_t ) + 32, kRespondExtern );
       }
       return;

       //  PostCommand( kDarkAngle, (char *)data, kRespondExtern );


}
