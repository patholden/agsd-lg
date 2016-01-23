#include <stdint.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>

#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "Protocol.h"
#include "Hobbs.h"
#include "CRCHandler.h"

time_t start_display;
time_t end_display;
time_t hobbs_counter;
int hobbs_state;

static int HobbsCountersUpdate(FILE *filefd, struct lg_master *pLgMaster)
{
  char   hobbsBuffer[HOBBS_MAX_BUFFLEN];
  size_t num_write;
  size_t bytes_to_write;

  memset(hobbsBuffer, 0, sizeof(hobbsBuffer));
  if (!filefd || !pLgMaster)
    return(-1);
  // Get Hobbs counters and put into buffer
  sprintf(hobbsBuffer, " %10ld \r\n %10ld \r\n %10ld \r\n %10ld \r\n",
	  pLgMaster->hobbs.hobbs_counter,
	  pLgMaster->hobbs.xscanner_counter,
	  pLgMaster->hobbs.yscanner_counter,
	  pLgMaster->hobbs.laser_counter);
  bytes_to_write = strlen(hobbsBuffer);
  if (bytes_to_write <= 0)
    return(-2);
  
  // Write counters out to file
  num_write = fwrite(hobbsBuffer, bytes_to_write, 1, filefd);
  if (num_write <= 0)
    return(-3);
  return(0);
}

int HobbsCountersInit(struct lg_master *pLgMaster)
{
  FILE  *filefd;
  long   file_size;
  
  filefd = fopen("/etc/ags/conf/newhobbs", "w+");
  if (!filefd)
    return(-1);

  fseek(filefd, 0, SEEK_SET);
  file_size = ftell(filefd);
  // If zero-length then new file
  if (!file_size)
    HobbsCountersUpdate(filefd, pLgMaster);
      
  return(0);
}
static int HobbsToFS(struct lg_master *pLgMaster)
{
     char CmdBuff[1024];
     int err;


     if (!pLgMaster)
       return(-1);
#if 0
     sprintf( CmdBuff, "cp /etc/ags/conf/hobbs /xscanner /yscanner /laser /etc/ags/conf" );
#else
     sprintf( CmdBuff, "cp /etc/ags/conf/hobbs /etc/ags/conf" );
#endif
     err = system(  CmdBuff );
     return(err);
}

void StartHobbs(struct lg_master *pLgMaster)
{
     end_display = 0;
     start_display = time( (time_t *)NULL );    
#ifdef SDEBUG
     fprintf( stderr
            , " StartHobbs start %d  end %d\r\n"
            , start_display
            , end_display
            );
#endif
}


void EndHobbs(struct lg_master *pLgMaster)
{
  time_t delta_time;

  end_display = time( (time_t *)NULL );
  if ( start_display > 0 ) {
    delta_time = end_display - start_display;
    // only count display intervals int32_ter that one second
    if ( delta_time > 1 ) {
      pLgMaster->hobbs.hobbs_counter += delta_time;
      pLgMaster->hobbs.xscanner_counter += delta_time;
      pLgMaster->hobbs.yscanner_counter += delta_time;
      pLgMaster->hobbs.laser_counter += delta_time;
    }
  }
  // FIXME---PAH---NEEDS TO BE REWRITTEN
  //  HobbsToFS(pLgMaster);  // write hobbs meters to flash
  start_display = 0;
  return;
}

  // just work from RAM copy
time_t ReadHobbs( char * HobbsName )
{
     char FromRAM[1024];
     char NameBuff[1024];
     time_t hobbs_time;
     int filenum;
     long length;

     // FIXME---PAH---NEED TO FIX ALL THE HOBBS COUNT
     return(0);
#ifdef OBFdebug
     fprintf( stderr, "OFBReadHobbs\n" );
#endif
     
     memset( (void *)FromRAM, 0, 1024 );

     sprintf( NameBuff, "/etc/ags/conf/%s", HobbsName ); 
     filenum = open( NameBuff , O_RDONLY );
     if ( filenum <= 0 ) {
          hobbs_time = 0;
          return( hobbs_time );
     }
     length = read( filenum, FromRAM, 1024 );
     close( filenum );
     if (length <=0)
       return(-1);
     if ( kCRC_OK == CheckCRC( (char *)FromRAM, 14 ) ) {
          FromRAM[14] = 0;
          FromRAM[15] = 0;
          sscanf( FromRAM, "%d", (int *)&hobbs_time );
     } else {
          hobbs_time = 0;
     }
#ifdef SDEBUG
     fprintf( stderr, " ReadHobbs counter %10d \r\n", hobbs_time );
#endif

     return( hobbs_time );
}

void DoHobbsSet(struct lg_master *pLgMaster, char * data, uint32_t respondToWhom )
{
  struct parse_hobbsset_parms *pInp = (struct parse_hobbsset_parms *)data;
  struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
  unsigned int hobbsIndex;
  unsigned int hobbsValue;

  hobbsIndex = pInp->inp_hobbsid;
  hobbsValue = pInp->inp_hobbsval;

  pResp->hdr.status = RESPGOOD;
  switch ( hobbsIndex )
    {
    case 1:
      WriteHobbs( hobbsValue, "hobbs" );
      break;
    case 2:
      WriteHobbs( hobbsValue, "xscanner" );
      break;
    case 3:
      WriteHobbs( hobbsValue, "yscanner" );
      break;
    case 4:
      WriteHobbs( hobbsValue, "laser" );
      break;
    default:
      pResp->hdr.status = RESPFAIL;
      break;
    }
  // FIXME---PAH---need to rewrite HOBBS counters
  HobbsToFS(pLgMaster);  //  write the hobbs meters to flash
  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
  return;
}

void DoHobbsGet( struct lg_master *pLgMaster, char * data, uint32_t respondToWhom )
{
  struct parse_hobbsget_parms *pInp = (struct parse_hobbsget_parms *)data;
  struct parse_hobbsget_resp *pResp=(struct parse_hobbsget_resp *)pLgMaster->theResponseBuffer;
  unsigned int hobbs_counter;
  unsigned int hobbsIndex;
  uint32_t theReturn=kOK;

  hobbsIndex = pInp->inp_hobbsid;
  
  switch ( hobbsIndex )
    {
    case 1:
      hobbs_counter = ReadHobbs("hobbs");
      break;
    case 2:
      hobbs_counter = ReadHobbs("xscanner");
      break;
    case 3:
      hobbs_counter = ReadHobbs("yscanner");
      break;
    case 4:
      hobbs_counter = ReadHobbs("laser");
      break;
    default:
      theReturn = kFail;
      break;
    }

  if ( theReturn == kOK ) {
    pResp->hdr.status = RESPGOOD;
    pResp->resp_hobbscount = hobbs_counter;
    
    HandleResponse(pLgMaster, (sizeof(struct parse_hobbsget_resp)-kCRCSize), respondToWhom);
  } else {
    pResp->hdr.status = RESPFAIL;
    HandleResponse(pLgMaster, (sizeof(struct parse_hobbsget_resp)-kCRCSize), respondToWhom);
  }
  return;
}

void WriteHobbs( time_t counter, char * HobbsName )
{
     char ToRAM[1024];
     char CmdBuff[1024];
     char NameBuff[1024];
     int filenum;
     
     memset( (void *)ToRAM, 0, 1024 );
     sprintf( CmdBuff, "touch /etc/ags/conf/%s", HobbsName ); 
     system( CmdBuff );
     memset( (void *)CmdBuff, 0, 1024 );

#ifdef SDEBUG
     fprintf( stderr, " WriteHobbs counter %10d \r\n", counter );
#endif

     sprintf( ToRAM, " %10d \r\n", (int)counter );
     AppendCRC( ToRAM, 14 );

     sprintf( NameBuff, "/etc/ags/conf/%s", HobbsName ); 
     filenum = open( NameBuff, O_WRONLY|O_TRUNC );
     if ( filenum <= 0 ) {
#ifdef SDEBUG
     fprintf( stderr, "cannot open Hobbs file for writing\n", counter );
#endif
     }
     write( filenum, ToRAM, 16 );
     close( filenum );
     
#ifdef OFBdebug
     fprintf( stderr, "OFBWriteHobbs\n" );
#endif
}
