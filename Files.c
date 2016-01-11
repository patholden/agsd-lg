/*
 static char rcsid[] = "$Id: Files.c,v 1.7 2010/05/26 17:53:30 ags-sw Exp ags-sw $";
*/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <sys/stat.h>
#include <sys/utsname.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include "BoardComm.h"
#include "AppCommon.h"
#include "parse_data.h"
#include "Files.h"
#include "compile.h"
#include "Hobbs.h"
#include "comm_loop.h"
#include "DoLevelScan.h"
#include "SensorSearch.h"
#include "ParseVisionFocus.h"
#include "asciiID.h"
#include "ParseAutoFocus.h"

#define  BIG_SIZE   AGS_SIZE
#define  HOBBS_BUFF_SIZE 128
#define  KERN_BUFF_SIZE  256
static char *pVersionBuff = 0;

static char BIG_Buffer[BIG_SIZE];

static int filelength;

static int ReadLevel ( char *buff, uint32_t * size, int32_t offset, int32_t request );
static int GetVersionSize(uint32_t*size);
static int GetVersionData(struct parse_getdata_resp *resp_buff, uint32_t *size);
static int ReadReturn(char *buff, uint32_t * size, int32_t offset, int32_t request);
static int ReadVersion ( char *buff, uint32_t * size );

/*
 * for now, the largest buffer shall be for the AGS executable
 */

static char default_auto[] =
    "LowLimit = 0\r\n"
    "HighLimit = 0\r\n"
    "SeekHomeDirection = CCW\r\n"
    "2 = 0\r\n"
    "3 = 0\r\n"
    "4 = 0\r\n"
    "5 = 0\r\n"
    "6 = 0\r\n"
    "7 = 0\r\n"
    "8 = 0\r\n"
    "9 = 0\r\n"
    "10 = 0\r\n"
    "11 = 0\r\n"
    "12 = 0\r\n"
    "13 = 0\r\n"
    "14 = 0\r\n"
    "15 = 0\r\n"
    "16 = 0\r\n"
    "17 = 0\r\n"
    "18 = 0\r\n"
    "19 = 0\r\n"
    "20 = 0\r\n"
    "21 = 0\r\n"
    "22 = 0\r\n"
    "23 = 0\r\n"
    "24 = 0\r\n"
    "25 = 0\r\n"
    ;
static char autotext[] = " = ";
static char cal_name[] = "calibration";
static char ini_name[] = "initialization";
static char nfo_name[] = "information";
static char ver_name[] = "version";
static char ags_name[] = "ags.gz";
static char auto_name[] = "autofocus";
static char hob_name[] = "hobbs";
static char level_name[] = "levelscandata";
static char vision_name[] = "visionparameters";
static char focus_vision_name[] = "visionfocus";
static char return_name[] = "targetreturns";
static uint32_t saveLength;

const char *time_string = AGS_COMPILE_TIME "\r\n"
                          AGS_ASCII_ID "\r\n"
                          ;

static int WriteToBuffer ( char *buff,     int32_t offset, int32_t size );
static int WriteBufferToFS (char * name, int32_t Size );
static int ReadFromFS ( char *buff,   char * name, int32_t offset, int32_t request );
static int GetFileSize(char *name, uint32_t*size);
static int GetBufferSize ( int32_t request,   int32_t * size );

void   DoFileGetStart (struct lg_master *pLgMaster, char * parameters, uint32_t respondToWhom )
{
  char     FSName[128];
  char     system_buff[128];
  char lcName[LCNAME_SIZE];
  int  i,j;
  int err=0;
  uint32_t inp_filelen;
  uint32_t local_len;
  uint32_t size = 0;
  struct parse_getstart_parms *pInp=(struct parse_getstart_parms *)parameters;
  struct parse_basic_resp *pRespErr=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
  struct parse_getstart_resp *pRespGood=(struct parse_getstart_resp *)pLgMaster->theResponseBuffer;
  

    // Initialize all buffers to be used
    memset(FSName, 0, sizeof(FSName));
    memset(lcName, 0, sizeof(lcName));
    memset(pRespGood, 0, sizeof(struct parse_getstart_resp));
    memset(system_buff, 0, sizeof(system_buff));

    inp_filelen = strlen(pInp->inp_filename);

    // Copy incoming filename to local buffer & set to all lowercase
    for (i=0, j=0; i<inp_filelen; i++)
      {
	if (isalpha(pInp->inp_filename[i]))
	  {
	    if (isupper(pInp->inp_filename[i]))
	      lcName[j++] = tolower(pInp->inp_filename[i]);
	  }
	lcName[j++] = pInp->inp_filename[i];
      }

    local_len = strlen(lcName);
    if (!local_len)
      {
	pRespErr->hdr.cmd = RESPFAIL;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      }    
      
    if ( strcmp( ini_name, lcName ) == 0 ) {
        strcpy( FSName, "/etc/ags/conf/init" );
    }
    else if ( strcmp( nfo_name, lcName ) == 0 ) {
       strcpy( FSName, "/etc/ags/conf/info" );
    }
    else if ( strcmp( cal_name, lcName ) == 0 ) {
        strcpy( FSName, "/etc/ags/conf/calib" );
    }
    else if ( strcmp( ags_name, lcName ) == 0 ) {
        strcpy( FSName, "/agsd" );
    }
    else if ( strcmp( auto_name, lcName ) == 0 ) {
        strcpy( FSName, "/etc/ags/conf/autofocus" );
    }
    else if ( strcmp( vision_name, lcName ) == 0 ) {
        strcpy( FSName, "/etc/ags/conf/vision" );
    }
    else if ( strcmp( focus_vision_name, lcName ) == 0 ) {
        strcpy( FSName, "/etc/ags/conf/focusvision" );
    }
    else if ( strcmp( hob_name, lcName ) == 0 ) {
        strcpy( FSName, "/etc/ags/conf/hobbs" );
    }
    else if ( strcmp( ver_name, lcName ) == 0 ) {
        strcpy( FSName, "/etc/ags/conf/version" );
    }
    else
      {
	pRespErr->hdr.cmd = RESPFAIL;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      }    

    if (strcmp(return_name, lcName) == 0)
        size = gLoutSize;
    // Special-case version "file"
    // (it's actually a statically declared buffer
    else if (strcmp(ver_name, lcName) == 0)
      {
	size = 0;
	err = (GetVersionSize(&size));
	if (err)
	  {
	    perror("\nFILEGETSTART: Get-Filesize error");
	    fprintf(stderr,": infile %s, fsfile %s",lcName, FSName);
	  }
      }
    else
      {
	size = 0;
	// First convert file to DOS text file
	sprintf(system_buff, "unix2dos -o %s >> /dev/null", FSName);
	system(system_buff);
	err = GetFileSize(FSName, &size);
	if (err)
	  {
	    perror("\nFILEGETSTART: Get-Filesize error");
	    fprintf(stderr,": infile %s, fsfile %s",lcName, FSName);
	  }
      }

    if (err || (size == 0))
      {
	pRespErr->hdr.cmd = RESPFAIL;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      }
    else
      {
	pRespGood->hdr.status = RESPGOOD;
	pRespGood->resp_filelen = size;
	HandleResponse (pLgMaster, (sizeof(struct parse_getstart_resp)-kCRCSize), respondToWhom );
      }
    return;
}

static int GetVersionData(struct parse_getdata_resp *resp_buff, uint32_t *size)
{
  uint32_t   ver_size;
  
  if (!pVersionBuff)
    {
      if (GetVersionSize(size))
	return(-1);
    }
  ver_size = strlen(pVersionBuff);
  if (!ver_size)
    return(-2);
  memcpy(resp_buff->resp_buffer, pVersionBuff, ver_size);
  *size = ver_size;
  free(pVersionBuff);
  return(0);
}

void DoFileGetData  (struct lg_master *pLgMaster, char * parameters, uint32_t respondToWhom )
{
    char lcName[32];
    char FSName[128];
    char * ptr;
    struct parse_basic_resp *pRespErr=(struct parse_basic_resp*)pLgMaster->theResponseBuffer;
    struct parse_getdata_parms *pInp=(struct parse_getdata_parms *)parameters;
    struct parse_getdata_resp *pRespGood=(struct parse_getdata_resp*)pLgMaster->theResponseBuffer;
    int i,j;
    int err;
    uint32_t MaxSize;
    uint32_t request=0;
    uint32_t offset=0;
    uint32_t size;
    uint32_t inp_filelen;
    uint32_t local_len;
    
    // Initialize buffers here
    memset(lcName, 0, sizeof(lcName));
    memset(pRespGood, 0, sizeof(struct parse_getdata_resp));

    // Check length of incoming filename & make sure we have something
    inp_filelen = strlen(pInp->inp_filename);
    for (i=0, j=0; i<inp_filelen; i++)
      {
	if (isalpha(pInp->inp_filename[i]))
	  {
	    if (isupper(pInp->inp_filename[i]))
	      lcName[j++] = tolower(pInp->inp_filename[i]);
	  }
	lcName[j++] = pInp->inp_filename[i];
      }
    local_len = strlen(lcName);
    if (!local_len)
      {
	pRespErr->hdr.cmd = RESPFAIL;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      }    

    offset = pInp->inp_offset;
    request = pInp->inp_numbytes;
    if (request > BUFF_SIZE)
      {
	pRespErr->hdr.cmd = RESPFAIL;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	return;
      }
    
    MaxSize = 0;
    if ( strcmp( ini_name, lcName ) == 0 ) {
      MaxSize = INIT_SIZE;
      strcpy( FSName, "/etc/ags/conf/init" );
    }
    else if ( strcmp( nfo_name, lcName ) == 0 ) {
      MaxSize = INFO_SIZE;
      strcpy( FSName, "/etc/ags/conf/info" );
    }
    else if ( strcmp( cal_name, lcName ) == 0 ) {
      MaxSize = CALIB_SIZE;
      strcpy( FSName, "/etc/ags/conf/calib" );
    }
    else if ( strcmp( ags_name, lcName ) == 0 ) {
      MaxSize = AGS_SIZE;
      strcpy( FSName, "/agsd" );
    }
    else if ( strcmp( auto_name, lcName ) == 0 ) {
      MaxSize = AUTO_SIZE;
      strcpy( FSName, "/etc/ags/conf/autofocus" );
    }
    else if ( strcmp( vision_name, lcName ) == 0 ) {
      MaxSize = BIG_SIZE;
      strcpy( FSName, "/etc/ags/conf/vision" );
    }
    else if ( strcmp( focus_vision_name, lcName ) == 0 ) {
      MaxSize = DATA_SIZE;
      strcpy( FSName, "/etc/ags/conf/focusvision" );
    }
    else if ( strcmp( hob_name, lcName ) == 0 ) {
      MaxSize = HOBB_SIZE;
      strcpy( FSName, "/etc/ags/conf/hobbs" );
    }
    else if (strcmp(ver_name, lcName) == 0 )
      {
	err = GetVersionData(pRespGood, &size);
	goto handle_resp;
      }
    else      
      sprintf(FSName, "etc/ags/conf/%s",lcName);

    MaxSize = 0;
    err = GetFileSize(FSName, &MaxSize);
    if (err)
      {
	// Handle special cases here.
	if ( strcmp( level_name, lcName ) == 0 ) {
	  MaxSize = AGS_SIZE;
	  err = ReadLevel(pRespGood->resp_buffer, &size, offset, request);
	}
	else if ( strcmp( return_name, lcName ) == 0 ) {
	  MaxSize = AGS_SIZE;
	  err = ReadReturn(pRespGood->resp_buffer, &size, offset, request);
	}
      else
	{
	  perror("\nFILEGETDATA: File not Found!");
	  pRespErr->hdr.cmd = RESPFAIL;
	  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	  return;
	}
      }
    err = 0;
    size = 0;
    if (MaxSize)
      {
	if (((offset + request) > MaxSize) ||
	    (request > (sizeof(struct parse_getdata_resp) - kCRCSize - offsetof(struct parse_getdata_resp,resp_buffer))))
	  {
	    err = -1;
	    fprintf(stderr, "\nFILEGETDATA: BAD in-file %s, our-file %s\n\toffs %d, req %d, size%d", lcName, FSName,offset, request, MaxSize);
	  }
	else
	  {
	    err = ReadFromFS(pRespGood->resp_buffer, FSName, offset, request);
	    size = request;
	    if (err)
	      {
		if ( strcmp( auto_name, lcName ) == 0 )
		  {
		    ptr = strstr(pRespGood->resp_buffer, autotext );
		    if ( ptr == NULL ) {
		      strcpy(pRespGood->resp_buffer, default_auto );
		      size = strlen (pRespGood->resp_buffer);
		      MaxSize = size;
		      err = 0;
		    }
		  }
		else if (strcmp(auto_name, lcName) == 0)
		  {
		    strcpy(pRespGood->resp_buffer, default_auto );
		    size = strlen (pRespGood->resp_buffer);
		    MaxSize = size;
		    err = 0;
		  }
	      }
	  }
      }
    if (MaxSize == 0)
      err = -1;
    
    // Prep response buffer for new response data
handle_resp:
    if ( err )
      {
	fprintf(stderr, "\nGETDATA: failed file %s, size %x, offset %d, request %d", FSName, MaxSize,offset,request);
	pRespErr->hdr.cmd = RESPFAIL;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      }
    else
      {
	// The contents of the requested file have already been copied
	// to response buffer at this point
	pRespGood->hdr.cmd = RESPGOOD;
	memcpy(pRespGood->lcName, (void *)lcName, LCNAME_SIZE);
	pRespGood->resp_offset = offset;
	pRespGood->resp_numbytes = size;
	HandleResponse (pLgMaster, (size+offsetof(struct parse_getdata_resp,resp_buffer)), respondToWhom );
      }
    return;
}
 
void   DoFilePutStart (struct lg_master *pLgMaster, char * parameters, uint32_t respondToWhom )
{
    int32_t MaxSize;
    char lcName[32];
    struct parse_putstart_parms *pInp=(struct parse_putstart_parms *)parameters;
    struct parse_putstart_resp *pResp=(struct parse_putstart_resp*)pLgMaster->theResponseBuffer;
    int i,j;
    int err;
    uint32_t request;
    uint32_t inp_filelen;
    uint32_t local_len;

    memset(BIG_Buffer, 0, BIG_SIZE );
    saveLength = 0;

    memset(lcName, 0, 32);
    inp_filelen = strlen(pInp->inp_filename);

    // Copy incoming filename to local buffer & set to all lowercase
    for (i=0, j=0; i<inp_filelen; i++)
      {
	if (isalpha(pInp->inp_filename[i]))
	  {
	    if (isupper(pInp->inp_filename[i]))
	      lcName[j++] = tolower(pInp->inp_filename[i]);
	  }
	lcName[j++] = pInp->inp_filename[i];
      }

    local_len = strlen(lcName);
    if (!local_len)
      {
	pResp->hdr.cmd = RESPFAIL;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	return;
      }    
      
    request = pInp->inp_filelen;
    if (request > BIG_SIZE)
      {
	pResp->hdr.cmd = RESPFAIL;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	return;
      }    
	
    MaxSize = 0;
    if ( strcmp( ini_name, lcName ) == 0 ) {
        MaxSize =    INIT_SIZE;
    }
    else if ( strcmp( nfo_name, lcName ) == 0 ) {
        MaxSize =    INFO_SIZE;
    }
    else if ( strcmp( cal_name, lcName ) == 0 ) {
        MaxSize =    CALIB_SIZE;
    }
    else if ( strcmp( ags_name, lcName ) == 0 ) {
        MaxSize =    AGS_SIZE;
    }
    else if ( strcmp( hob_name, lcName ) == 0 ) {
        MaxSize =    HOBB_SIZE;
    }
    else if ( strcmp( auto_name, lcName ) == 0 ) {
        MaxSize =    AUTO_SIZE;
    }
    else if ( strcmp( focus_vision_name, lcName ) == 0 ) {
        MaxSize =    DATA_SIZE;
    }
    else if ( strcmp( vision_name, lcName ) == 0 ) {
        MaxSize =    BIG_SIZE;
    }
    else MaxSize = 0;
    err = 0;
    if ( request > MaxSize ) { err = -1; }
    if ( MaxSize == 0 ) { err = -1; }

    // Prep message to go back to sender
    if (err)
      pResp->hdr.cmd = RESPFAIL;
    else
      {
        saveLength = request;
	pResp->hdr.cmd = RESPGOOD;
      }
      memcpy(pResp->filename, (void *)lcName, 32 );
      pResp->resp_filelen = MaxSize;
      HandleResponse (pLgMaster, (sizeof(struct parse_putstart_resp) - kCRCSize) , respondToWhom );
      return;
}

void   HandleFilePutData  (struct lg_master *pLgMaster, char * parameters, uint32_t respondToWhom )
{
    int32_t MaxSize=0;
    char lcName[LCNAME_SIZE];
    int i,j;
    int err;
    int32_t offset;
    int32_t request;
    int32_t length;
    int32_t inp_filelen;
    int32_t local_len;
    struct parse_putdata_parms *pInp=(struct parse_putdata_parms *)parameters;
    struct parse_putdata_resp *pResp=(struct parse_putdata_resp *)pLgMaster->theResponseBuffer;
    
    err = 0;
    memset(lcName, 0, sizeof(lcName));
    memset(pResp, 0, sizeof(struct parse_putdata_resp));
    
    inp_filelen = strlen(pInp->inp_filename);
    
    // Copy incoming filename to local buffer & set to all lowercase
    for (i=0, j=0; i<inp_filelen; i++)
      {
	if (isalpha(pInp->inp_filename[i]))
	{
	    if (isupper(pInp->inp_filename[i]))
	      lcName[j++] = tolower(pInp->inp_filename[i]);
	  }
	lcName[j++] = pInp->inp_filename[i];
      }

    local_len = strlen(lcName);
    if (!local_len)
      {
	fprintf(stderr,"\nPUTDATA: file %s not found", pInp->inp_filename);
	pResp->hdr.cmd = RESPFAIL;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	return;
      }    

    offset = pInp->inp_offset;
    request = pInp->inp_buf_numbytes;
    length  = pInp->inp_write_numbytes;
    if (length > (sizeof(struct parse_putdata_parms)-offsetof(struct parse_putdata_parms, inp_buffer)))
      {
	fprintf(stderr,"\nPUTDATA: file %s bad buffer length %d", lcName,length);
	pResp->hdr.cmd = RESPFAIL;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	return;
      }
    if (request > sizeof(pInp->inp_buffer))
      {
	fprintf(stderr,"\nPUTDATA: file %s bad buffer len %d", lcName, request);
	pResp->hdr.cmd = RESPFAIL;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	return;
      }
#if defined(ZDEBUG)
    fprintf(stderr, "Files515 off %d req %d len %d\n", offset, request, length );
#endif

    if ( strcmp( ini_name, lcName ) == 0 ) {
      MaxSize =    INIT_SIZE;
    }
    else if ( strcmp( nfo_name, lcName ) == 0 ) {
      MaxSize =    INFO_SIZE;
    }
    else if ( strcmp( cal_name, lcName ) == 0 ) {
        MaxSize =    CALIB_SIZE;
    }
    else if ( strcmp( ags_name, lcName ) == 0 ) {
        MaxSize =    AGS_SIZE;
    }
    else if ( strcmp( hob_name, lcName ) == 0 ) {
        MaxSize =    HOBB_SIZE;
    }
    else if ( strcmp( auto_name, lcName ) == 0 ) {
        MaxSize =    AUTO_SIZE;
    }
    else if ( strcmp( focus_vision_name, lcName ) == 0 ) {
        MaxSize =    DATA_SIZE;
    }
    else if ( strcmp( vision_name, lcName ) == 0 ) {
        MaxSize =    BIG_SIZE;
    }

    if (((offset+length) > saveLength) || ((offset+length) > BIG_SIZE))
      err = -1;
    else
      {
	err = WriteToBuffer(pInp->inp_buffer, offset, length  );
#if defined(ZDEBUG)
	fprintf( stderr, "line 470 file %s offs %d, len %d, err %s\n", lcName, offset, length, err );
#endif
	if (err)
	  {
	    pResp->hdr.cmd = RESPFAIL;
	    HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	    return;
	  }
      }
    
    if (MaxSize == 0) { err = -1; }
    if ( err ) {
      pResp->hdr.cmd = RESPFAIL;
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      return;
    } else {
        if ( !(pLgMaster->gHeaderSpecialByte & 0x80)  ) {
	  pResp->hdr.cmd = RESPGOOD;
	  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	  return;
        }
    }
    return;
}

void   DoFilePutDone  ( struct lg_master *pLgMaster, char * parameters, uint32_t respondToWhom )
{
    int32_t MaxSize=0;
    char lcName[LCNAME_SIZE];
    char FSName[128];
    int i,j;
    int itest;
    int err;
    int32_t size;
    int32_t inp_filelen;
    int32_t local_len;
    char * ptr;
    char localcopy[INIT_SIZE];
    char ttstring[] = "transformtolerance";
    struct parse_putdone_parms *pInp=(struct parse_putdone_parms *)parameters;
    struct parse_putdone_resp *pResp=(struct parse_putdone_resp *)pLgMaster->theResponseBuffer;
    
    err = 0;
    memset(FSName, 0, sizeof(FSName));
    memset(lcName, 0, sizeof(lcName));
    memset(pResp, 0, sizeof(struct parse_putdone_resp));
    
    inp_filelen = strlen(pInp->inp_filename);

    // Copy incoming filename to local buffer & set to all lowercase
    for (i=0, j=0; i<inp_filelen; i++)
      {
	if (isalpha(pInp->inp_filename[i]))
	{
	    if (isupper(pInp->inp_filename[i]))
	      lcName[j++] = tolower(pInp->inp_filename[i]);
	  }
	lcName[j++] = pInp->inp_filename[i];
      }

    local_len = strlen(lcName);
    if (!local_len)
      {
	pResp->hdr.cmd = RESPFAIL;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      }    
    MaxSize = 0;
    if ( strcmp( ini_name, lcName) == 0)
      {
	fprintf( stderr, "PUTDONE: file %s, length %d\n", lcName, saveLength );
        MaxSize = INIT_SIZE;
        strcpy( FSName, "/etc/ags/conf/init" );
        memset( (void *)localcopy, 0, INIT_SIZE );
        if (saveLength >= INIT_SIZE)
	  {
	    fprintf( stderr, "saveLength %d > expected len%d\n", saveLength, INIT_SIZE );
	    pResp->hdr.cmd = RESPFAIL;
	    HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	    return;
	  }
        strncpy(localcopy, BIG_Buffer, saveLength );
        size = saveLength;
        for (i=0; i<size; i++)
	  {
	    if (isupper(localcopy[i]))
	      localcopy[i] = tolower( localcopy[i] );
	  }
        ptr = strstr( localcopy, ttstring );
        if (ptr == NULL)
	  {
	    for (i=0; i<size; i++)
	      {
		if (!isprint(localcopy[i]))
		  localcopy[i] = '?';
              }
	    fprintf(stderr, "PUTDONE:  no transform tolerance\n%s\n",localcopy);
	    MaxSize = 0;
	  }
      }
    else if ( strcmp( nfo_name, lcName ) == 0 ) {
        MaxSize = INFO_SIZE;
        strcpy( FSName, "/etc/ags/conf/info" );
    }
    else if ( strcmp( cal_name, lcName ) == 0 ) {
        MaxSize = CALIB_SIZE;
        strcpy( FSName, "/etc/ags/conf/calib" );
    }
    else if ( strcmp( ags_name, lcName ) == 0 ) {
        MaxSize = AGS_SIZE;
        strcpy( FSName, "/agsd" );
    }
    else if ( strcmp( hob_name, lcName ) == 0 ) {
        MaxSize =    HOBB_SIZE;
        strcpy( FSName, "/etc/ags/conf/hobbs" );
    }
    else if ( strcmp( vision_name, lcName ) == 0 ) {
        MaxSize =    BIG_SIZE;
        strcpy( FSName, "/etc/ags/conf/vision" );
    }
    else if ( strcmp( auto_name, lcName ) == 0 ) {
        MaxSize =    AUTO_SIZE;
        strcpy( FSName, "/etc/ags/conf/autofocus" );
           // parse buffer to check file
        itest = ParseAutoFocus(saveLength, BIG_Buffer);
        if ( itest != 0 ) {
             MaxSize = 0;
        }
    }
    else if ( strcmp( focus_vision_name, lcName ) == 0 ) {
        MaxSize =    DATA_SIZE;
        strcpy( FSName, "/etc/ags/conf/focusvision" );
           // parse buffer to check file
        itest = ParseVisionFocus( saveLength, BIG_Buffer );
        if ( itest != 0 ) {
             MaxSize = 0;
        }
    }
    else
        sprintf(FSName, "/etc/ags/conf/%s",pInp->inp_filename);

    fprintf( stderr, "\nPUTDONE: file %s, write-len %d  MaxSize %d\n", FSName, saveLength, MaxSize );

    if (MaxSize)
      {
        err = GetBufferSize( MaxSize, &size );
        if ( !err && (size == saveLength)) 
	  err = WriteBufferToFS( FSName, saveLength );
        else
            err = -1;
      }
    else
        err = -1;
    if (err)
      {
	pResp->hdr.cmd = RESPFAIL;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      }
    else
      {
	pResp->hdr.cmd = RESPGOOD;
	memcpy(pResp->filename, (void *)lcName, 32 );
	pResp->filelen = size;
	HandleResponse (pLgMaster, (sizeof(struct parse_putdone_resp)-kCRCSize), respondToWhom );
      }
    return;
}




static int ReadLevel ( char *buff, uint32_t * size, int32_t offset, int32_t request )
{
  char * ptr;
  uint32_t x, y;
  uint32_t jndex;

#ifdef ZZZDEBUG
fprintf( stderr, "entering ReadLevel   buff %d\n", buff );
#endif

  memset( buff, 0, BIG_SIZE );
  memset( BIG_Buffer, 0, BIG_SIZE );
  ptr = &(BIG_Buffer[0]);

  for ( y = 0x1U; y <= LSCNTSTEP; y += 0x1U ) {
         for ( x = 0x1U; x <= LSCNTSTEP; x += 0x1U ) {
             jndex = y * LSCNTSTEP + x;
             ptr += sprintf( ptr,  "%2d %2d %8d", x, y, scandata[jndex] );
             ptr += sprintf( ptr, "\r\n" );
         }
         ptr += sprintf( ptr, "\r\n" );
  }

  strncpy( buff, &(BIG_Buffer[offset]), request );

  *size = strlen( buff );
  
#ifdef ZZZDEBUG
fprintf( stderr, "ReadLevel  size %d\n", *size );
#endif
  return( 0 );

}


static int ReadReturn ( char *buff, uint32_t * size, int32_t offset, int32_t request )
{
  char * ptr;
  int nget;

#if defined(ZZZDEBUG) || defined(LOUTDEBUG)
  fprintf( stderr, "entering ReadReturn  buff %d  off %d req %d\n", *buff, offset, request );
#endif


  if ( (offset + request) >= gLoutSize ) {
     nget = gLoutSize - offset;
  } else {
     nget = request;
  }
  *size = nget;

  ptr = gLoutBase;
  memcpy( buff, (char *)(&(ptr[offset])), nget );
  
#if defined(ZZZDEBUG) || defined(LOUTDEBUG)
fprintf( stderr, "ReadReturn  size %d\n", *size );
#endif
  return( 0 );

}

static int ReadVersion ( char *buff, uint32_t * size )
{
  int length;
  int hobbs=0;
  char hobb_buffer[HOBBS_BUFF_SIZE];
  int hlen;
  char kernel_buffer[256];
  int klen;
  struct utsname utsbuff;

  memset( hobb_buffer, 0, 128 );
  memset( kernel_buffer, 0, 256 );
  length = strlen( time_string );

#ifdef ZDEBUG
fprintf( stderr, "\nsize %d %d  \n\n%s\n", *size, length, time_string );
#endif

  memmove( (void *)buff, (void *)time_string, (size_t)length);

  uname( &utsbuff ) ;
  klen = sprintf( kernel_buffer
                , "kernel version %s %s\r\n"
                , utsbuff.release
                , utsbuff.version
                );
  strncat( buff, kernel_buffer, klen );
#ifdef ZDEBUG
fprintf( stderr
       , "%s\n%s\n klen %d,  len %d\n"
       , kernel_buffer
       , buff
       , klen
       , length
       );
#endif

//FIXME---PAH---NEED TO FIX HOBBS COUNT
// hobbs = ReadHobbs( "hobbs" );
  hlen = sprintf( hobb_buffer
                , "Hobbs %-10d \r\n"
                  "QuickCheck V2.0\r\n"
                , hobbs
                );
  strncat( buff, hobb_buffer, hlen );
#ifdef ZDEBUG
  fprintf( stderr
	   , "%s\n%s\n hlen %d,  len %d\n"
	   , hobb_buffer
	   , buff
	   , hlen
	   , length
	   );
#endif
  *size = length + klen + hlen;
#ifdef ZDEBUG
  fprintf( stderr, "\ninput_size %d, time size %d\n\n", *size, length);
#endif
 return(0);
}

static int WriteToBuffer ( char *buff, int32_t offset, int32_t size )
{
  int err;

  err = 0;

  if((size+offset) >= BIG_SIZE)
    return(-1);
  memmove( (void *)(&(BIG_Buffer[offset])), (void *)buff, (size_t) size );
#ifdef SDEBUG
  printf( "1176 size %d  offset %d\n", size, offset );
#endif

  return err;
}

static int WriteBufferToFS ( char * name, int32_t Size )
{
  int request, size, i;
  char *ptr;
  int filenum;
  int length;
  int  err;
  char delete[80]  = "rm -f ";

  request = Size;
  i = request;
  ptr = &(BIG_Buffer[request-1]);
  while ( ( *ptr== 0 ) && ( i > 0 ) ) {
       ptr--; i--;
  }
  size = i;
  if( size > BIG_SIZE) {size = BIG_SIZE;}

#ifdef SDEBUG
fprintf( stderr, "%s", BIG_Buffer );
fprintf( stderr, "size %d\n", size );
#endif

  strcat( delete, name );
  err = system( delete );
  if (err)
    return(-1);
  
  filenum = open( name
                , O_CREAT|O_RDWR
                , S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH
                );
  if (filenum <= 0)
    return(-2);
  length = write( filenum, BIG_Buffer, size );
  close( filenum );
  if (length <=0)
    return(-3);
  return(0);
}

static int ReadFromFS ( char *buff, char * name, int32_t offset, int32_t request )
{
  int    filenum;
  int    length;

#ifdef SDEBUG
  printf( "ReadFromFS req %d off %d \n", request, offset );
#endif
  
  if ( offset == 0 )
  {
       memset( (void *)BIG_Buffer, 0, BIG_SIZE );
       filelength = 0;

       // FIXME---PAH---CONVERT TO FILE OPERATIONS
       filenum = open( name, O_RDONLY );
       if ( filenum <= 0 ) {
         printf( "error opening %s\n", name );
         return(1);
       }
       filelength = read( filenum, BIG_Buffer, BIG_SIZE );
#ifdef SDEBUG
printf( "1265 file size %d\n",length );
#endif
       close( filenum );
  }
 
  if ( filelength > 0 ) {
     if ( filelength > request )
          length = request;
     else
          length = filelength;

     memmove( (void *)buff, (void *)(&(BIG_Buffer[offset])), (size_t)(length) );
  }

  return(0);
}
static int GetVersionSize(uint32_t*size)
{
  if (!pVersionBuff)
    pVersionBuff = malloc(HOBBS_BUFF_SIZE+ KERN_BUFF_SIZE + strlen(time_string));
  if (!pVersionBuff)
    return(-1);
  if (ReadVersion(pVersionBuff, size))
      return(-2);

  return(0);
}
static int GetFileSize(char *filename, uint32_t*size)
{
  FILE *handle;
  long  file_size;

  handle = fopen(filename, "r+");
  if (!handle)
    return(-1);

  // Figure out length of file
  fseek(handle, 0L, SEEK_END);
  file_size = ftell(handle);
  fseek(handle, 0L, SEEK_SET);
  *size = (uint32_t)(file_size & 0xFFFFFFFF);
  fclose(handle);
  return(0);
}

int GetBufferSize ( int32_t request, int32_t * size )
{
  int i;
  char *ptr;

  i = request;  ptr = &(BIG_Buffer[request-1]);
  while ( ( *ptr== 0 ) && ( i > 0 ) ) {
       ptr--; i--;
  }
  *size = i;

  return(0);
}

