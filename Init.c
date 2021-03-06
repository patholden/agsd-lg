/*
 * static char rcsid[] = "$Id: Init.c,v 1.12 2001/01/03 17:58:15 ags-sw Exp ags-sw $";
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <ctype.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "AppCommon.h"
#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "LaserInterface.h"
#include "LaserPattern.h"
#include "Init.h"
#include "SensorSearch.h"
#include "QuickCheckManager.h"
#include "Video.h"
#include "Files.h"
#include "Hobbs.h"

void DoReInit( struct lg_master *pLgMaster, uint32_t respondToWhom )
{
    struct parse_basic_resp *pResp;

  pResp = (struct parse_basic_resp *)pLgMaster->theResponseBuffer;
  
  memset(pLgMaster->theResponseBuffer, 0, sizeof(COMM_RESP_MIN_LEN));
  
  syslog(LOG_NOTICE, "OPBDoReInit\n" );
  if(ConfigDataInit(pLgMaster))
    pResp->hdr.status = RESPFAIL;
  FlashLed(pLgMaster, 5 );

#if 0
  // FIXME:  old projector only applies new IP on REBOOT! 
  // Set flag to reset comms
  pLgMaster->gResetComm = 1;
#else
  pLgMaster->gResetComm = 0;
#endif  
  pLgMaster->enet_retry_count = 0;
  pResp->hdr.status = RESPGOOD;
  HandleResponse(pLgMaster, sizeof(uint32_t), respondToWhom);
  return;
}


int ConfigDataInit(struct lg_master* pLgMaster)
{
    char testStr[512];
    char tmpline[512];
    char setipcmd[512];
    char word[512];
    char ipaddr[4];
    char gwaddr[4];
    char bcaddr[4];
    FILE *handle;
    char *localBuff;
    char *token;
    char *ptr;
    double   ArcSteps = 60.0;
    double   dTemp;
    long     file_size;
    size_t   length;
    int32_t  i;
    uint32_t ufactor;
    int      count;
    uint32_t return_code;
    uint32_t  ip0, ip1, ip2, ip3;
    
    // First check version file
    return_code = InitCheckVersion(pLgMaster);
    if (return_code)
      return(return_code);
    // Now read config file & parse objects
    memset(testStr, 0, sizeof(testStr));
    memset(setipcmd, 0, sizeof(setipcmd));
    memset(ipaddr, 0, sizeof(ipaddr));
    memset(ipaddr, 0, sizeof(bcaddr));
    memset(ipaddr, 0, sizeof(gwaddr));

    handle = fopen("/laservision/data/init", "r");
    if (handle == NULL)
      { 
	syslog(LOG_ERR, "init file not found, errno %d, looking for backup", errno);
	handle = fopen("/backup/data/init", "r");
      if (handle == NULL)
	{ 
	  syslog(LOG_ERR, "init file not found, errno %d, using defaults", errno);
	  return(0);
	}
      }
    fseek(handle, 0L, SEEK_END);
    file_size = ftell(handle);
    fseek(handle, 0L, SEEK_SET);   // Reset back to beginning of file
    // Get length of file, get a local buffer, and then read it.
    if (file_size <=0) {
      syslog(LOG_ERR, "/laservision/data/init incorrect file size %lx\n", file_size );
      return(-2);
    }
    localBuff = malloc(file_size);
    if (localBuff <=0)
      {
	syslog(LOG_ERR, "Unable to malloc a local buffer\n" );
	return(-3);
      }
    length = (int)fread((void *)localBuff, (size_t)file_size, 1, handle );
    if (length <=0)
      syslog(LOG_ERR, "initialization file bad size %ld,errno %d\n",(long)file_size,errno);
    else
      syslog(LOG_NOTICE, "initialization file size %ld\n",(long)file_size);
    fclose(handle);

    ptr = localBuff;
    // Need to change all uppercase letters to lowercase
    for (i=0; i<file_size; i++, ptr++)
      {
	if (isalpha(*ptr))
	  {
	    if (*ptr == '\r') *ptr = '\n';
	    if (isupper(*ptr))
	      *ptr = tolower(*ptr);
	  }
      }

    // default mode is "VISION"
    pLgMaster->projector_mode    = PROJ_VISION;
    
    token = localBuff; 
    count = 0;
    while ((count < file_size) && (*token != 0))
      {
	strncpy(tmpline, token, 512);
	for (i = 0; i < 512; i++)
	  {
	    if ((tmpline[i] == 0x0a) || (tmpline[i] == 0x0d))
	      tmpline[i] = 0;
	  }
	
	strcpy( testStr, "transformtolerance =" );
	if (strncmp( tmpline, testStr, strlen(testStr)) == 0)
	  {
	    sscanf( tmpline, "transformtolerance = %lf", &dTemp );
	    if( (dTemp >= 1.0e-10) && (dTemp < 1.0) ) {
	      pLgMaster->gArgTol  = dTemp;
	    }
	  }
	
	strcpy(testStr, "displayperiod =");
	if ( strncmp( tmpline, testStr, strlen(testStr)) == 0)
	  {
	    sscanf( tmpline, "displayperiod = %d", &pLgMaster->gPeriod );
	    if (pLgMaster->gPeriod < KETIMER_50U)
	      pLgMaster->gPeriod = KETIMER_50U;
	  }
	strcpy(testStr, "curveinterpolation =" );
	if (strncmp(token, testStr, strlen(testStr)) == 0)
	  {
	    sscanf( token, "curveinterpolation = %lf", &gCurveMin );
	    if (gCurveMin < 0.01)
	      gCurveMin = 0.01;
	    if (gCurveMin > 10.0)
	      gCurveMin = 10.0;
	    if (gCurveMin > gCurveMax)
	      gCurveMin = 0.5 * gCurveMax;
	  }
	strcpy( testStr, "arcsteps =" );
	if (strncmp(token, testStr, strlen(testStr)) == 0)
	  {
	    sscanf( token, "arcsteps = %lf", &ArcSteps );
	    if (ArcSteps < 60.0)
	      ArcSteps = 60.0;
	    //  old 32bit range pLgMaster->dmax = 4294967296.0 / ArcSteps;
	    pLgMaster->dmax = 65536.0 / ArcSteps;
	  }
	strcpy( testStr, "beamlinearrangex =" );
	if ( strncmp( tmpline, testStr, strlen(testStr) ) == 0 ) {
	  sscanf( tmpline, "beamlinearrangex = %lf", &gBeamLinearRangeX ); }
	strcpy( testStr, "beamlinearrangey =" );
	if ( strncmp( tmpline, testStr, strlen(testStr) ) == 0 ) {
	  sscanf( tmpline, "beamlinearrangey = %lf", &gBeamLinearRangeY ); }
	strcpy( testStr, "coarsefactor" );
	if ( strncmp( token, testStr, strlen(testStr) ) == 0 ) {
	  sscanf( token, "coarsefactor = %d", &pLgMaster->gCoarse2Factor); 
	  pLgMaster->gCoarse2SearchStep = pLgMaster->gCoarse2Factor;
	  pLgMaster->gNumberOfSpirals = pLgMaster->gSpiralFactor / pLgMaster->gCoarse2Factor;
	}
	strcpy( testStr, "searchstepperiod =" );
	if ( strncmp( tmpline, testStr, strlen(testStr) ) == 0 ) {
	  sscanf( tmpline, "searchstepperiod = %d", &pLgMaster->gSrchStpPeriod ); }
	strcpy( testStr, "coarseperiod =" );
	if ( strncmp( tmpline, testStr, strlen(testStr) ) == 0 ) {
	  sscanf( tmpline, "coarseperiod = %d", &pLgMaster->gCoarsePeriod ); }
	strcpy( testStr, "fineperiod =" );
	if ( strncmp( tmpline, testStr, strlen(testStr) ) == 0 ) {
	  sscanf( tmpline, "fineperiod = %d", &pLgMaster->gFinePeriod ); }
	strcpy( testStr, "sensethreshold =" );
	if ( strncmp( tmpline, testStr, strlen(testStr) ) == 0 ) {
	  sscanf( tmpline, "sensethreshold = %d", &pLgMaster->gSenseThreshold ); }
	strcpy( testStr, "superfinecount =" );
	if ( strncmp( token, testStr, strlen(testStr) ) == 0 ) {
	  sscanf( token, "superfinecount = %d", &gSuperFineCount ); }
	strcpy( testStr, "superfinefactor =" );
	if ( strncmp( token, testStr, strlen(testStr) ) == 0 ) {
	  sscanf( token, "superfinefactor = %d", &ufactor );
	  if ( ufactor > 0x100 ) { ufactor = 0x100; }
	  if ( ufactor <     1 ) { ufactor =     1; }
	  SetSuperFineFactor( ufactor );
	}
	strcpy( testStr, "searchattempts =" );
	if ( strncmp( token, testStr, strlen(testStr) ) == 0 ) {
	  sscanf( token, "searchattempts = %d", &gNumberOfSensorSearchAttempts );
	  if ((gNumberOfSensorSearchAttempts < 1)
	      || (gNumberOfSensorSearchAttempts > 100))
	    {
	      gNumberOfSensorSearchAttempts = 1;
	    }
	}
	strcpy( testStr, "multiplesweeps =" );
	if ( strncmp( token, testStr, strlen(testStr) ) == 0 ) {
	  sscanf( token, "multiplesweeps = %d", &pLgMaster->gMultipleSweeps );
	}
	strcpy( testStr, "dwell =" );
	if ( strncmp( token, testStr, strlen(testStr) ) == 0 ) {
	  sscanf( token, "dwell = %d", &pLgMaster->gDwell );
	  if ( (pLgMaster->gDwell < 10 ) || (pLgMaster->gDwell > 1000) ) {
	  pLgMaster->gDwell = 0;
	  }
	}
        strcpy( testStr, "projectormode =" );
        if ( strncmp( token, testStr, strlen(testStr) ) == 0 ) {
          sscanf( token, "projectormode = %s", word );

          strcpy( testStr, "laser" );                         
          if ( strncmp( word, testStr, strlen(testStr) ) == 0 ) {
            pLgMaster->projector_mode    = PROJ_LASER;        
          }                                                   
          strcpy( testStr, "vision" );                        
          if ( strncmp( word, testStr, strlen(testStr) ) == 0 ) {
            pLgMaster->projector_mode    = PROJ_VISION;       
          }                                                  
        }                                                    
#if 0
	strcpy( testStr, "commtype =" );
	if ( strncmp( token, testStr, strlen(testStr) ) == 0 ) {
	  sscanf( token, "commtype = %s", word );
	  strcpy( testStr, "serial" );
	  if ( strncmp( word, testStr, strlen(testStr) ) == 0 ) {
	    pLgMaster->serial_ether_flag = COMM_SERIAL;
	  }
	  strcpy( testStr, "ether" );
	  if ( strncmp( word, testStr, strlen(testStr) ) == 0 ) {
	    pLgMaster->serial_ether_flag = COMM_ETHER;
	  }
	}
#else
	pLgMaster->serial_ether_flag = COMM_ETHER;
#endif      
	strcpy(testStr, "longorshortthrowsearch =");
	if (strncmp(token, testStr, strlen(testStr)) == 0)
	  {
	    sscanf( token, "longorshortthrowsearch = %s", word );
	    strcpy( testStr, "long" );
	    if (strncmp( word, testStr, strlen(testStr)) == 0)
	      pLgMaster->LongOrShortThrowSearch = 1;
	    strcpy( testStr, "short" );
	    if (strncmp(word, testStr, strlen(testStr)) == 0)
	      pLgMaster->LongOrShortThrowSearch = 0;
	  }
	strcpy(testStr, "ipaddress =");
	if (strncmp(token, testStr, strlen(testStr)) == 0)
	  {
	    syslog(LOG_NOTICE, "Setting new IP to %s", ipaddr);
	    sscanf(token, "ipaddress = %d.%d.%d.%d", &ip0, &ip1, &ip2, &ip3);
	    sprintf(ipaddr, "%d.%d.%d.%d", ip0, ip1, ip2, ip3);
	    ip3 = 1;
	    sprintf(gwaddr, "%d.%d.%d.%d", ip0, ip1, ip2, ip3);
	    ip3 = 254;
	    sprintf(bcaddr, "%d.%d.%d.%d", ip0, ip1, ip2, ip3);
	    sprintf(setipcmd, "sh /agslaser/LVDsetipaddr %s %s %s", ipaddr, bcaddr, gwaddr);
	    system(setipcmd);
	  }
	while ( *token != 0x00 && *token != 0x0a && *token != 0x0d ) {
	  token++;
	  count++;
	}
	if ( *token == 0x0a || *token == 0x0d ) {
	  token++;
	  count++;
	}
        // syslog(LOG_NOTICE, "count ---%d---\n", count );
      }
    free(localBuff);
    return(0);
}
int LGMasterInit(struct lg_master *pLgMaster)
{
  memset((char *)pLgMaster, 0, sizeof(struct lg_master));
  pLgMaster->gInputBuffer = (unsigned char *)malloc( kMaxDataLength );
  pLgMaster->gRawBuffer = (unsigned char *)malloc( kMaxDataLength );
  pLgMaster->theResponseBuffer = (unsigned char *)malloc( kMaxDataLength );
  pLgMaster->gAFInputBuffer = (char *)malloc((size_t)(kSizeOfCommand
					      + kMaxParametersLength
					      + kMaxDataLength
					      + kCRCSize ) );
  pLgMaster->gDataChunksBuffer = (char *)malloc((size_t)kMaxDataLength);
  pLgMaster->gSensorBuffer = (char *)malloc((size_t)
		(kMaxNumberOfPlies * kNumberOfFlexPoints * 2 * sizeof(uint32_t)));
  pLgMaster->gScan = (uint16_t *)calloc(1024, sizeof(int16_t));
  pLgMaster->coarsedata = (uint16_t *)calloc(512*512, sizeof(uint16_t));
  pLgMaster->gLsort = (uint16_t *)calloc((size_t)MAX_LG_BUFFER,sizeof(int16_t));

  if (!pLgMaster->gInputBuffer || !pLgMaster->gRawBuffer
      || !pLgMaster->theResponseBuffer || !pLgMaster->gAFInputBuffer
      || !pLgMaster->gDataChunksBuffer || !pLgMaster->gSensorBuffer
      || !pLgMaster->gScan || !pLgMaster->coarsedata || !pLgMaster->gLsort)
    {
      syslog(LOG_ERR,"\nMalloc failed for one or more master buffers");
      return(-1);
    }
    
  // Initialize elements here
  pLgMaster->gTolerance = 0.0300;
  pLgMaster->gHalfMirror = 0.025;
  pLgMaster->gArgTol = GARGTOL_DEFAULT;
  pLgMaster->gProjectorSerialNumber = -1L;
  pLgMaster->gSrchStpPeriod = KETIMER_75U;
  pLgMaster->gCoarsePeriod = KETIMER_75U;
  pLgMaster->gFinePeriod = KETIMER_75U;
  pLgMaster->gSearchType = SUPERSEARCH;
  pLgMaster->gSenseThreshold = 750;
  pLgMaster->gQCcount = GQCCOUNT_DEFAULT;
  // old 32-bit range pLgMaster->dmax = 4294967296.0 / 60.0;
  pLgMaster->dmax = 65536.0 / 60.0;
  pLgMaster->gCALIBFileOK = false;
  pLgMaster->gCoarse2SearchStep = kCoarseSrchStpDef;
  pLgMaster->gCoarse2Factor= kCoarseFactorDef;
  pLgMaster->gHatchFactor = kHatchFactorDef;
  pLgMaster->gNumberOfSpirals = 128;
  pLgMaster->gSpiralFactor   = 512;
#if 0
  // FIXME---PAH---NOT DONE YET
  if (HobbsCountersInit(pLgMaster))
    return(-2);
#endif
  return(0);
}
void LGMasterFree(struct lg_master *pLgMaster)
{
  if (!pLgMaster)
    return;
  free(pLgMaster->gAFInputBuffer);
  free(pLgMaster->gInputBuffer);
  free(pLgMaster->gRawBuffer);
  free(pLgMaster->theResponseBuffer);
  free(pLgMaster->gDataChunksBuffer);
  free(pLgMaster->gSensorBuffer);
  free(pLgMaster->vers_data.pVersions);
  free(pLgMaster->gScan);
  free(pLgMaster->coarsedata);
  free(pLgMaster->gLsort);
  return;
}
