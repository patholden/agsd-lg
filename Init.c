/*
 * static char rcsid[] = "$Id: Init.c,v 1.12 2001/01/03 17:58:15 ags-sw Exp ags-sw $";
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
#include "LaserInterface.h"
#include "LaserPattern.h"
#include "Init.h"
#include "SensorSearch.h"
#include "QuickCheckManager.h"
#include "Video.h"
#include "Files.h"

void DoReInit( struct lg_master *pLgMaster, uint32_t respondToWhom )
{
  int  return_code=0;

  memset(pLgMaster->theResponseBuffer, 0, sizeof(COMM_RESP_MIN_LEN));
  
  fprintf( stderr, "OPBDoReInit\n" );
  ConfigDataInit(pLgMaster);
  FlashLed(pLgMaster, 5 );

  return_code = kOK;
  memcpy(pLgMaster->theResponseBuffer, &return_code, sizeof(uint32_t));
  HandleResponse(pLgMaster, sizeof(uint32_t), respondToWhom);
  return;
}


int ConfigDataInit(struct lg_master* pLgMaster)
{
  FILE *handle;
  char *localBuff;
  char testStr[256];
  char tmpline[256];
  char *token;
  char *ptr;
  int32_t i;
  uint32_t ufactor;
  uint32_t inaddr;
  long     file_size;
  size_t   length;
  int      count;
  double ArcSteps = 60.0;
  int d1,d2,d3,d4;
  double dTemp;
  uint32_t return_code;
  
  // First check version file
  return_code = InitCheckVersion(pLgMaster);
  if (return_code)
    return(return_code);
  // Now read config file & parse objects
  memset(testStr, 0, sizeof(testStr));
  handle = fopen("/etc/ags/conf/init", "rw");
  if (handle == NULL) { 
    fprintf( stderr, "/etc/ags/conf/init file not found\n" );
    return(-1);
  }

  fseek(handle, 0L, SEEK_END);
  file_size = ftell(handle);
  fseek(handle, 0L, SEEK_SET);   // Reset back to beginning of file
  // Get length of file, get a local buffer, and then read it.
  if (file_size <=0) {
    fprintf( stderr, "/etc/ags/conf/init incorrect file size %lx\n", file_size );
    return(-2);
  }
  localBuff = malloc(file_size);
  if (localBuff <=0)
    {
      fprintf( stderr, "Unable to malloc a local buffer\n" );
      return(-3);
    }
  length = (int)fread((void *)localBuff, (size_t)file_size, 1, handle );
  if (length <=0)
    fprintf( stderr, "initialization file bad size %ld,errno %d\n",(long)file_size,errno);
  else
    fprintf( stderr, "initialization file size %ld\n",(long)file_size);
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

  token = localBuff; 
  count = 0;
  while ((count++ < file_size) && (*token != 0))
    {
      strncpy(tmpline, token, 255);
      for (i = 0; i < 255; i++)
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
	  pLgMaster->dmax = 4294967296.0 / ArcSteps;
	}
      strcpy( testStr, "beamlinearrangex =" );
      if ( strncmp( tmpline, testStr, strlen(testStr) ) == 0 ) {
	sscanf( tmpline, "beamlinearrangex = %lf", &gBeamLinearRangeX ); }
      strcpy( testStr, "beamlinearrangey =" );
      if ( strncmp( tmpline, testStr, strlen(testStr) ) == 0 ) {
	sscanf( tmpline, "beamlinearrangey = %lf", &gBeamLinearRangeY ); }
      strcpy( testStr, "coarsefactor" );
      if ( strncmp( token, testStr, strlen(testStr) ) == 0 ) {
	sscanf( token, "coarsefactor = %d", &gCoarseFactor ); 
	gCoarseSearchStep = (uint32_t)(
				       0xffff0000 &  (gCoarseFactor * 0x00010000));
	gNumberOfSpirals = gSpiralFactor / gCoarseFactor;
      }
      strcpy( testStr, "searchstepperiod =" );
      if ( strncmp( tmpline, testStr, strlen(testStr) ) == 0 ) {
	sscanf( tmpline, "searchstepperiod = %d", &pLgMaster->gSrchStpPeriod ); }
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
	sscanf( token, "multiplesweeps = %d", &gMultipleSweeps );
      }
      strcpy( testStr, "dwell =" );
      if ( strncmp( token, testStr, strlen(testStr) ) == 0 ) {
	sscanf( token, "dwell = %d", &gDwell );
	if ( (gDwell < 10 ) || (gDwell > 1000) ) {
	  gDwell = 0;
	}
      }
      strcpy( testStr, "ipaddress =" );
      if ( strncmp( token, testStr, strlen(testStr) ) == 0 ) {
	sscanf( token, "ipaddress = %d.%d.%d.%d", &d1, &d2, &d3, &d4);
	memset( pLgMaster->webhost, 0, 128 );
	sprintf(pLgMaster->webhost, "%d.%d.%d.%d", d1, d2, d3, d4 );
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
      while ( *token != 0x00 && *token != 0x0a && *token != 0x0d ) {
	token++;
	count++;
      }
      if ( *token == 0x0a || *token == 0x0d ) {
	token++;
	count++;
      }
    }

  if ((inaddr = inet_addr(pLgMaster->webhost)) == INADDR_NONE)
    {
      return(-4);
    }
  else
    free(localBuff);
  return(0);
}
int LGMasterInit(struct lg_master *pLgMaster)
{
  memset((char *)pLgMaster, 0, sizeof(struct lg_master));
  pLgMaster->gInputBuffer = malloc( kMaxDataLength );
  pLgMaster->gRawBuffer = malloc( kMaxDataLength );
  pLgMaster->theResponseBuffer = malloc( kMaxDataLength );
  pLgMaster->gAFInputBuffer = malloc((size_t)(kSizeOfCommand
					      + kMaxParametersLength
					      + kMaxDataLength
					      + kCRCSize ) );
  pLgMaster->gDataChunksBuffer = malloc((size_t)kMaxDataLength);
  pLgMaster->gSensorBuffer = malloc((size_t)
		(kMaxNumberOfPlies * kNumberOfFlexPoints * 2 * sizeof(uint32_t)));
  
  if (!pLgMaster->gInputBuffer || !pLgMaster->gRawBuffer
      || !pLgMaster->theResponseBuffer || !pLgMaster->gAFInputBuffer
      || !pLgMaster->gDataChunksBuffer || !pLgMaster->gSensorBuffer)
    {
      perror("\nMalloc failed for one or more master buffers");
      return(-1);
    }
    
  // Initialize elements here
  pLgMaster->gTolerance = 0.0300;
  pLgMaster->gArgTol = GARGTOL_DEFAULT;
  pLgMaster->gProjectorSerialNumber = -1L;
  pLgMaster->gPeriod  = KETIMER_75U;
  pLgMaster->gSrchStpPeriod = 32;
  pLgMaster->gQCcount = GQCCOUNT_DEFAULT;
  pLgMaster->dmax = 4294967296.0 / 60.0;
  pLgMaster->gCALIBFileOK = false;
#if 0
  // FIXME---PAH---not working yet
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
  return;
}
