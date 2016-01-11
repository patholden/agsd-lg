#include <stdint.h>
//static char rcsid[] = "$Id: Main.c,v 1.29 2001/01/03 18:02:20 ags-sw Exp ags-sw $";

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <signal.h>
#include <math.h>
#include <sys/io.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "AppCommon.h"
#include "SystemMaint.h"
#include "UserMaint.h"
#include "CRCHandler.h"
#include "LaserCmds.h"
#include "LaserInterface.h"
#include "LaserPattern.h"
#include "BoardComm.h"
#include "AngleCorrections.h"
#include "SensorSearch.h"
#include "Init.h"
#include "FOM.h"
#include "Hobbs.h"
#include "asciiID.h"
#include "DoLevelScan.h"
#include "DoCoarseScan.h"
#include "Video.h"
#include "3DTransform.h"
#include "compile.h"
#include "APTParser.h"
#include "DoAutoFocusCmd.h"

const char *ags_banner =
  "AGS version " AGS_COMPILE_TIME  
  "  " AGS_UNIX_TIME "\n"
  AGS_ASCII_ID "\n";


static	int	InitUserStuff (struct lg_master *pLgMaster);
static	int32_t	UserGoAhead ( void );
static void	CloseUserStuff(struct lg_master *pLgMaster);

char * gQCsaveData=0;
uint32_t gQClengthOfData;
unsigned char                   gHeaderSpecialByte = 0;

// This points to all the config data used by this application
// Make it global so debugger can see (like GDB)
// Should consider using sqlite to store all data.....
struct lg_master *pConfigMaster=0;

int  gSearchCurrentSensor;
uint32_t gCoarseSearchStep  = 0x00080000U;
int           gCoarseFactor      = 8;
uint32_t gCoarse2SearchStep = 0x00080000U;
int           gCoarse2Factor     = 8;
int gNumberOfSpirals = 128;
int gSpiralFactor   = 512;
int gHatchFactor    = 100;
int gSuperFineCount = 256;
int gSuperFineSkip = 1;
int32_t gDwell = 0;

int gVideoCount = 0;
int gVideoCheck = 0;

int32_t gQuickCheck = 0;

uint32_t gVideoPreDwell = 0;

extern	int32_t	SystemGoAhead ( void );

int gForceTransform = 0;
int termination_flag=0;

void sigact_handler(void)
{
  termination_flag = 1;
}
static int main_capture_signals(void)
{
  struct   sigaction sigact;

  return(0);
  memset(&sigact, 0, sizeof(struct sigaction));

  sigact.sa_handler = (void *)&sigact_handler;
  sigact.sa_flags  = SA_RESTART;
  sigfillset(&sigact.sa_mask);
  
  if (((sigaction(SIGINT, &sigact, 0)) < 0) || 
      ((sigaction(SIGQUIT, &sigact, 0)) < 0) ||
      ((sigaction(SIGTERM, &sigact, 0)) < 0) ||
      ((sigaction(SIGKILL, &sigact, 0)) < 0))
    return(-1);

  return(0);
}
static void ags_cleanup(struct lg_master *pLgMaster)
{
  // Do housekeeping before exiting program
  CloseUserStuff (pLgMaster );
  if (gQCsaveData)
    free(gQCsaveData);
  if (pLgMaster->socketfd >= 0)
    close(pLgMaster->socketfd);
  if (pLgMaster->datafd >= 0)
    close(pLgMaster->datafd);
  pLgMaster->socketfd = -1;
  pLgMaster->datafd = -1;
  LGMasterFree(pLgMaster);
  free(pLgMaster);
  return;
}
int main ( int argc, char **argv )
{
  int32_t i = 0;
  int     error=0;
  
  fprintf( stderr, "%s", ags_banner );

  /* defaults */
  gBeamLinearRangeX = GBEAMLNRNG_DEF;	
  gBeamLinearRangeY = GBEAMLNRNG_DEF;	


  gSlowExponent = 8;
  gFastExponent = 11;
  gSlowBabies = 1U << gSlowExponent;
  gFastBabies = 1U << gFastExponent;
  gDwell      = 0;
  gFOM        = -1;
  gCentroid   = 1;
  gMaxPiledPts= 9;
  gSearchFlag = 0;
  gBestTargetNumber = 0;
  for ( i=0; i<128; i++ ) {
    gBestTargetArray[i] = 0;
  }
//
//  try board initialization here
//
  pConfigMaster = malloc(sizeof(struct lg_master));
  if (!pConfigMaster)
    {
      fprintf(stderr,"LG_MASTER_MALLOC Failed\n");
      exit(EXIT_FAILURE);
    }
  error = LGMasterInit(pConfigMaster);
  if (error)
    {
      fprintf(stderr,"\nMain: Can't initialize master struct, err %d",error);
      free(pConfigMaster);
      exit(EXIT_FAILURE);
    }
  error = InitBoard(pConfigMaster );
  if (error)
    {
      fprintf(stderr,"\nMain: Can't start board laser-fd %d, err %d, errno %d",pConfigMaster->fd_laser, error, errno);
      free(pConfigMaster);
      exit(EXIT_FAILURE);
    }
  fprintf(stderr,"\nMAIN: INITBOARD /dev/laser, fd %d",pConfigMaster->fd_laser);
  error = ConfigDataInit(pConfigMaster);
  if (error)
    {
      fprintf(stderr,"\nConfigDataInit: Can't get config data, err %d", error);
      free(pConfigMaster);
      exit(EXIT_FAILURE);
    }

  error = CommInit(pConfigMaster);
  if (error)
    {
      perror("\nMain: Can't initialize Ethernet/Serial Interfaces");
#ifdef PATDEBUG
      fprintf(stderr,"COMMINIT Failed err %x\n", error);
#endif
      free(pConfigMaster);
      exit(EXIT_FAILURE);
    }
	    
  if (pConfigMaster->gHEX == 0)
    fprintf(stderr, "old binary serial I/O\n");
  else if (pConfigMaster->gHEX == 1)
    fprintf(stderr, "limited HEX serial I/O in ASCII hex\n");
  else if (pConfigMaster->gHEX == 2)
    fprintf( stderr, "full HEX serial I/O in ASCII hex\n" );

  fprintf(stderr, "tol %f", pConfigMaster->gArgTol);
  fprintf( stderr, " period %d",  pConfigMaster->gPeriod );
  fprintf( stderr, " QC %d",  pConfigMaster->gQCcount );
  fprintf( stderr, " X %f",  gBeamLinearRangeX );
  fprintf( stderr, " Y %f",  gBeamLinearRangeY );
  fprintf( stderr, " baud %d\n",  GBAUD_DEFAULT);
  fprintf( stderr, "   gSearchStepPeriod %d", pConfigMaster->gSrchStpPeriod);
  fprintf( stderr, "   gTargetDrift %d\n",  gTargetDrift );
  fprintf( stderr, " coarse step 0x%x",   gCoarseSearchStep );
  fprintf( stderr, " spirals %d",   gNumberOfSpirals );
  fprintf( stderr, " factor %d",   gCoarseFactor );
  fprintf( stderr, " attempt %d\n",   gNumberOfSensorSearchAttempts );
  fprintf( stderr, " triggerlevel %d\n",           gDELLEV );
  fprintf( stderr, " factor spiral %d",   gSpiralFactor );
  fprintf( stderr, " hatch %d\n",           gHatchFactor );
  fprintf( stderr, " SuperFineCount %d",   gSuperFineCount );
  fprintf( stderr, " SuperFineSkip %d",   gSuperFineSkip );
  fprintf( stderr, " SuperFineFactor %d\n",   gSuperFineFactor );
  fprintf( stderr, "fast %x",   gFastBabies );
  fprintf( stderr, " (expo %d), ",       gFastExponent );
  fprintf( stderr, "slow %x",   gSlowBabies );
  fprintf( stderr, " (expo %d) ",       gSlowExponent );
  fprintf( stderr, "dwell %d ",   gDwell );
  fprintf( stderr, "gCentroid %d \n",   gCentroid );
  fprintf( stderr, "MaxPiledPts %d ",   gMaxPiledPts );
  fprintf( stderr, "MaxCos %f ",   gMaxCos );
  fprintf( stderr, "LongToShort %f \n",   gLongToShort );
  fprintf( stderr, "CurveInterpolation %f ",   gCurveMin );
  fprintf( stderr, "QuickCheck %d ",   gQuickCheck );
  fprintf( stderr, "MaxQuickSearches %d \n", gMaxQuickSearches);
  fprintf( stderr, "MaxDiffMag %f ",   gMaxDiffMag );
  fprintf( stderr, "MultipleSweeps %d ",   gMultipleSweeps );
  fprintf( stderr, "webhostIP %s \n", pConfigMaster->webhost );

  ClearLinkLED(pConfigMaster);
  StopPulse(pConfigMaster); 
  if ((InitUserStuff (pConfigMaster)) < 0)
    {
      perror("\nINIT: Malloc failure");
      ags_cleanup(pConfigMaster);
      exit(EXIT_FAILURE);
    }
  SlowDownAndStop (pConfigMaster);
  SetQCcounter(pConfigMaster, 0);
  SearchBeamOff(pConfigMaster);

  // Set up program to capter user-entered signals
  if ((main_capture_signals()) < 0)
    {
      perror("\nSIGACT failure");
      ags_cleanup(pConfigMaster);
      exit(EXIT_FAILURE);
    }
  // The main loop should never really exit.
  fprintf(stderr, "\nWaiting to Accept packets from %s", pConfigMaster->webhost);
  while (SystemGoAhead() && UserGoAhead())
    {
      if (pConfigMaster->serial_ether_flag == 2)
	{
	  error = DoProcEnetPackets(pConfigMaster);
	  if (error >= 0)
	    continue;
	  if (error == -1)
	    {
	      if (pConfigMaster->enet_retry_count != COMM_MAX_ENET_RETRIES)
		{
		  fprintf(stderr, "\nCOMMLOOP: Read data failure, err = %x, try to re-open", error);
		  // Try to open Comms interface to PC host
		  if ((CommConfigSockfd(pConfigMaster)) < 0)
		    {
		      fprintf( stderr, "\nCOMMLOOP: Unable to restart Comms, shutting down\n");
		      return(error);
		    }
		}
	      else
		{
		  fprintf( stderr, "\nCOMMLOOP: Exceeded Comm restart retry count, shutting down\n");
		  ags_cleanup(pConfigMaster);
		  exit(EXIT_FAILURE);
		}
	    }
	}
    }

  FlashLed(pConfigMaster, 7);
  // Clean up AGS specific stuff before exiting
  fprintf( stderr, "\nShutting down do to System or User interrupt\n");
  ags_cleanup(pConfigMaster);
  exit(EXIT_SUCCESS);
}

int InitUserStuff (struct lg_master *pLgMaster)
{
  gQCsaveData = (char *)malloc(
                (size_t)( kSizeOfCommand
                         + kMaxParametersLength
                         + kMaxDataLength
                         + kCRCSize ) );
  if (!gQCsaveData)
    return(-1);
  
  FlashLed(pLgMaster, 3);
  LedBoot(pLgMaster);
  SearchBeamOff(pLgMaster);
  InitCRCHandler (  );
  InitLaserInterface (pLgMaster);
  InitLaserCommands (  );
  if (InitLaserPattern())
    return(-1);
  InitAPTParser (  );
  InitSensorSearch (  );
  InitLevelScan( );
  InitCoarseScan( );
  return(0);
}

int32_t UserGoAhead ( void )
{
  // Check for user-input trying to kill this program
  if (termination_flag)   // Check for Ctrl-C, do proper shut down
    return(0);

  return(1);
}

static void CloseUserStuff (struct lg_master *pLgMaster)
{

/*	Close_HAL_CELL_Interface (  ); */
        CloseSensorSearch (  );
        CloseAPTParser (  );
        CloseLaserPattern (  );
        CloseLaserCommands (  );
	CloseLaserInterface (  );
	CloseCRCHandler (  );
        ReleaseBoard(pLgMaster);
}
