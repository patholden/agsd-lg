/*
static char rcsid[] = "$Id: QuickCheckManager.c,v 1.15 2007/03/30 18:59:23 pickle Exp pickle $";
*/
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "AppCommon.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "SensorSearch.h"
#include "LaserCmds.h"
#include "QuickCheckManager.h"
#include "Init.h"
#include "ROI.h"
#include "sys/time.h"

#define	kMaxMissedInQuickCheck			3
#define	kQCHistoryLength			1



static	uint16_t	gNumberOfSets=0;
static	unsigned char	gFailureArray[kMaxNumberOfPlies]
				[kNumberOfFlexPoints][kQCHistoryLength];
int16_t  gCurrentQCSensor=0;
uint16_t  gCurrentQC=0;
int16_t  gCurrentQCSet=0;
int     gRealTimeTransform = 0;
int32_t gQCtimer = -1;
uint32_t	gQuickCheckAngles[ kMaxNumberOfPlies * kNumberOfFlexPoints * 2 ];
uint32_t	gRespondToWhom;

int32_t gQuickCheckTargetNumber [ kMaxNumberOfPlies ];

int32_t gQuickFailNumber = kMaxMissedInQuickCheck;

static	void		InitQuickCheckHistory ( void );
static	void		AnalyzeQuickChecks (struct lg_master *pLgMaster, uint32_t respondToWhom );


void PerformAndSendQuickCheck(struct lg_master *pLgMaster, int32_t *anglepairs, uint32_t nTargets )
{
  char localdata[ 1024 ];
  uint32_t *ptrX, *ptrY;
  struct parse_basic_resp *pResp;
  uint16_t lostSensors;
  int theResult;
  uint16_t lostSum, i;

  // Zero out area for response
  pResp = (struct parse_basic_resp *)pLgMaster->theResponseBuffer;
  memset(pResp, 0, sizeof(struct parse_basic_resp));

  // initialize array to invalid angles
  memset(localdata, 0xFF, sizeof(localdata));

  memcpy(localdata, anglepairs, (sizeof(int32_t) * 2 * nTargets));
  lostSensors = 0;
  lostSum = 0;

  ptrX = (uint32_t *)localdata; ptrY = ptrX; ptrY++;
	
  if (pLgMaster->gQCcount == -2)
    theResult = 0;
  else {
    for (i = 0; i < kNumberOfFlexPoints; i++)
      {
	gCurrentQCSensor = i;
	theResult = QuickCheckASensor(pLgMaster, *ptrX, *ptrY );
	if ( theResult )
	  {
	    if ( theResult == kSuperFineNotFound )
	      {
		lostSensors += 1U << i;
		lostSum++;
		theResult = 0;
	      }
	    else break;
	  }
	ptrX++; ptrX++; ptrY++, ptrY++;
      }
  }
  SlowDownAndStop(pLgMaster);

  if (theResult)
      pResp->hdr.status = RESPFAIL;
  else
    {
      if (lostSum >= gQuickFailNumber)
	{
	  pResp->hdr.status = RESPFAIL;
	  pResp->hdr.errtype = lostSensors;
	}
      else
	{
	  pResp->hdr.status = RESPGOOD;
	  pResp->hdr.errtype = lostSensors;
	}
    }
  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), kRespondExtern);
  return;
}

void PerformThresholdQuickCheck ( struct lg_master *pLgMaster,
				  char * data,
				  uint32_t nTargets,
				  uint32_t nThresh)
{
	uint32_t *ptrX, *ptrY;
	uint32_t return_code;
	struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
	uint32_t lostSensors;
	int theResult;
	unsigned short lostSum, i;
        char localdata[ 1024 ];


	// Initialize response buffer area
	memset(pResp, 0, sizeof(struct parse_basic_resp));

	// initialize array to invalid angles
	memset(localdata, 0xFF, sizeof(localdata));

        // Get data from caller
	memcpy( localdata, data, sizeof(int32_t) * 2 * nTargets );

	lostSensors = 0U;
	lostSum = 0;

	ptrX = (uint32_t *)localdata; ptrY = ptrX; ptrY++;
	
	i = 0;
	if (pLgMaster->gQCcount == -2)
	  theResult = 0;
        else
	  {
	    while ( i < nTargets )
	      {
		gCurrentQCSensor = i;
		theResult = QuickCheckASensor (pLgMaster, *ptrX, *ptrY );
		if (theResult)
		  {
		    if ( theResult == kSuperFineNotFound )
		      {
			lostSensors += 1U << i;
			lostSum++;
			theResult = 0;
			*ptrX = 0xFFFFFFFF;
			*ptrY = 0xFFFFFFFF;
			if ( lostSum >= nThresh ) {
			  break;
			}
		      }
		    else break;
		  }
		ptrX++; ptrX++; ptrY++, ptrY++;
		i++;
	      }
	  }
	
        SlowDownAndStop(pLgMaster);
	if (theResult)
	  pResp->hdr.status = RESPFAIL;
	else
	  {
	    if ( lostSum >= gQuickFailNumber )
	      {
		return_code = htonl(kFail | lostSensors);
		memcpy(pResp, &return_code, sizeof(uint32_t));
	      }
	    else
	      {
		return_code = htonl(kOK | lostSensors);
		memcpy(pResp, &return_code, sizeof(uint32_t));
	      }
	  }
	HandleResponse ( pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), kRespondExtern );
	return;
}



/* When you see PostCommand ( pLgMaster, theCommand, data, respondToWhom ) */
/* Call this in PostCommand before displaying a pattern */
/*
	when you are about to display something ( theCommand == kDisplay )
	and if everything checks out OK ( pattern length, etc. etc. ) you also
	should
		SaveAnglesForQuickCheck ( data, respondToWhom );
*/
/* If SaveAnglesForQuickCheck does not return kOK */
/* do not display anything and send the result as a reply */
uint32_t SaveAnglesForQuickCheck (struct displayData* data,
                              uint32_t respondToWhom )
{
	unsigned short	tempNumberOfSets;
	
	gRespondToWhom = respondToWhom;

	tempNumberOfSets = data->numberOfSensorSets;
	if ( tempNumberOfSets > kMaxNumberOfPlies )
		return ( kFail | kTooManyPlies );
		
	gNumberOfSets = tempNumberOfSets;
	
	if ( gNumberOfSets ) {
		memmove ( (char *)gQuickCheckAngles
                        , (char *)data->sensorAngles
                        , (size_t)( gNumberOfSets * kNumberOfFlexPoints * 
                                    2 * sizeof ( uint32_t ) )
                        );
	} else {
		memmove (
		 	 (char *)gQuickCheckAngles,
			 (char *) data->sensorAngles,
			 (size_t)( kNumberOfFlexPoints * 
                                    2 * sizeof ( uint32_t ) ) );
        }
			
	InitQuickCheckHistory (  );
	
	return kOK;
}

int ShouldDoQuickCheck ( void )
{
        int doCheck;
        uint32_t xRaw, yRaw;
        int i;

        doCheck = 0;
        for ( i=0; i<kNumberOfFlexPoints; i++ ) {
            xRaw = gQuickCheckAngles[gCurrentQCSet*2*kNumberOfFlexPoints+2*i  ];
            yRaw = gQuickCheckAngles[gCurrentQCSet*2*kNumberOfFlexPoints+2*i+1];
            if ((xRaw & kMaxUnsigned) || (yRaw & kMaxUnsigned))
	      doCheck = 1;
        }

        return doCheck;
}

void DummyQuickCheck(void)
{
  gCurrentQCSensor++;
  gCurrentQCSensor = gCurrentQCSensor % kNumberOfFlexPoints;
  if (!gCurrentQCSensor)
    {
      gCurrentQCSet++;
      gCurrentQCSet = (gNumberOfSets ? (gCurrentQCSet % gNumberOfSets) : 0 );
      if (!gCurrentQCSet)
	{
	  gCurrentQC++;
	  gCurrentQC = gCurrentQC % kQCHistoryLength;
	}
    }
  return;
}

/* Call this function periodically */
int PerformPeriodicQuickCheck (struct lg_master *pLgMaster)
{
  int result;
  uint32_t xbin, ybin;

  xbin = gQuickCheckAngles
    [ gCurrentQCSet * 2 * kNumberOfFlexPoints
      + 2 * gCurrentQCSensor ];
  ybin = gQuickCheckAngles
    [ gCurrentQCSet * 2 * kNumberOfFlexPoints
      + 2 * gCurrentQCSensor + 1];

  result =  QuickCheckASensor (pLgMaster, xbin, ybin );
  if (result)
    {
      gFailureArray[gCurrentQCSet][gCurrentQCSensor][gCurrentQC]
	= true;
      if ( result != kStopWasDone )
	AnalyzeQuickChecks (pLgMaster, gRespondToWhom );
    }
  else
    gFailureArray[gCurrentQCSet][gCurrentQCSensor][gCurrentQC]
                                                  = false;
  //NOTE: in general, plies will NOT necessarily have kNumberOfFlexPoints	
  gCurrentQCSensor++;
  if ( gCurrentQCSensor == gQuickCheckTargetNumber[gCurrentQCSet] ) {
    gCurrentQCSensor = 0;
  }
  if (gCurrentQCSensor > gQuickCheckTargetNumber[gCurrentQCSet])
    {
      syslog(LOG_ERR, "QuickCheckManager 301 BAD ERROR  %d %d %d", gNumberOfSets,
	      gCurrentQCSensor, gQuickCheckTargetNumber[gCurrentQCSet]);
      gCurrentQCSensor = 0;
    }
  if (!gCurrentQCSensor)
    {
      gCurrentQCSet++;
      gCurrentQCSet = (gNumberOfSets ? (gCurrentQCSet % gNumberOfSets) : 0);
      if (!gCurrentQCSet)
	{
	  gCurrentQC++;
	  gCurrentQC = gCurrentQC % kQCHistoryLength;
	}
    }
  return(result);
}

void InitQuickCheckHistory ( void )
{
	short i, j, k;
	k = kMaxNumberOfPlies;
	while ( k-- )
	{
		i = kNumberOfFlexPoints;
		while ( i-- )
		{
			j = kQCHistoryLength;
			while ( j-- ) gFailureArray[k][i][j] = false;
		}
	}
	gCurrentQCSensor = 0;
	gCurrentQC = 0;
	gCurrentQCSet = 0;
	return;
}

//FIXME---PAH---NEED TO FIX CMD/RESP HERE
void AnalyzeQuickChecks (struct lg_master *pLgMaster, uint32_t respondToWhom )
{
  char *theResponseBuffer=(char *)pLgMaster->theResponseBuffer;
  char *tmpPtr=theResponseBuffer;
  short numberOfBadSensors = 0;
  short numberOfMissesOnASensor, i, j;
  uint32_t tempMsg;
  uint32_t return_code;
  uint16_t ret_16;
  uint16_t byte3=0;

  memset(theResponseBuffer, 0, (sizeof(uint32_t) + (2 * sizeof(uint32_t) + kCRCSize)));
  tempMsg = 0U;

  i = kNumberOfFlexPoints;
  while (i--)
    {
      numberOfMissesOnASensor = 0;
      j = kQCHistoryLength;
      while (j--)
	{
	  if (gFailureArray[gCurrentQCSet][i][j])
	    numberOfMissesOnASensor++;
	}
      if (numberOfMissesOnASensor >= 1)
	{
	  tempMsg += (1U << i);
	  numberOfBadSensors++;
	}
    }
  if (numberOfBadSensors >= gQuickFailNumber)
    {
      byte3 = (0x00FF0000 & tempMsg) >> 16;
      return_code = kQuickCheckPlyFail + byte3;
      memcpy(theResponseBuffer, &return_code, sizeof(return_code));
      tmpPtr = (char *)(theResponseBuffer + sizeof(uint32_t));
      ret_16 = (uint16_t)(gCurrentQCSet + 1);
      memcpy(tmpPtr, &ret_16, sizeof(uint16_t));
      ShortConv((unsigned char *)(tmpPtr));
      tmpPtr = (char *)(theResponseBuffer + sizeof(uint32_t) + sizeof(uint16_t));
      ret_16 = (uint16_t)(0xFFFF & tempMsg);
      memcpy(tmpPtr, &ret_16, sizeof(uint16_t));
      ShortConv((unsigned char *)tmpPtr);
      HandleResponse(pLgMaster, (sizeof(uint32_t) + 2 * sizeof(uint16_t)), kRespondExtern);
    }
}


void DoQCcount ( struct lg_master *pLgMaster, char * data, uint32_t respondToWhom )
{
  int itemp;
  struct parse_basic_resp*pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
  struct parse_quickcheckcount_parms*pInp=(struct parse_quickcheckcount_parms *)data;

  stopQCcounter(pLgMaster);
  resetQCcounter(pLgMaster);

  // Prep buffer for response
  memset(pResp, 0, sizeof(struct parse_basic_resp));
  
  itemp = pInp->inp_qccount;
  if ( itemp <= 0 )
    {
      if ( itemp == -2 )
	pLgMaster->gQCcount = -2;
      else
	pLgMaster->gQCcount = -1;

      SetQCcounter(pLgMaster, pLgMaster->gQCcount);
      gQuickCheck = 0;
      stopQCcounter(pLgMaster);
    }
  else
    {
      pLgMaster->gQCcount = itemp;
      gQuickCheck = 1;
      SetQCcounter(pLgMaster, pLgMaster->gQCcount);
    }

  resetQCcounter(pLgMaster);
  pResp->hdr.status = RESPGOOD;
  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
  return;
}

void DoQCtimer (struct lg_master *pLgMaster, char * data, uint32_t respondToWhom )
{
  int itemp;
  struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
  struct parse_quickchecktime_parms *pInp=(struct parse_quickchecktime_parms *)data;

  stopQCcounter(pLgMaster);

  itemp = pInp->inp_qctime;
  if ( itemp < 0 ) {
    gQCtimer = -1;
    gQuickCheck = 0;
    stopQCcounter(pLgMaster);
  } else {
    gQCtimer = itemp;
    if ( gQCtimer > 1000 ) gQCtimer = 1000;
    gQuickCheck = 1;
  }

 stopQCcounter(pLgMaster);
 pResp->hdr.status = RESPGOOD;
 HandleResponse ( pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
 return;
}
