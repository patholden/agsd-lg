/*
static char rcsid[] = "$Id$";

*/
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include "BoardComm.h"
#include "AppCommon.h"
#include "SensorSearch.h"
#include "QuickCheckManager.h"
#include "Protocol.h"
#include "comm_loop.h"
#include "DoCoarseScan.h"
#include "DoCoarseScan2.h"
#include "laserguide.h"

#define NZERO 5

int     gNumberOfSensorSearchAttempts  = 2;

int iflag = 0;
int itemp;
int gNoFine = 1;
uint32_t minlevel = 70;
int firstCoarse = 0;
int firstLeg = 1;
int gSuperIndex = 0;
int32_t * gXsuperSave;
int32_t * gYsuperSave;
int gXsuperCount;
int gYsuperCount;

int * gSaveMatch;
int * gSaveSweep;
int * gSaveLout1;
int * gSaveLout2;
double * gSaveDblX;
double * gSaveDblY;
double * gSaveDblX1;
double * gSaveDblX2;
double * gSaveDblY1;
double * gSaveDblY2;
char ** gSuperReturn;
int    gLoutIndex = 0;
int32_t   gLoutCount = 0;
int32_t   * gLoutTargetCount;
int32_t   gLoutSize = 0;

char * gLoutBase;

char * gLoutPtr;

int LongOrShortThrowSearch = 0;

int gMaxQuickSearches = 3;

int gMultipleSweeps = 0;

int
int32_tsort( const void *elem1, const void *elem2 );

static int
findFirstLast(struct lg_master *pLgMaster,
	      uint32_t *firstX,
	      uint32_t *lastX,
	      uint32_t *firstY,
	      uint32_t *lastY,
	      int32_t *medianX,
	      int32_t *medianY,
	      uint32_t currentX,
	      uint32_t currentY,
	      uint32_t xStep,
	      uint32_t yStep,
	      uint32_t nSteps);

static int DoFineLevel ( struct lg_master *pLgMaster,
			 uint32_t *foundX
			 , uint32_t *foundY
			 , double * outMinX
			 , double * outMinY
			 , double * outMaxX
			 , double * outMaxY
			 )
;

static int
DoSuperLevelSearch ( struct lg_master *pLgMaster,
		     uint32_t *foundX,
		     uint32_t *foundY,
		     double * dXmin,
		     double * dYmin,
		     double * dXmax,
		     double * dYmax);

static int        
DoRedundantSuperFineSearch (struct lg_master *pLgMaster,
			    uint32_t *foundX,
			    uint32_t *foundY,
			    double * dMinX,
			    double * dMinY,
			    double * dMaxX,
			    double * dMaxY);

static int SuperSearch (struct lg_master *pLgMaster,
			uint32_t *foundX
			, uint32_t *foundY
			, double * dMinX
			, double * dMinY
			, double * dMaxX
			, double * dMaxY
			);

void
limitCalc( uint32_t centerX
         , uint32_t centerY
         , uint32_t *eolXNeg
         , uint32_t *eolXPos
         , uint32_t *eolYNeg
         , uint32_t *eolYPos
         );

static int  CoarseLeg(struct lg_master *pLgMaster,
		      uint32_t X,    uint32_t Y,
		      uint32_t delX, uint32_t delY,
		      uint32_t nSteps,
		      uint32_t * foundX, uint32_t * foundY );
static int  FakeLeg( struct lg_master *pLgMaster, uint32_t X,    uint32_t Y,
                    uint32_t delX, uint32_t delY,
                    uint32_t nSteps,
                    uint32_t * foundX, uint32_t * foundY );

static int DoQuickFineSearch(struct lg_master *pLgMaster, uint32_t *foundX, uint32_t *foundY );

int32_t gQCerrors[6] = {0,0,0,0,0,0};

#define kMaxOutLength   0x80000

int32_t gDELLEV =  60;

unsigned char * gOut;
uint32_t * gLout;
uint32_t * gLout1;
uint32_t * gLout2;
uint32_t * gLsort;

int gTargetDrift = 0;
double g_aX = 0.0;
double g_bX = 1.0;
double g_aY = 0.0;
double g_bY = 1.0;

double gFixX[ kNumberDrift ];
double gFixY[ kNumberDrift ];
double gDriftX[ kNumberDrift ];
double gDriftY[ kNumberDrift ];

uint32_t * xPosSave;
uint32_t * xNegSave;
uint32_t * yPosSave;
uint32_t * yNegSave;

uint32_t * xPosEdge;
uint32_t * xNegEdge;
uint32_t * yPosEdge;
uint32_t * yNegEdge;


uint32_t * xPosSuper;
uint32_t * xNegSuper;
uint32_t * yPosSuper;
uint32_t * yNegSuper;

uint32_t * xPosESuper;
uint32_t * xNegESuper;
uint32_t * yPosESuper;
uint32_t * yNegESuper;

uint32_t * xPosPosition;
uint32_t * yPosPosition;
uint32_t * xNegPosition;
uint32_t * yNegPosition;

double * xpResArr;
double * xnResArr;
double * ypResArr;
double * ynResArr;

uint32_t  xPosESum;
uint32_t  xNegESum;
uint32_t  yPosESum;
uint32_t  yNegESum;

uint32_t  XpEdge;
uint32_t  XnEdge;
uint32_t  YpEdge;
uint32_t  YnEdge;

int compare ( const void * a, const void * b );

double xPosAvg;
double xNegAvg;
double yPosAvg;
double yNegAvg;

int gCentroid;

#define kMask                                              0xFFFFC000U

#define kMaxNumberX                                        0x7FFFC000U
#define kMinNumberX                                        0x80000000U
#define kMaxNumberY                                        0x7FFFC000U
#define kMinNumberY                                        0x80000000U

#define kBlankBit                                        0x00000040U

#define        kLevelBit                        0x20  /*  level trigger */


/*
 * #define kNumberOfSpirals                              512 
 * #define kCoarseSearchStep                                0x00020000U
 */


// #define kFineSearchStep                                  0x00010000U
// #define kFineSearchSpanSteps                              150
// #define kSuperFineSearchStep                             0x00010000U
// #define kSuperFineSearchSpanSteps                         256

#define kFineSearchStep                                  0x00020000U
#define kFineSearchSpanSteps                                70
// #define kSuperFineSearchStep                             0x00010000U
#define kSuperFineSearchStep                             0x00008000U
uint32_t gSuperFineSearchStep = 0x00008000U;
uint32_t gSuperFineFactor = 1;
#define kSuperFineSearchSpanSteps                         256


/*
 * #define kSuperFineSearchSpanSteps                         150
 * #define kFineSearchSpanSteps                              150
 */

/* #define kSuperFineSearchSpanSteps        512 */
/* #define kFineSearchSpanSteps             256 */


#define kNumberOfCrossesToAverage                        7

/* #define kNumberOfCrosses                                30 */
#define kNumberOfCrosses                                7

/* #define kNumberOfSuperCrossesToAverage        65 */
#define kNumberOfSuperCrossesToAverage        2

int QuickCheckASensor(struct lg_master *pLgMaster, uint32_t centerX, uint32_t centerY)
{
        int theResult;
        int i;
	int nTries;
        int nSuccess;
        int nSteps;
        uint32_t tempX, tempY;

        if ( ( centerX == 0xFFFFFFFFU ) ||
                ( centerY == 0xFFFFFFFFU ) ) return 0;

        tempX = centerX;
        tempY = centerY;

        nSteps = kSuperFineSearchSpanSteps;
        for( i = 0; i<nSteps; i++ ) {
           xPosESuper[i] = 0;
           xNegESuper[i] = 0;
           yPosESuper[i] = 0;
           yNegESuper[i] = 0;
        }

        SearchBeamOff (pLgMaster);
        move_dark(pLgMaster, centerX, centerY );
        SearchBeamOn(pLgMaster);
#ifdef DEBUG
	GoToRaw(pLgMaster, &centerX, &centerY );
        usleep( 1000000U );
#endif

#ifdef SZDEBUG
    fprintf( stderr, "QCAS centerX/Y %10x %10x\n", centerX, centerY );
#endif

/*
 *  a more "robusty" quick check
 *    - certain percentage of quick checks must succeed (desire 30%)
 *    - minimum of 2 quick checks must succeed
 *    try up to 30 times.
 */
#ifdef FINESEARCH
        theResult = DoFineSearch( &tempX, &tempY );
        if(theResult == kStopWasDone) {
                   SearchBeamOff(pLgMaster);
                   return theResult;
        }
#endif
        tempX = centerX;
        tempY = centerY;
        if( gTargetDrift ) {
            CorrectDrift( centerX, centerY, &tempX, &tempY);
        }

        nTries  = 0;
	nSuccess = 0;
	while( nTries < gMaxQuickSearches ) {
		nTries++;
        	tempX = centerX;
        	tempY = centerY;
        	theResult = DoQuickFineSearch (pLgMaster, &tempX, &tempY );
                if(theResult == kStopWasDone) {
        		SearchBeamOff(pLgMaster);
                        return theResult;
                }
		
		if( theResult == 0 ) { nSuccess++; }
		if( (nSuccess >= 1) ) {
		    theResult = 0;
                    return(0);
		} else {
		    theResult = kSuperFineNotFound;
		}
        }
        
        SearchBeamOff(pLgMaster);
        return theResult;
}

int QuickCheckOne(struct lg_master *pLgMaster,
		  uint32_t centerX,
		  uint32_t centerY,
		  uint32_t * foundX,
		  uint32_t * foundY
                  )
{
        int theResult;
        int i;
	int nTries;
        int nSuccess;
        int nSteps;
        uint32_t tempX, tempY;

        if ( ( centerX == 0xFFFFFFFFU ) ||
                ( centerY == 0xFFFFFFFFU ) ) return 0;

        tempX = centerX;
        tempY = centerY;

        nSteps = kSuperFineSearchSpanSteps;
        for( i = 0; i<nSteps; i++ ) {
           xPosESuper[i] = 0;
           xNegESuper[i] = 0;
           yPosESuper[i] = 0;
           yNegESuper[i] = 0;
        }

        SearchBeamOff (pLgMaster);
        move_dark(pLgMaster, centerX, centerY );
        SearchBeamOn  (pLgMaster);
#ifdef DEBUG
	GoToRaw(pLgMaster, &centerX, &centerY );
        usleep( 1000000U );
#endif

/*
 *  a more "robusty" quick check
 *    - certain percentage of quick checks must succeed (desire 30%)
 *    - minimum of 2 quick checks must succeed
 *    try up to 30 times.
 */
        tempX = centerX;
        tempY = centerY;

        nTries  = 0;
	nSuccess = 0;
	// while( nTries < 30 ) {
	while( nTries < gMaxQuickSearches ) {
		nTries++;
        	tempX = centerX;
        	tempY = centerY;
        	theResult = DoQuickFineSearch(pLgMaster, &tempX, &tempY );

                if(theResult == kStopWasDone) {
        		SearchBeamOff (pLgMaster);
                        return theResult;
                }
		
		if( theResult == 0 ) { nSuccess++; }
		if( (nSuccess >= 1) ) {
		    theResult = 0;
		    return(0);
		} else {
		    theResult = kSuperFineNotFound;
		}
        }
        
        SearchBeamOff(pLgMaster);
        *foundX = tempX;
        *foundY = tempY;
        return theResult;
}


int SearchForASensor (struct lg_master *pLgMaster,
        uint32_t startX, uint32_t startY,
        uint32_t *foundX, uint32_t *foundY )
{
        uint32_t f1x, f1y, f2x, f2y;
        uint32_t tempX, tempY;
        uint32_t currentX, currentY;
        uint32_t eolXPos, eolYPos;
        uint32_t eolXNeg, eolYNeg;
        uint32_t posStepSize, negStepSize;
        uint32_t xSteps, ySteps;

        uint32_t delX, delY, centX, centY;
        uint32_t nSteps;
        uint32_t avgX, avgY;

	int32_t HatchCount;

        int32_t i;
        int int32_tFactor;
        int theResult;
        double dMinX;
        double dMinY;
        double dMaxX;
        double dMaxY;

        
        gSearchFlag = -1;
        firstCoarse = 1;
#ifdef SZDEBUG
        fprintf(stderr,"476 SearchForASensor gSearchFlag = %d\n", gSearchFlag );
        fprintf( stderr, "477 Search start x,y %x %x\n", startX, startY );
#endif

        currentX = startX & kMask;
        currentY = startY & kMask;

#ifdef SZDEBUG
        fprintf( stderr, "477 Search current x,y %x %x\n", currentX, currentY );
#endif

        move_dark(pLgMaster, currentX, currentY );
        *foundX = 0x00000000U;
        *foundY = 0x00000000U;
        
        eolXPos = currentX;
        eolYPos = currentY;

        eolXNeg = currentX;
        eolYNeg = currentY;
        
        
                
        SearchBeamOn(pLgMaster);

        OnHWflag(pLgMaster);

        pLgMaster->gHeaderSpecialByte = 0x40;
        if ( LongOrShortThrowSearch == 1 ) {
              for ( int32_tFactor = 1; int32_tFactor <= 4 ; int32_tFactor *= 2 ) {
                  minlevel = gDELLEV;
                  theResult = DoCoarseScan(pLgMaster, currentX
                                      , currentY
                                      , 0x40000
                                      , 32 * int32_tFactor
                                      , &f1x
                                      , &f1y
                                      );
                  if(theResult == kStopWasDone) {
                       SearchBeamOff(pLgMaster);
                       return theResult;
                  }
                  if(theResult == kCoarseNotFound) {
                       continue;
                  }
		  theResult = DoCoarseScan2(pLgMaster, currentX
					    , currentY
					    , 0x40000
					    , 32 * int32_tFactor
					    , &f2x
					    , &f2y);
                  if(theResult == kStopWasDone) {
                       SearchBeamOff(pLgMaster);
                       return theResult;
                  }
                  if(theResult == kCoarseNotFound) {
                       continue;
                  }
#ifdef SZDEBUG
        fprintf( stderr, "47a DoCoarseScan2 result %x\n", theResult );
        fprintf( stderr, "47b DoCoarseScan2 xy %8x %8x\n", f2x, f2y );
#endif
                  avgX = f2x;
                  avgY = f1y;
#ifdef SZDEBUG
        fprintf( stderr, "DoCoarseScan avg xy %8x %8x\n", avgX, avgY );
#endif
	theResult = DoFineLevel( pLgMaster, &avgX, &avgY, &dMinX, &dMinY, &dMaxX, &dMaxY);
#ifdef SZDEBUG
	fprintf( stderr, "DoFine  %x \n", theResult );
#endif
	if(theResult == kStopWasDone) {
	  SearchBeamOff(pLgMaster);
	  return theResult;
	}
	if(theResult == kFineNotFound) {
	  continue;
	}
#ifdef SZDEBUG
	fprintf( stderr, "about to DoRedundantSuper  %x %x\n", avgX, avgY );
#endif
	theResult = DoRedundantSuperFineSearch ( pLgMaster, &avgX
						 , &avgY
						 , &dMinX
						 , &dMinY
						 , &dMaxX
						 , &dMaxY
						 );
#ifdef SZDEBUG
	fprintf( stderr, "DoSuper  %x \n", theResult );
#endif
	if(theResult == kStopWasDone) {
	  SearchBeamOff(pLgMaster);
	  return theResult;
	}
	if(theResult == kSuperFineNotFound) {
	  continue;
	}
	if ( theResult == 0 ) {
	  break;
	}
	
              }
              if ( theResult ) return theResult;
              *foundX = avgX;
              *foundY = avgY;
	      
              return 0;
        }
#if SZDEBUG
    fprintf( stderr
           , "508 SearchForASensor gCoarseSearchStep %x  (2) %x\n"
           , gCoarseSearchStep
           , gCoarse2SearchStep
           );
#endif

	HatchCount       =  gHatchFactor  / gCoarse2Factor;
	// gNumberOfSpirals = 1024 / gCoarse2Factor;
	// gNumberOfSpirals =  512 / gCoarse2Factor;
	gNumberOfSpirals =  gSpiralFactor / gCoarse2Factor;

        theResult = kCoarseNotFound;
        if ( gCentroid == 1 ) {
          if ( (int32_t)currentX >=
		  (int32_t)(kMaxNumberX - HatchCount*gCoarse2SearchStep) ) {
             eolXPos   = kMaxNumberX;
          } else {  eolXPos =  currentX + HatchCount*gCoarse2SearchStep; }
          if ( (int32_t)currentX <=
		  (int32_t)(kMinNumberX + HatchCount*gCoarse2SearchStep) ) {
             eolXNeg   = kMinNumberX;
          } else {  eolXNeg =  currentX - HatchCount*gCoarse2SearchStep; }
          if ( (int32_t)currentY >=
		  (int32_t)(kMaxNumberY - HatchCount*gCoarse2SearchStep) ) {
             eolYPos   =  kMaxNumberY;
          } else {  eolYPos =  currentY + HatchCount*gCoarse2SearchStep; }
          if ( (int32_t)currentY <= 
		  (int32_t)(kMinNumberY + HatchCount*gCoarse2SearchStep) ) {
             eolYNeg   =  kMinNumberY;
          } else {  eolYNeg =  currentY - HatchCount*gCoarse2SearchStep; }

          i = 0;
          // do a fake search to see if there is a false trigger problem 
                tempX = currentX + (((i&1)<<1)-1)*(i>>1)*gCoarse2SearchStep;
                tempY = currentY + (((i&1)<<1)-1)*(i>>1)*gCoarse2SearchStep;

                posStepSize = (uint32_t)( (int32_t)gCoarse2SearchStep );
                negStepSize = (uint32_t)(-(int32_t)gCoarse2SearchStep );
                xSteps = (eolXPos - eolXNeg) / posStepSize;
                ySteps = (eolYPos - eolYNeg) / posStepSize;
#ifdef ZPAUSE
if ( iflag == 0 ) {
printf( "Search 563 FakeLeg Pause: " );
itemp=getchar();
if ( itemp == 'q' || itemp == 'Q' ) { iflag = 1; }
}
#endif
#ifdef SZDEBUG
printf( "Search 573 FakeLeg\n" );
#endif

 theResult = FakeLeg(pLgMaster, eolXNeg, tempY, posStepSize, 0, xSteps,
		     foundX, foundY );
 if(theResult == kStopWasDone) {
   SearchBeamOff(pLgMaster);
   return theResult;
 }
#ifdef ZPAUSE
if ( iflag == 0 ) {
printf( "Search 575 FakeLeg Pause: " );
itemp=getchar();
if ( itemp == 'q' || itemp == 'Q' ) { iflag = 1; }
}
#endif
#ifdef SZDEBUG
printf( "Search 586 FakeLeg\n" );
#endif
 theResult = FakeLeg(pLgMaster, tempX, eolYNeg, 0, posStepSize, ySteps,
		     foundX, foundY );
 if(theResult == kStopWasDone) {
   SearchBeamOff(pLgMaster);
   return theResult;
 }
 // end of fake search
 while ( (++i < (2*(HatchCount-2)))  &&  theResult )
   {
     tempX = currentX + (((i&1)<<1)-1)*(i>>1)*gCoarse2SearchStep;
     tempY = currentY + (((i&1)<<1)-1)*(i>>1)*gCoarse2SearchStep;
     posStepSize = (uint32_t)( (int32_t)gCoarse2SearchStep );
     negStepSize = (uint32_t)(-(int32_t)gCoarse2SearchStep );
     xSteps = (eolXPos - eolXNeg) / posStepSize;
     ySteps = (eolYPos - eolYNeg) / posStepSize;

#ifdef ZPAUSE
     if ( iflag == 0 ) {
       printf( "Search 575 CoarseLeg Pause: " );
       itemp=getchar();
       if ( itemp == 'q' || itemp == 'Q' ) { iflag = 1; }
     }
#endif
#ifdef SZDEBUG
     printf( "Search 610 CoarseLeg\n" );
#endif
     theResult = CoarseLeg(pLgMaster, eolXNeg, tempY, posStepSize, 0, xSteps,
			   foundX, foundY );
#ifdef SZDEBUG
     printf( "Search 615 CoarseLeg\n" );
#endif
                if(theResult == kStopWasDone) {
                       SearchBeamOff(pLgMaster);
                       return theResult;
                }
                if(theResult == 0) {break;}
                theResult = CoarseLeg(pLgMaster, tempX, eolYNeg, 0, posStepSize, ySteps,
                                foundX, foundY );
                if(theResult == kStopWasDone) {
                       SearchBeamOff(pLgMaster);
                       return theResult;
                }
                if(theResult == 0) {break;}
          }
          i = gNumberOfSpirals - (HatchCount/2);
        } else {
          i = gNumberOfSpirals;
        }

        while ( i-- && theResult)
        {
                if (IfStopThenStopAndNeg1Else0(pLgMaster)) {
                     gSearchFlag = 0;
#ifdef SZDEBUG
        fprintf( stderr, "568 SearchForASensor gSearchFlag = %d\n", gSearchFlag );
#endif
                     SearchBeamOff(pLgMaster);
                     return kStopWasDone;
                }

                if( (int32_t)eolXPos >= (int32_t)(kMaxNumberX - gCoarse2SearchStep) ) {
                     eolXPos  = kMaxNumberX;
                } else { eolXPos += gCoarse2SearchStep; }
                if( (int32_t)eolXNeg <= (int32_t)(kMinNumberX + gCoarse2SearchStep) ) {
                     eolXNeg  = kMinNumberX;
                } else { eolXNeg -= gCoarse2SearchStep; }

                if( (int32_t)eolYPos >= (int32_t)(kMaxNumberY - gCoarse2SearchStep) ) {
                     eolYPos  = kMaxNumberY;
                } else { eolYPos += gCoarse2SearchStep; }
                if( (int32_t)eolYNeg <= (int32_t)(kMinNumberY + gCoarse2SearchStep) ) {
                     eolYNeg  = kMinNumberY;
                } else { eolYNeg -= gCoarse2SearchStep; }

                posStepSize = (uint32_t)( (int32_t)gCoarse2SearchStep );
                negStepSize = (uint32_t)(-(int32_t)gCoarse2SearchStep );
                xSteps = (eolXPos - eolXNeg) / posStepSize;
                ySteps = (eolYPos - eolYNeg) / posStepSize;

                theResult = CoarseLeg(pLgMaster, eolXNeg, eolYNeg, posStepSize, 0, xSteps,
                                foundX, foundY );
                if(theResult == kStopWasDone) {
                       SearchBeamOff(pLgMaster);
                       return theResult;
                }
                if(theResult == 0) {break;}
                theResult = CoarseLeg(pLgMaster, eolXPos, eolYNeg, 0, posStepSize, ySteps,
                                foundX, foundY );
                if(theResult == kStopWasDone) {
                       SearchBeamOff(pLgMaster);
                       return theResult;
                }
                if(theResult == 0) {break;}
                theResult = CoarseLeg(pLgMaster, eolXPos, eolYPos, negStepSize, 0, xSteps,
                                foundX, foundY );
                if(theResult == kStopWasDone) {
                       SearchBeamOff(pLgMaster);
                       return theResult;
                }
                if(theResult == 0) {break;}
                theResult = CoarseLeg(pLgMaster, eolXNeg, eolYPos, 0, negStepSize, ySteps,
                                foundX, foundY );
                if(theResult == kStopWasDone) {
                       SearchBeamOff(pLgMaster);
                       return theResult;
                }
                if(theResult == 0) {break;}
        }
        OffHWflag(pLgMaster);
        if( gDwell > 0 ) {
           centX = *foundX | 0x40;
           centY = *foundY;
           for( i=0; i<gDwell; i++) {
              delY = 8*kSuperFineSearchStep;
              delX = 8*kSuperFineSearchStep;
              nSteps =  32; 
              centX = *foundX;
              centY = *foundY - 64*delY;
	      if ( DoLineSearch(pLgMaster, centX, centY, 0, delY, nSteps, gOut ) ) {
                   SearchBeamOff(pLgMaster);
                   return kStopWasDone;
              }
              centX = *foundX - 64*delX;
              centY = *foundY;
              if ( DoLineSearch(pLgMaster, centX, centY, delX, 0, nSteps, gOut ) ) {
                   SearchBeamOff(pLgMaster);
                   return kStopWasDone;
              }
           }
        }
        SearchBeamOff(pLgMaster);
        centX = *foundX & 0x00;
        centY = *foundY;
        move_dark(pLgMaster, centX, centY );

        gSearchFlag = 0;
#ifdef SZDEBUG
        fprintf( stderr, "635 SearchForASensor gSearchFlag = %d\n", gSearchFlag );
#endif

        return theResult;
}

static int CoarseLeg(struct lg_master *pLgMaster, uint32_t Xin, uint32_t Yin,
	       uint32_t delX, uint32_t delY, uint32_t nStepsIn,
	       uint32_t * foundX, uint32_t * foundY )
{
        uint32_t tmpX, tmpY;
        uint32_t eolX, eolY;
        uint32_t negX, negY;
        uint32_t avgX, avgY;
        uint32_t *ptr;
        uint32_t X;
        uint32_t Y;
        uint32_t nSteps;
        double dMinX;
        double dMinY;
        double dMaxX;
        double dMaxY;
        double sumX, sumY, count;
        int32_t i;
        int theResult;

        theResult = kCoarseNotFound;

        X = Xin;
        Y = Yin;
        nSteps = nStepsIn;

	SetHWtrigger(pLgMaster);
        i = 0; ptr  = gLout;
        while( i < nSteps ) {
                  *ptr = 0; ptr++; i++;
        }
        if ( DoLevelSearch(pLgMaster, X, Y, delX, delY, nSteps, gLout ) ) {
	  return kStopWasDone;
        }
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;
        /* 
         * gLout[0] = 0;
         * gLout[1] = 0;
         * gLout[2] = 0;
         * gLout[3] = 0;
         * gLout[4] = 0;
         * gLout[5] = 0;
         * gLout[6] = 0;
         * gLout[7] = 0;
         * gLout[8] = 0;
         * gLout[9] = 0;
         */
        ptr  = &(gLout[0]);
        memset( gLsort,   0, nSteps*sizeof(uint32_t) );
        for ( i = 0 ; i < nSteps ; i++ ) {
             gLsort[i] = gLout[i] ;
        }
        qsort( gLsort, nSteps, sizeof(uint32_t), int32_tsort );
#ifdef ZZZDEBUG
        i = 0;
        while( i < nSteps ) {
fprintf( stderr, "SensorSearch i %d  gLs %x out %x\n", i, gLsort[i], gLout[i] );
            i++;
        }
#endif
        minlevel = gDELLEV;

        ptr  = &(gLout[0]);
        count = 0.0;
        sumX  = 0.0;
        sumY  = 0.0;
        for ( i = 0; i < 10; i++ ) {
            tmpX = X + i * delX;
            tmpY = Y + i * delY;
            if ( gLout[i] >= minlevel ) {
                       sumX += (double)((int32_t)tmpX);
                       sumY += (double)((int32_t)tmpY);
                       count += 1.0;
#ifdef SZDEBUG
fprintf( stderr
	       , "894sum XY %x %x i %d  sum %lf %lf count %lf\n"
	       , tmpX
	       , tmpY
	       , i
	       , sumX
	       , sumY
	       , count
       );
#endif
            }
        }
        if ( count >= 1.0 ) {
            avgX = (uint32_t)((int32_t)(sumX/count));
            avgY = (uint32_t)((int32_t)(sumY/count));
            tmpX = avgX & 0xFFFFFF00;
            tmpY = avgY & 0xFFFFFF00;
                   //
                   // center back-search on found point
                   //
            eolX = tmpX + (nSteps / 2) * delX;
            eolY = tmpY + (nSteps / 2) * delY;
            negX = (uint32_t)(-(int32_t)delX);
            negY = (uint32_t)(-(int32_t)delY);
	    if ( DoLevelSearch(pLgMaster, eolX, eolY, negX, negY, nSteps, gLout ) ) {
                   return kStopWasDone;
            }
            if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return kStopWasDone;
                 
	    // zero out first ten points
            gLout[0] = 0;
            gLout[1] = 0;
            gLout[2] = 0;
            gLout[3] = 0;
            gLout[4] = 0;
            gLout[5] = 0;
            gLout[6] = 0;
            gLout[7] = 0;
            gLout[8] = 0;
            gLout[9] = 0;

            ptr  = &(gLout[0]);
            tmpX = eolX;
            tmpY = eolY;
            i = 0;
            sumX = 0.0;
            sumY = 0.0;
            count = 0.0;
            while( (i < nSteps) ) {
                    tmpX += negX;
                    tmpY += negY;
                    if ( *ptr > minlevel ) {
                       sumX += (double)((int32_t)tmpX);
                       sumY += (double)((int32_t)tmpY);
                       count += 1.0;
                    }
                    ptr++;
                    i++;
            }
            if( count >= 1 ) {
	      avgX = (uint32_t)((int32_t)(sumX/count));
	      avgY = (uint32_t)((int32_t)(sumY/count));

#ifdef SZDEBUG
	      fprintf( stderr, "about to DoFineLevel  %x %x\n", avgX, avgY );
#endif
	      theResult = DoFineLevel( pLgMaster, &avgX
				       , &avgY
				       , &dMinX
				       , &dMinY
				       , &dMaxX
				       , &dMaxY
				       );
#ifdef SZDEBUG
	      fprintf( stderr, "finish DoFineLevel  result %x %.0lf %.0lf %.0lf %.0lf\n"
		       , theResult
		       , dMinX
		       , dMaxX
		       , dMinY
		       , dMaxY
		       );
#endif
	      if ( theResult ) { return theResult; }

#ifdef SZDEBUG
	      fprintf( stderr, "about to DoRedundand  %x %x\n", avgX, avgY );
#endif
	      theResult = DoRedundantSuperFineSearch ( pLgMaster,
						       &avgX,
						       &avgY,
						       &dMinX,
						       &dMinY,
						       &dMaxX,
						       &dMaxY);
#if defined( SZDEBUG ) || defined(ZZZDEBUG)
	      fprintf( stderr, "finished DoRedundand  %x %x  result %x\n", avgX, avgY, theResult );
#endif
	      if (  theResult == 0 ) {
		*foundX = avgX & kMask;
		*foundY = avgY & kMask;
		return 0;
	      }
            }
        }

        i = 10;
        tmpX = X + i * delX;
        tmpY = Y + i * delY;
        ptr  = &(gLout[10]);
        while( (i < nSteps) && (*ptr < minlevel) ) {
            tmpX += delX; tmpY += delY; ptr++; i++;
        }
        if( (*ptr >= minlevel) && (i < nSteps) ) {
                   //
                   // center back-search on found point
                   //
		eolX = tmpX + (nSteps / 2) * delX;
                eolY = tmpY + (nSteps / 2) * delY;
                negX = (uint32_t)(-(int32_t)delX);
                negY = (uint32_t)(-(int32_t)delY);
                if ( DoLevelSearch(pLgMaster, eolX, eolY, negX, negY, nSteps, gLout ) ) {
                   return kStopWasDone;
                }
                if (IfStopThenStopAndNeg1Else0(pLgMaster))
		  return kStopWasDone;
                 
                      // zero out first ten points
                gLout[0] = 0;
                gLout[1] = 0;
                gLout[2] = 0;
                gLout[3] = 0;
                gLout[4] = 0;
                gLout[5] = 0;
                gLout[6] = 0;
                gLout[7] = 0;
                gLout[8] = 0;
                gLout[9] = 0;

                ptr  = &(gLout[0]);
                tmpX = eolX;
                tmpY = eolY;
                i = 0;
                sumX = 0.0;
                sumY = 0.0;
                count = 0.0;
                while( (i < nSteps) ) {
                    tmpX += negX;
                    tmpY += negY;
                    if ( *ptr > minlevel ) {
                       sumX += (double)((int32_t)tmpX);
                       sumY += (double)((int32_t)tmpY);
                       count += 1.0;
#ifdef SZDEBUG
fprintf( stderr
       , "tmpXY1104 %x %x %x\n"
       , tmpX
       , tmpY
       , *ptr
       );
#endif
                    }
                    ptr++;
                    i++;
                }
#ifdef SZDEBUG
fprintf( stderr
       , "1048min XY %x %x %lf %lf eol %x %x nStep %d  count %lf\n"
       , tmpX
       , tmpY
       , sumX
       , sumY
       , eolX
       , eolY
       , nSteps 
       , count
       );
#endif
                if( count >= 1 ) {
                    avgX = (uint32_t)((int32_t)(sumX/count));
                    avgY = (uint32_t)((int32_t)(sumY/count));

#ifdef SZDEBUG
fprintf( stderr, "about to DoFineLevel  %x %x\n", avgX, avgY );
#endif
 theResult = DoFineLevel(pLgMaster, &avgX, &avgY, &dMinX, &dMinY, &dMaxX, &dMaxY);
#ifdef SZDEBUG
fprintf( stderr, "finish DoFineLevel  result %x %.0lf %.0lf %.0lf %.0lf\n"
               , theResult
               , dMinX
               , dMaxX
               , dMinY
               , dMaxY
               );
#endif
                    if ( theResult ) { return theResult; }

#ifdef SZDEBUG
fprintf( stderr, "about to DoRedundand  %x %x\n", avgX, avgY );
#endif
 theResult = DoRedundantSuperFineSearch (pLgMaster, &avgX, &avgY, &dMinX, &dMinY, &dMaxX, &dMaxY);
#if defined( SZDEBUG ) || defined(ZZZDEBUG)
fprintf( stderr, "finished DoRedundand  %x %x  result %x\n", avgX, avgY, theResult );
#endif
                    if (  theResult == 0 ) {
                        *foundX = avgX & kMask;
                        *foundY = avgY & kMask;
                        return 0;
                    }
                }
        }

        return theResult;
}        
                
int compare ( const void * a, const void * b )
{
        double * aaa;
        double * bbb;

        aaa = (double *) a;
        bbb = (double *) b;
        if( *aaa <  *bbb ) return -1;
        if( *aaa == *bbb ) return  0;
        if( *aaa >  *bbb ) return  1;

        return 0;
}



static int DoQuickFineSearch(struct lg_master *pLgMaster, uint32_t *foundX, uint32_t *foundY)
{
        uint32_t currentX, currentY;
        uint32_t centerX, centerY, theSpan;
        uint32_t delX;
        uint32_t tmpX;
        uint32_t delY;
        uint32_t tmpY;
        uint32_t nSteps;
        uint32_t finetest;
        short numberOfXScansToAverage;
        short numberOfYScansToAverage;
        int32_t i, j, index;
        uint32_t *ptr;
        
        theSpan = kSuperFineSearchStep * kSuperFineSearchSpanSteps;

        centerX = *foundX;
        centerY = *foundY;
        
        *foundX = 0x00000000U;
        *foundY = 0x00000000U;

        j = 1;
        numberOfXScansToAverage = 0;
        numberOfYScansToAverage = 0;
        while (j <= 3)
	  {
	    finetest = 0;
	    j++;
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      {
#ifdef SZDEBUG
		fprintf( stderr, "1279 kStopWasDone %d %d\n", gSearchFlag, gStopFlag );
#endif
		return kStopWasDone;
	      }

	    /* Go right */
	    currentX = centerX - theSpan;
	    currentY = centerY;
	    delX = 2*gSuperFineSearchStep;
	    nSteps = theSpan / gSuperFineSearchStep;
	    SetHWtrigger(pLgMaster);
#ifdef SZDEBUG
	    fprintf( stderr, "DoSuperFine pause 1\n" ); 
#endif
	    // xxx move_find( currentX, currentY );
	    if (DoLevelSearch(pLgMaster, currentX, currentY, delX, 0, nSteps, gLout))
	      {
	      }
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return kStopWasDone;

	    if (j == 1)
	      {
		ptr  = &(gLout[0]);
		memset( gLsort,   0, nSteps*sizeof(uint32_t) );
		for ( i = 0 ; i < nSteps ; i++ )
		  {
		    gLsort[i] = gLout[i] ;
		  }
		qsort( gLsort, nSteps, sizeof(uint32_t), int32_tsort );
		minlevel  = gDELLEV;
	      }

	    gLout[0] = 0;
	    gLout[1] = 0;
	    gLout[2] = 0;
	    gLout[3] = 0;
	    gLout[4] = 0;
	    gLout[5] = 0;
	    gLout[6] = 0;
	    index = 0; ptr = &(gLout[0]);   tmpX = currentX;
	    if ( j > 1 ) {
	      while( (index<nSteps) ) {
		if( gLout[index] >= minlevel ) {
		  xNegSuper[index]++;
		  numberOfXScansToAverage++;
		  finetest = finetest | 0x01;
#ifdef SZDEBUG
		  fprintf( stderr, "DoSuper1279 finetest %04x  %d\n", finetest, index ); 
#endif
		}
		xNegPosition[index] = tmpX;
		index++; ptr++;  tmpX += delX;
	      }
	    }

	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return kStopWasDone;

	    /* Go left */

	    currentX = centerX + theSpan;
	    currentY = centerY;
	    delX = (uint32_t)(-2*(int32_t)gSuperFineSearchStep);
	    nSteps = theSpan / gSuperFineSearchStep;
#ifdef SZDEBUG
	    fprintf( stderr, "DoSuper1298 finetest %04x  %d\n", finetest, index ); 
#endif
	    if ( DoLevelSearch(pLgMaster, currentX, currentY, delX, 0, nSteps, gLout))
	      {
	      }
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return kStopWasDone;

	    gLout[0] = 0;
	    gLout[1] = 0;
	    gLout[2] = 0;
	    gLout[3] = 0;
	    gLout[4] = 0;
	    gLout[5] = 0;
	    gLout[6] = 0;

	    index = 0; ptr = &(gLout[0]);  tmpX = currentX;
	    if ( j > 1 ) {
	      while( (index<nSteps) ) {
		if( gLout[index] >= minlevel ) {
		  xPosSuper[index]++;
		  numberOfXScansToAverage++;
		  finetest = finetest | 0x02;
#ifdef SZDEBUG
		  fprintf( stderr, "DoSuper1332 finetest %04x  %d\n", finetest, index ); 
#endif
		}
		xPosPosition[index] = tmpX;
		index++; ptr++;   tmpX += delX;
	      }
	    }

	    /* Go up    */
	    currentX = centerX;
	    currentY = centerY - theSpan;
	    delY = 2*gSuperFineSearchStep;
	    nSteps = theSpan / gSuperFineSearchStep;
	    SetHWtrigger(pLgMaster);
	    // xxx move_find( currentX, currentY );
	    if (DoLevelSearch(pLgMaster, currentX, currentY, 0, delY, nSteps, gLout))
	      {
	      }
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return kStopWasDone;

	    if ( j == 1 ) {
	      ptr  = &(gLout[0]);
	      memset( gLsort,   0, nSteps*sizeof(uint32_t) );
	      for ( i = 0 ; i < nSteps ; i++ ) {
		gLsort[i] = gLout[i] ;
	      }
	      qsort( gLsort, nSteps, sizeof(uint32_t), int32_tsort );
	      minlevel  = gDELLEV;
	    }

	    gLout[0] = 0;
	    gLout[1] = 0;
	    gLout[2] = 0;
	    gLout[3] = 0;
	    gLout[4] = 0;
	    gLout[5] = 0;
	    gLout[6] = 0;
	    index = 0; ptr = &(gLout[0]);   tmpY = currentY;
	    if ( j > 1 ) {
	      while( (index<nSteps) ) {
		if( gLout[index] >= minlevel ) {
		  yNegSuper[index]++;
		  numberOfYScansToAverage++;
		  finetest = finetest | 0x04;
#ifdef SZDEBUG
		  fprintf( stderr, "DoSuper1400 finetest %04x  %d\n", finetest, index ); 
#endif
		}
		yNegPosition[index] = tmpY;
		index++; ptr++;  tmpY += delY;
	      }
	    }

                if (IfStopThenStopAndNeg1Else0(pLgMaster))
		  return kStopWasDone;
		
                /* Go down */
                currentX = centerX;
                currentY = centerY + theSpan;
                delY = (uint32_t)(-2*(int32_t)gSuperFineSearchStep);
                nSteps = theSpan / gSuperFineSearchStep;
#ifdef SZDEBUG
                fprintf( stderr, "DoSuperFine pause 2\n" ); 
#endif
                if (DoLevelSearch(pLgMaster, currentX, currentY, 0, delY, nSteps, gLout))
		  {
		  }
                if (IfStopThenStopAndNeg1Else0(pLgMaster))
		  return kStopWasDone;

                gLout[0] = 0;
                gLout[1] = 0;
                gLout[2] = 0;
                gLout[3] = 0;
                gLout[4] = 0;
                gLout[5] = 0;
                gLout[6] = 0;

                index = 0; ptr = &(gLout[0]);  tmpY = currentY;
                if ( j > 1 ) {
                    while( (index<nSteps) ) {
                        if( gLout[index] >= minlevel ) {
                             yPosSuper[index]++;
                             numberOfYScansToAverage++;
                             finetest = finetest | 0x08;
                        }
                        yPosPosition[index] = tmpY;
                        index++; ptr++;   tmpY += delY;
                    }
                }
                if ( finetest == 0x0f ) break;
		
	  }

#ifdef SZDEBUG
	fprintf( stderr
		 , "1008DQFS numberOfXScansToAverage %3d   finetest %04x\n"
		 , numberOfXScansToAverage
		 , finetest
		 );
#endif

        if ( finetest == 0x0f ) return 0;

        
        if ( numberOfXScansToAverage <= 2 && numberOfYScansToAverage <= 2 )
                return kSuperFineNotFound;

        return kSuperFineNotFound;
}

void InitSensorSearch ( void )
{
  int i;

  gOut = (unsigned char *)calloc((size_t)kMaxOutLength,(size_t)1);
  gLout = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(uint32_t));
  gLout1 = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(uint32_t));
  gLout2 = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(uint32_t));
  gLsort = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(uint32_t));

  xPosSave = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  xNegSave = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  yPosSave = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  yNegSave = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));

  xPosEdge = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  xNegEdge = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  yPosEdge = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  yNegEdge = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));

  xPosSuper = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  xNegSuper = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  yPosSuper = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  yNegSuper = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));

  xPosESuper = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  xNegESuper = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  yPosESuper = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  yNegESuper = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));

  xPosPosition = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  yPosPosition = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));

  xNegPosition = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  yNegPosition = (uint32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));

  xpResArr = (double *)calloc((size_t)kNumberOfSuperCrossesToAverage,
          sizeof(double));
  xnResArr = (double *)calloc((size_t)kNumberOfSuperCrossesToAverage,
          sizeof(double));
  ypResArr = (double *)calloc((size_t)kNumberOfSuperCrossesToAverage,
          sizeof(double));
  ynResArr = (double *)calloc((size_t)kNumberOfSuperCrossesToAverage,
          sizeof(double));

  gSaveMatch = (int *)calloc(   (size_t)500000,sizeof(int)); 
  gSaveSweep = (int *)calloc(   (size_t)500000,sizeof(int)); 
  gSaveLout1 = (int *)calloc(   (size_t)500000,sizeof(int)); 
  gSaveLout2 = (int *)calloc(   (size_t)500000,sizeof(int)); 
  gSaveDblX  = (double *)calloc((size_t)500000,sizeof(double)); 
  gSaveDblY  = (double *)calloc((size_t)500000,sizeof(double)); 
  gSaveDblX1 = (double *)calloc((size_t)500000,sizeof(double)); 
  gSaveDblX2 = (double *)calloc((size_t)500000,sizeof(double)); 
  gSaveDblY1 = (double *)calloc((size_t)500000,sizeof(double)); 
  gSaveDblY2 = (double *)calloc((size_t)500000,sizeof(double)); 
  gXsuperSave = (int32_t *)calloc((size_t)500000,sizeof(int32_t)); 
  gYsuperSave = (int32_t *)calloc((size_t)500000,sizeof(int32_t)); 
  gLoutBase   = (char *)calloc((size_t)6000000,sizeof(int32_t)); 
  gLoutTargetCount = (int32_t *)calloc((size_t)25,sizeof(int32_t));

  gSuperReturn   = (char **)calloc((size_t)1024,sizeof(char *)); 
  for ( i=0; i<1024; i++) {
      gSuperReturn[i]   = (char *)calloc((size_t)1024,sizeof(char)); 
  }

}

void CloseSensorSearch ( void )
{
  free( gOut);
  free( gLout);

  free( xPosSave);
  free( xNegSave);
  free( yPosSave);
  free( yNegSave);

  free( xPosEdge);
  free( xNegEdge);
  free( yPosEdge);
  free( yNegEdge);

  free( xPosSuper);
  free( xNegSuper);
  free( yPosSuper);
  free( yNegSuper);

  free( xPosESuper);
  free( xNegESuper);
  free( yPosESuper);
  free( yNegESuper);

  free( xPosPosition);
  free( yPosPosition);

  free( xNegPosition);
  free( yNegPosition);

  free( xpResArr);
  free( xnResArr);
  free( ypResArr);
  free( ynResArr);

}

void CorrectDrift( uint32_t cX,
                   uint32_t cY,
                   uint32_t *fX,
                   uint32_t *fY )
{
     int32_t tmpX, tmpY;
     double doubleX, doubleY;
     double correctX, correctY;

     tmpX = (int32_t)cX;
     tmpY = (int32_t)cY;

     doubleX = (double)tmpX;
     doubleY = (double)tmpY;

     correctX = g_aX + (g_bX * doubleX);
     correctY = g_aY + (g_bY * doubleY);

     tmpX = (int32_t)correctX;
     tmpY = (int32_t)correctY;

     *fX = ((uint32_t)tmpX) & 0xFFFFF000;
     *fY = ((uint32_t)tmpY) & 0xFFFFF000;

}

void FindDrift( uint32_t cX,
              uint32_t cY,
              uint32_t fX,
              uint32_t fY )
{
     int32_t tmpX, tmpY;
     double driftX, driftY;
     double fixX, fixY;
     int i;
     double sumD, sumF, sumFF, sumFD;
     double delta;
     double aX, aY;

     tmpX = (int32_t)cX;
     tmpY = (int32_t)cY;
     fixX = (double)tmpX;
     fixY = (double)tmpY;
     tmpX = (int32_t)fX;
     tmpY = (int32_t)fY;
     driftX = (double)tmpX;
     driftY = (double)tmpY;

     for( i = (kNumberDrift-1); i > 0; i-- ) {
        gFixX[i]   = gFixX[i-1];
        gFixY[i]   = gFixY[i-1];
        gDriftX[i] = gDriftX[i-1];
        gDriftY[i] = gDriftY[i-1];
     }
     gFixX[0] = fixX;
     gFixY[0] = fixY;
     gDriftX[0] = driftX;
     gDriftY[0] = driftY;

     if( gTargetDrift == 2 ) {
       sumF = 0.0; sumD = 0.0; sumFF = 0.0; sumFD = 0.0;
       for( i = 0; i < kNumberDrift; i++ ) {
          sumF += gFixX[i];
          sumD += gDriftX[i];
          sumFF += gFixX[i] * gFixX[i];
          sumFD += gFixX[i] * gDriftX[i];
       }
       delta = (double)kNumberDrift * sumFF - sumF * sumF;
       g_aX = (sumFF * sumD - sumF * sumFD) / delta;
       g_bX = ((double)kNumberDrift * sumFD - sumF * sumD) / delta;
       sumF = 0.0; sumD = 0.0; sumFF = 0.0; sumFD = 0.0;
       for( i = 0; i < kNumberDrift; i++ ) {
          sumF += gFixY[i];
          sumD += gDriftY[i];
          sumFF += gFixY[i] * gFixY[i];
          sumFD += gFixY[i] * gDriftY[i];
       }
       delta = (double)kNumberDrift * sumFF - sumF * sumF;
       g_aY = (sumFF * sumD - sumF * sumFD) / delta;
       g_bY = ((double)kNumberDrift * sumFD - sumF * sumD) / delta;
     } else if( gTargetDrift == 1 ) {
       sumFD = 0.0;
       for( i = 0; i < kNumberDrift; i++ ) {
         sumFD += gDriftX[i] - gFixX[i];
       }
       g_aX = sumFD / (double)kNumberDrift;
       g_bX = 1.0;
       sumFD = 0.0;
       for( i = 0; i < kNumberDrift; i++ ) {
         sumFD += gDriftY[i] - gFixY[i];
       }
       g_aY = sumFD / (double)kNumberDrift;
       g_bY = 1.0;
     }


     tmpX = (int32_t)g_aX;
     tmpY = (int32_t)g_aY;
     aX = g_aX / 65536.0;
     aY = g_aY / 65536.0;
     fprintf( stderr, "aXbXaYbY %12.2f %12.6f %12.2f %12.6f\n", 
         aX, g_bX, aY, g_bY);
}

void InitDrift( uint32_t *Xarr, uint32_t *Yarr)
{
    int i, j;

    g_aX = 0.0;  g_bX = 1.0;  g_aY = 0.0; g_bY = 1.0;
    for( i = 0; i < kNumberDrift; i++ ) {
       j = i % kNumberOfRegPoints;
       gFixX[i]   = (double)((int32_t)Xarr[j]);
       gDriftX[i] = (double)((int32_t)Xarr[j]);
       gFixY[i]   = (double)((int32_t)Yarr[j]);
       gDriftY[i] = (double)((int32_t)Yarr[j]);
    }
}

int  FakeLeg( struct lg_master *pLgMaster, uint32_t X,    uint32_t Y,
	      uint32_t delX, uint32_t delY,
	      uint32_t nSteps,
	      uint32_t * foundX, uint32_t * foundY )
{
        uint32_t tmpX, tmpY;
        uint32_t eolX, eolY;
        uint32_t negX, negY;
        uint32_t *ptr;
        int32_t i;
        int theResult;

        theResult = kCoarseNotFound;

	SetHWtrigger(pLgMaster);
        i = 0; ptr  = gLout;
        while( i < nSteps ) {
                  *ptr = 0; ptr++; i++;
        }
	move_dark(pLgMaster, X, Y );
        if ( DoLevelSearch(pLgMaster, X, Y, delX, delY, nSteps, gLout ) ) {
                   return kStopWasDone;
        }
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;
	
        ptr  = gLout;
        tmpX = X;
        tmpY = Y;
        gLout[0] = 0;
        gLout[1] = 0;
        gLout[2] = 0;
        i = 0;
        ptr  = gLout;
        while( (i < nSteps) && (*ptr < minlevel) ) {
            tmpX += delX; tmpY += delY; ptr++; i++;
        }
        if( (*ptr >= minlevel) && (i < nSteps) ) {
                eolX = X + nSteps * delX;
                eolY = Y + nSteps * delY;
                negX = (uint32_t)(-(int32_t)delX);
                negY = (uint32_t)(-(int32_t)delY);
		move_dark(pLgMaster, eolX, eolY );

		if ( DoLevelSearch(pLgMaster, eolX, eolY, negX, negY, nSteps, gLout ) ) {
                   return kStopWasDone;
               }
               if (IfStopThenStopAndNeg1Else0(pLgMaster))
		 return kStopWasDone;

               gLout[0] = 0;
               gLout[1] = 0;
               gLout[2] = 0;
               ptr  = &(gLout[0]);
               tmpX = eolX;
               tmpY = eolY;
               i = 0;
               while( (i < nSteps) && (*ptr < minlevel) ) {
                    tmpX += negX;
                    tmpY += negY;
                    ptr++;
                    i++;
               }
        }

        return theResult;
}        


static int DoFineLevel (struct lg_master *pLgMaster,
			uint32_t *foundX
			, uint32_t *foundY
			, double * outMinX
			, double * outMinY
			, double * outMaxX
			, double * outMaxY
			)
{
        uint32_t currentX, currentY;
        uint32_t eolXPos, eolYPos;
        uint32_t eolXNeg, eolYNeg;
        uint32_t centerX, centerY;
        uint32_t tmpX, tmpY;
        uint32_t firstX, firstY, lastX, lastY;
        uint32_t stepSize, negStep;
        uint32_t nSteps;
        int32_t Xm, Ym;
        double dFX;
        double dFY;
        double dLX;
        double dLY;
        double avX;
        double avY;
        double dMinX;
        double dMinY;
        double dMaxX;
        double dMaxY;
        int result;
        
        
	nSteps = kFineSearchSpanSteps << 1;
                
        centerX = *foundX;
        centerY = *foundY;
                
#ifdef SZDEBUG
	fprintf( stderr, "763 DoFine centerX/Y %10x %10x\n", centerX, centerY );
#endif
	move_dark(pLgMaster, centerX, centerY );

        stepSize = kFineSearchStep;
        negStep  = (uint32_t)(-(int32_t)kFineSearchStep);
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;
	
	/* Go right */
	limitCalc(centerX, centerY, &eolXNeg, &eolXPos, &eolYNeg, &eolYPos);
	currentX = eolXNeg;
	currentY = centerY;
	result =  findFirstLast(pLgMaster, &firstX, &firstY, &lastX, &lastY,
				&Xm, &Ym, currentX, currentY, stepSize, 0,
				nSteps);
	if ( result )
	  {
	    currentX = centerX;
	    currentY = eolYNeg;
	    
	    result =  findFirstLast(pLgMaster, &firstX, &firstY, &lastX,
				    &lastY, &Xm, &Ym, currentX, currentY,
				    0, stepSize, nSteps);
	    if ( result ) {
	      return result;
	    }
	  }

	dFX = (double)((int32_t)firstX);
	dFY = (double)((int32_t)firstY);
	// initialization
	dMinX = dFX;
	dMaxX = dFX;
	dMinY = dFY;
	dMaxY = dFY;
#ifdef SZDEBUG
	fprintf(stderr, "MinMax X %.0lf %.0lf Y %.0lf %.0f\n",dMinX,dMaxX,dMinY,dMaxY);
#endif
	dLX = (double)((int32_t)lastX);
	dLY = (double)((int32_t)lastY);
	if ( dLX < dMinX ) dMinX = dLX;
	if ( dLY < dMinY ) dMinY = dLY;
	if ( dLX > dMaxX ) dMaxX = dLX;
	if ( dLY > dMaxY ) dMaxY = dLY;
	avX = (double)Xm;
	avY = (double)Ym;
	tmpX = (uint32_t)((int32_t)avX);
	tmpY = (uint32_t)((int32_t)avY);
#ifdef SZDEBUG
	fprintf(stderr, "MinMax X %.0lf %.0lf Y %.0lf %.0f\n",dMinX,dMaxX,dMinY,dMaxY);
#endif
	centerX = tmpX;
	centerY = tmpY;
	
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;

	/* Go up */
	limitCalc( centerX
		   , centerY
		   , &eolXNeg
		   , &eolXPos
		   , &eolYNeg
		   , &eolYPos 
		   );

	currentX = centerX;
	currentY = eolYNeg;

	result =  findFirstLast( pLgMaster, &firstX
				 , &firstY
				 , &lastX
				 , &lastY
				 , &Xm
				 , &Ym
				 , currentX
				 , currentY
				 , 0
				 , stepSize
				 , nSteps
				 );
	if ( result ) return result;

	dFX = (double)((int32_t)firstX);
	dFY = (double)((int32_t)firstY);
	dLX = (double)((int32_t)lastX);
	dLY = (double)((int32_t)lastY);
	if ( dFX < dMinX ) dMinX = dFX;
	if ( dFY < dMinY ) dMinY = dFY;
	if ( dFX > dMaxX ) dMaxX = dFX;
	if ( dFY > dMaxY ) dMaxY = dFY;
	if ( dLX < dMinX ) dMinX = dLX;
	if ( dLY < dMinY ) dMinY = dLY;
	if ( dLX > dMaxX ) dMaxX = dLX;
	if ( dLY > dMaxY ) dMaxY = dLY;
	avX = Xm;
	avY = Ym;
	tmpX = (uint32_t)((int32_t)avX);
	tmpY = (uint32_t)((int32_t)avY);
#ifdef SZDEBUG
	fprintf(stderr, "MinMax X %.0lf %.0lf Y %.0lf %.0f\n",dMinX,dMaxX,dMinY,dMaxY);
#endif
	centerX = tmpX;
	centerY = tmpY;
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;

	/* Go left */
	limitCalc( centerX
		   , centerY
		   , &eolXNeg
		   , &eolXPos
		   , &eolYNeg
		   , &eolYPos 
		   );

	currentX = eolXPos;
	currentY = centerY;

	result =  findFirstLast( pLgMaster, &firstX
				 , &firstY
				 , &lastX
				 , &lastY
				 , &Xm
				 , &Ym
				 , currentX
				 , currentY
				 , negStep
				 , 0
				 , nSteps
				 );
	if ( result ) return result;
	
	dFX = (double)((int32_t)firstX);
	dFY = (double)((int32_t)firstY);
	dLX = (double)((int32_t)lastX);
	dLY = (double)((int32_t)lastY);
	if ( dFX < dMinX ) dMinX = dFX;
	if ( dFY < dMinY ) dMinY = dFY;
	if ( dFX > dMaxX ) dMaxX = dFX;
	if ( dFY > dMaxY ) dMaxY = dFY;
	if ( dLX < dMinX ) dMinX = dLX;
	if ( dLY < dMinY ) dMinY = dLY;
	if ( dLX > dMaxX ) dMaxX = dLX;
	if ( dLY > dMaxY ) dMaxY = dLY;
	avX = Xm;
	avY = Ym;
	tmpX = (uint32_t)((int32_t)avX);
	tmpY = (uint32_t)((int32_t)avY);
#ifdef SZDEBUG
	fprintf(stderr, "MinMax X %.0lf %.0lf Y %.0lf %.0f\n",dMinX,dMaxX,dMinY,dMaxY);
#endif
	centerX = tmpX;
	centerY = tmpY;
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;

	/* Go right (again) */
	limitCalc( centerX
		   , centerY
		   , &eolXNeg
		   , &eolXPos
		   , &eolYNeg
		   , &eolYPos 
		   );

	currentX = eolXNeg;
	currentY = centerY;
	result =  findFirstLast( pLgMaster, &firstX
				 , &firstY
				 , &lastX
				 , &lastY
				 , &Xm
				 , &Ym
				 , currentX
				 , currentY
				 , stepSize
				 , 0
				 , nSteps
				 );
	if ( result ) return result;

	dFX = (double)((int32_t)firstX);
	dFY = (double)((int32_t)firstY);
	dLX = (double)((int32_t)lastX);
	dLY = (double)((int32_t)lastY);
	if ( dFX < dMinX ) dMinX = dFX;
	if ( dFY < dMinY ) dMinY = dFY;
	if ( dFX > dMaxX ) dMaxX = dFX;
	if ( dFY > dMaxY ) dMaxY = dFY;
	if ( dLX < dMinX ) dMinX = dLX;
	if ( dLY < dMinY ) dMinY = dLY;
	if ( dLX > dMaxX ) dMaxX = dLX;
	if ( dLY > dMaxY ) dMaxY = dLY;
#ifdef SZDEBUG
	fprintf(stderr, "MinMax X %.0lf %.0lf Y %.0lf %.0f\n",dMinX,dMaxX,dMinY,dMaxY);
#endif
	avX = Xm;
	avY = Ym;
	tmpX = (uint32_t)((int32_t)avX);
	tmpY = (uint32_t)((int32_t)avY);
#ifdef SZDEBUG
	fprintf(stderr, "MinMax X %.0lf %.0lf Y %.0lf %.0f\n",dMinX,dMaxX,dMinY,dMaxY);
#endif
	centerX = tmpX;
	centerY = tmpY;
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;

	/* Go down */
	limitCalc( centerX
		   , centerY
		   , &eolXNeg
		   , &eolXPos
		   , &eolYNeg
		   , &eolYPos 
		   );

	currentX = centerX;
	currentY = eolYPos;
	
	result =  findFirstLast( pLgMaster, &firstX
				 , &firstY
				 , &lastX
				 , &lastY
				 , &Xm
				 , &Ym
				 , currentX
				 , currentY
				 , 0
				 , negStep
				 , nSteps
				 );
	if ( result ) return result;

	dFX = (double)((int32_t)firstX);
	dFY = (double)((int32_t)firstY);
	dLX = (double)((int32_t)lastX);
	dLY = (double)((int32_t)lastY);
	if ( dFX < dMinX ) dMinX = dFX;
	if ( dFY < dMinY ) dMinY = dFY;
	if ( dFX > dMaxX ) dMaxX = dFX;
	if ( dFY > dMaxY ) dMaxY = dFY;
	if ( dLX < dMinX ) dMinX = dLX;
	if ( dLY < dMinY ) dMinY = dLY;
	if ( dLX > dMaxX ) dMaxX = dLX;
	if ( dLY > dMaxY ) dMaxY = dLY;
	avX = Xm;
	avY = Ym;
	tmpX = (uint32_t)((int32_t)avX);
	tmpY = (uint32_t)((int32_t)avY);
#ifdef SZDEBUG
	fprintf(stderr, "MinMax X %.0lf %.0lf Y %.0lf %.0f\n",dMinX,dMaxX,dMinY,dMaxY);
#endif
	centerX = tmpX;
	centerY = tmpY;
	*outMinX = dMinX - 8 * (double)((int32_t)stepSize);
        *outMinY = dMinY - 8 * (double)((int32_t)stepSize);
        *outMaxX = dMaxX + 8 * (double)((int32_t)stepSize);
        *outMaxY = dMaxY + 8 * (double)((int32_t)stepSize);

#ifdef SZDEBUG
        *outMinX = dMinX - 16 * (double)((int32_t)stepSize);
        *outMinY = dMinY - 16 * (double)((int32_t)stepSize);
        *outMaxX = dMaxX + 16 * (double)((int32_t)stepSize);
        *outMaxY = dMaxY + 16 * (double)((int32_t)stepSize);

	fprintf( stderr, "X Y 2909" );
	fprintf( stderr, " %lf", dMinX / 1000000 );
	fprintf( stderr, " %lf", dMaxX / 1000000 );
	fprintf( stderr, " %lf", dMinY / 1000000 );
	fprintf( stderr, " %lf", dMaxY / 1000000 );
	fprintf( stderr, "\n" );
#endif

        *foundX = centerX & kMask;
        *foundY = centerY & kMask;
#ifdef SZDEBUG
	fprintf( stderr, "search 1234  %x %x\n", *foundX, *foundY );
#endif
        
        return 0;
}



static int        
DoRedundantSuperFineSearch (struct lg_master *pLgMaster,
			    uint32_t *foundX,
			    uint32_t *foundY,
			    double * dMinX,
			    double * dMinY,
			    double * dMaxX,
			    double * dMaxY)
{
        uint32_t tempX;
        uint32_t tempY;
        int result;

        tempX = *foundX;
        tempY = *foundY;

	result = DoSuperLevelSearch(pLgMaster, &tempX,
				    &tempY, dMinX, dMinY,
				    dMaxX, dMaxY);
	if (result)
	  return result;

        *foundX = kMask & (uint32_t)((int32_t)tempX);
        *foundY = kMask & (uint32_t)((int32_t)tempY);
        return 0;
}

static int
DoSuperLevelSearch ( struct lg_master *pLgMaster,
		     uint32_t *foundX,
		     uint32_t *foundY,
		     double * dMinX,
		     double * dMinY,
		     double * dMaxX,
		     double * dMaxY)
{
        double sumX, sumY;
        double sweepSumX, sweepSumY;
        double sweepNumX, sweepNumY;
        double sweepAvgX, sweepAvgY;
        double avgX, avgY;
        double Dix, Djy;
        uint32_t tmpX, tmpY;
        int count;
        int localCount;
        int result;
        int i,j;
        int ix, jy;
        int ixsum, jysum, nsum;
        char * ptr;
        
        *foundX = 0x00000000U;
        *foundY = 0x00000000U;
        count = 0;
        sumX = 0;
        sumY = 0;
        

	result = SuperSearch( pLgMaster, &tmpX, &tmpY, dMinX, dMinY, dMaxX, dMaxY);
        if (result)
	  return result;
        count = 1;
        sumX += (double)((int32_t)tmpX);
        sumY += (double)((int32_t)tmpY);

        avgX = sumX / (double)count;
        avgY = sumY / (double)count;

        *foundX = kMask & (uint32_t)((int32_t)avgX);
        *foundY = kMask & (uint32_t)((int32_t)avgY);
        if ( gSearchCurrentSensor == 0 || gLoutPtr == 0 ) {
          gLoutPtr   = gLoutBase + 3*sizeof(int32_t) + 24 * sizeof(int32_t);
          gLoutSize  =             3*sizeof(int32_t) + 24 * sizeof(int32_t);
          gLoutCount = 0;
        }
        for ( i = 0; i < 1024; i++ ) {
            for ( j = 0; j < 1024; j++ ) {
                 gSuperReturn[i][j] = 0;
            }
        }
        ptr = gLoutPtr;
        localCount = 0;
        sweepSumX = 0.0; 
        sweepSumY = 0.0; 
        sweepNumX = 0.0; 
        sweepNumY = 0.0; 
        sweepAvgX = 0.0; 
        sweepAvgY = 0.0; 
        if ( gLoutIndex > 1 ) {
           for ( i = 0; i < gLoutIndex ; i++ ) { 
                    // X average
               if ( gSaveSweep[i] == 1 ) {
                   sweepSumX += gSaveDblX1[i];
                   sweepNumX += 1.0;
               }
                    // Y average
               if ( gSaveSweep[i] == 2 ) {
                   sweepSumY += gSaveDblY1[i];
                   sweepNumY += 1.0;
               }
               if ( (gSaveMatch[i] == 1) || (gSaveMatch[i] == 11) ) {

                   // array filling
                   ix = (int)((gSaveDblX1[i] - *dMinX)/(double)gSuperFineSearchStep);
                   jy = (int)((gSaveDblY1[i] - *dMinY)/(double)gSuperFineSearchStep);
                   if ( ix>=0 && jy>=0 && ix<1024 && jy<1024 ) {
                        gSuperReturn[ix][jy] = 1;
                   } else {
fprintf( stderr, "error SS2208 xy %d %d\n", ix, jy );
                   }

                   ((int32_t *)ptr)[0] = (int32_t)(gSaveDblX1[i]);
                   ((int32_t *)ptr)[1] = (int32_t)(gSaveDblY1[i]);
                   ptr      += 2 * sizeof(int32_t);
                   gLoutPtr += 2 * sizeof(int32_t);
                   *((short *)ptr) = gSaveLout1[i];
                   ptr      += sizeof(short);
                   gLoutPtr += sizeof(short);
                   ((unsigned char *)ptr)[0] = (unsigned char)(10*gSearchCurrentSensor+gSaveSweep[i]);

#if defined(LOUTDEBUG)
fprintf(stderr,"qqqyz %x %d %d %d\n", gSearchCurrentSensor, ((unsigned char *)ptr)[0], i, gLoutCount );
fprintf(stderr,"q2419 %d %d %d %d\n", ((unsigned char *)ptr)[0], (int32_t)(gSaveDblX1[i]), (int32_t)(gSaveDblY1[i]), i );
#endif
                   ((char *)ptr)[1] = (char)(gSaveMatch[i]);
                   if (gSaveMatch[i] == 12) ((char *)ptr)[1] = 0;
                   ptr      += 2 * sizeof(char);
                   gLoutPtr += 2 * sizeof(char);

                   gLoutSize  += (2*sizeof(int32_t)+sizeof(short)+2*sizeof(char) );
                   gLoutCount += 1;
                   localCount += 1;
               }

               if ( (gSaveMatch[i] == 1) || (gSaveMatch[i] == 12) ) {
                   ((int32_t *)ptr)[0] = (int32_t)(gSaveDblX2[i]);
                   ((int32_t *)ptr)[1] = (int32_t)(gSaveDblY2[i]);
                   // array filling
                   ix = (int)((gSaveDblX2[i] - *dMinX)/(double)gSuperFineSearchStep);
                   jy = (int)((gSaveDblY2[i] - *dMinY)/(double)gSuperFineSearchStep);
                   if ( ix>=0 && jy>=0 && ix<1024 && jy<1024 ) {
                        gSuperReturn[ix][jy] = 1;
                   } else {
fprintf( stderr, "error SS2208 xy %d %d\n", ix, jy );
                   }
                   ptr      += 2 * sizeof(int32_t);
                   gLoutPtr += 2 * sizeof(int32_t);
                   *((short *)ptr) = gSaveLout2[i];
                   ptr      += sizeof(short);
                   gLoutPtr += sizeof(short);
                   ((char *)ptr)[0] = (char)(10*gSearchCurrentSensor+gSaveSweep[i]);
                   ((char *)ptr)[1] = (char)(gSaveMatch[i]);
                   if (gSaveMatch[i] == 11) ((char *)ptr)[1] = 0;
                   ptr      += 2 * sizeof(char);
                   gLoutPtr += 2 * sizeof(char);

                   gLoutSize  += (2*sizeof(int32_t)+sizeof(short)+2*sizeof(char) );
                   gLoutCount += 1;
                   localCount += 1;
               }
           }
        }
        ixsum = 0; jysum = 0; nsum = 0;
        for ( i = 0; i < 1024; i++ ) {
            for ( j = 0; j < 1024; j++ ) {
                 if (  gSuperReturn[i][j] == 1 ) {
                     ixsum += i;
                     jysum += j;
                     nsum++;
                 }
            }
        }
        if ( nsum > 0 ) {
            Dix = (double)ixsum / (double)nsum;
            Djy = (double)jysum / (double)nsum;
            avgX =  Dix * gSuperFineSearchStep + *dMinX;
            avgY =  Djy * gSuperFineSearchStep + *dMinY;
            *foundX = kMask & (uint32_t)((int32_t)avgX);
            *foundY = kMask & (uint32_t)((int32_t)avgY);
#ifdef SZDEBUG
fprintf( stderr, "DoS2281 avg %lf %lf fnd %x %x\n", avgX, avgY,*foundX,*foundY);
#endif
        }
        if ( gMultipleSweeps == 2 && sweepNumX > 1 && sweepNumY > 1 ) {
            sweepAvgX = sweepSumX / sweepNumX;
            sweepAvgY = sweepSumY / sweepNumY;
            *foundX = kMask & (uint32_t)((int32_t)sweepAvgX);
            *foundY = kMask & (uint32_t)((int32_t)sweepAvgY);
#ifdef SZDEBUG
fprintf( stderr, "DoS2485 avg %lf %lf fnd %x %x\n", sweepAvgX, sweepAvgY,*foundX,*foundY);
fprintf( stderr, "DoS2483 sum %lf %lf \n", sweepSumX, sweepSumY );
fprintf( stderr, "DoS2484 num %lf %lf \n", sweepNumX, sweepNumY );
#endif
        }

        ptr = gLoutBase + 1 * sizeof(int32_t);
        *((int32_t *)ptr) = gLoutSize;
        ptr = gLoutBase + 2 * sizeof(int32_t);
        *((int32_t *)ptr) = gLoutCount;

             // zero based current target (sensor)
        ptr = gLoutBase + (3+gSearchCurrentSensor) * sizeof(int32_t);
        *((int32_t *)ptr) = localCount;

#if defined(LOUTDEBUG)
fprintf( stderr, "Lout3sensor %d %d %d\n", gSearchCurrentSensor, gLoutSize, gLoutCount );
if ( gSearchCurrentSensor == 5 ) {
  for ( i = 1; i < 1000; i++ ) {
   ptr = gLoutBase + i * sizeof(int32_t);
   fprintf( stderr, "L0ut5sensor %4d %9d %08x\n", i, *((int32_t *)ptr) , *((int32_t *)ptr));
  }
  for ( i = 20000; i < 21000; i++ ) {
   ptr = gLoutBase + i * sizeof(int32_t);
   fprintf( stderr, "L0ut5sensor %4d %9d %08x\n", i, *((int32_t *)ptr) , *((int32_t *)ptr));
  }
}
#endif

        return 0;
}


static int
findFirstLast( struct lg_master *pLgMaster,
	       uint32_t *firstX,
	       uint32_t *firstY,
	       uint32_t *lastX,
	       uint32_t *lastY,
	       int32_t *medianX,
	       int32_t *medianY,
	       uint32_t currentX,
	       uint32_t currentY,
	       uint32_t xStep,
	       uint32_t yStep,
	       uint32_t nSteps)
{
        int i;
        uint32_t *ptr;
        int iCountMedian;
        int ihalf;
        double tmpX;
        double tmpY;
        double startX;
        double startY;
        double dX;
        double dY;
        double dI;
        int32_t dXMedian[4096];
        int32_t dYMedian[4096];
        int32_t Xm, Ym;
        int firstFlag;

        firstFlag = 1;

	if (DoLevelSearch(pLgMaster, currentX, currentY, xStep, yStep, nSteps, gLout))
	  {
	    return kStopWasDone;
	  }
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;

#ifdef SZDEBUG
        startX = (double)((int32_t)currentX);
        startY = (double)((int32_t)currentY);
        dX = (double)((int32_t)xStep);
        dY = (double)((int32_t)yStep);
        i = 0;
        while( (i<nSteps) ) {
               dI = (double)i;
               tmpX = startX + dI * dX;
               tmpY = startY + dI * dY;
fprintf( stderr
       , "ss2038xyf  %lf %lf %d \n"
       , tmpX / 1000000
       , tmpY / 1000000
       , gLout[i]
       );
                i++;
        }
#endif

           // zero out the first twelve entries
        i = 0;
        gLout[0] = 0;
        gLout[1] = 0;
        gLout[2] = 0;
        gLout[3] = 0;
        gLout[4] = 0;
        gLout[5] = 0;
        gLout[6] = 0;
        gLout[7] = 0;
        gLout[8] = 0;
        gLout[9] = 0;
        gLout[10] = 0;
        gLout[11] = 0;

        i = 0;
        ptr = &(gLout[0]);
        iCountMedian = 0;
        startX = (double)((int32_t)currentX);
        startY = (double)((int32_t)currentY);
        dX = (double)((int32_t)xStep);
        dY = (double)((int32_t)yStep);
#ifdef SZDEBUG
fprintf( stderr, "fstart %.0lf %.0lf del %.0lf %.0lf \n", startX, startY,dX,dY );
#endif
        while( (i<nSteps) ) {
               dI = (double)i;
               tmpX = startX + dI * dX;
               tmpY = startY + dI * dY;
                   // throw away first few 
               if( *ptr >= minlevel && i > NZERO ) {
                  if ( firstFlag == 1 ) {
                     firstFlag = 0;
                     *firstX = (uint32_t)((int32_t)tmpX);
                     *firstY = (uint32_t)((int32_t)tmpY);
                  }
                  *lastX = (uint32_t)((int32_t)tmpX);
                  *lastY = (uint32_t)((int32_t)tmpY);
                  dXMedian[iCountMedian] = (int32_t)tmpX;
                  dYMedian[iCountMedian] = (int32_t)tmpY;
                  iCountMedian++;
               }
#ifdef SZDEBUG
fprintf( stderr, "flast %.0lf %.0lf %x %x %x minlev %x\n", tmpX, tmpY, *ptr, *lastX, *lastY, minlevel );
fprintf( stderr, "flast %.0lf %.0lf %x %x %x minlev %x\n", tmpX, tmpY, *ptr, *lastX, *lastY, minlevel );
#endif
               i++; ptr++;
        }
        if ( iCountMedian == 0 ) {
               return kFineNotFound;
        }

        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;

        qsort( dXMedian, iCountMedian, sizeof(int32_t), int32_tsort );
        qsort( dYMedian, iCountMedian, sizeof(int32_t), int32_tsort );

        ihalf = iCountMedian / 2;
        Xm  = dXMedian[ihalf];
        Ym  = dYMedian[ihalf];

#ifdef SZDEBUG
fprintf( stderr, "find1stlast 2979  %8x %8x %8x %8x\n", Xm, Ym, *lastX, *lastY );
#endif
        *medianX = Xm;
        *medianY = Ym;

        return 0;
}

void
limitCalc( uint32_t centerX
         , uint32_t centerY
         , uint32_t *eolXNeg
         , uint32_t *eolXPos
         , uint32_t *eolYNeg
         , uint32_t *eolYPos
         )
{
      uint32_t theSpan, twiceSpan;

      theSpan = kFineSearchStep * kFineSearchSpanSteps;
      twiceSpan = theSpan << 1;

      if ( (int32_t)centerX <= (int32_t)kMinNumberX + (int32_t)theSpan ) {
             *eolXNeg = kMinNumberX + theSpan;
      } else {
             *eolXNeg = centerX - theSpan;
             if ( (int32_t)*eolXNeg > ( (int32_t)kMaxNumberX - (int32_t)twiceSpan ) )
                     *eolXNeg = kMaxNumberX - twiceSpan;
      }
      *eolXPos = *eolXNeg + twiceSpan;

       if ( (int32_t)centerY <= (int32_t)kMinNumberY + (int32_t)theSpan ) {
             *eolYNeg = kMinNumberY + theSpan;
       } else {
             *eolYNeg = centerY - theSpan;
             if ( (int32_t)*eolYNeg > ( (int32_t)kMaxNumberY - (int32_t)twiceSpan ) )
                     *eolYNeg = kMaxNumberY - twiceSpan;
       }
       *eolYPos = *eolYNeg + twiceSpan;

#ifdef SZDEBUG
fprintf( stderr, "Zen 65   x %x y %x   %x %x %x %\n",
     centerX, centerY, *eolXPos, *eolXNeg, *eolYPos, *eolYNeg);
#endif

}

void
SetSuperFineFactor ( uint32_t n )
{
  if ( n > 0x100 ) { n = 0x100; }
  if ( n < 0x1 ) { n = 0x1; }
  gSuperFineFactor  = n;
  gSuperFineSearchStep =  gSuperFineFactor * kSuperFineSearchStep;
#ifdef SZDEBUG
fprintf(stderr, "2267 SFfac %d step %x\n", gSuperFineFactor, gSuperFineSearchStep);
#endif
}

int         
SuperSearch (struct lg_master *pLgMaster,
	     uint32_t *foundX,
	     uint32_t *foundY,
	     double * dMinX,
	     double * dMinY,
	     double * dMaxX,
	     double * dMaxY)
{
  int i;
  int result;
  double sumX;
  double sumY;
  double avgX;
  double avgY;
  uint32_t startX;
  uint32_t startY;
  uint32_t endX;
  uint32_t endY;
  uint32_t startX1;
  uint32_t startX2;
  uint32_t startY1;
  uint32_t startY2;
  uint32_t delNeg;
  uint32_t delPos;
        
  double dblX;
  double dblY;
  double dblX1;
  double dblX2;
  double dblY1;
  double dblY2;
  double Xmid, Ymid, Xlow, Ylow, Xhigh, Yhigh;
  int i1, i2;
  double dCount;
  double dXSpan;
  double dYSpan;
  double dXsteps;
  double dYsteps;
  double Dstep;
  int nXsteps;
  int nYsteps;
  int halfIndex;
  int sweep;
  int firstSuper;

  gLoutIndex  = 0;
  gSuperIndex = 0;
  firstSuper  = 1;

  dXSpan = *dMaxX - *dMinX;
  dYSpan = *dMaxY - *dMinY;
  dXsteps = dXSpan / (double)gSuperFineSearchStep;
  dYsteps = dYSpan / (double)gSuperFineSearchStep;
  nXsteps = (int) dXsteps;
  nYsteps = (int) dYsteps;

  Xmid  = (*dMaxX)/2 + (*dMinX)/2;
  Ymid  = (*dMaxY)/2 + (*dMinY)/2;
  Xlow  = *dMinX - (((*dMaxY) - (*dMinY))/2);
  Xhigh = *dMaxX + (((*dMaxY) - (*dMinY))/2);
  Ylow  = *dMinY - (((*dMaxX) - (*dMinX))/2);
  Yhigh = *dMaxY + (((*dMaxX) - (*dMinX))/2);

  delNeg = (uint32_t)(-(int32_t)gSuperFineSearchStep);
  delPos = (uint32_t)( (int32_t)gSuperFineSearchStep);
  
#ifdef SZDEBUG
  fprintf( stderr, "X Y 3240 " );
  fprintf( stderr, "%lf ", *dMinX / 1000000 );
  fprintf( stderr, "%lf ", *dMaxX / 1000000 );
  fprintf( stderr, "%lf ", *dMinY / 1000000 );
  fprintf( stderr, "%lf ", *dMaxY / 1000000 );
  fprintf( stderr, "n " );
  fprintf( stderr, "%.0lf ", dXsteps );
  fprintf( stderr, "%.0lf ", dYsteps );
  fprintf( stderr, " %x ", gSuperFineSearchStep );
  fprintf( stderr, "\n" );
  fprintf( stderr, "X Y 3241 " );
  fprintf( stderr, "%lf ", Xmid / 1000000 );
  fprintf( stderr, "%lf ", Ymid / 1000000 );
  fprintf( stderr, "%lf ", Xlow / 1000000 );
  fprintf( stderr, "%lf ", Ylow / 1000000 );
  fprintf( stderr, "%lf ", Xhigh / 1000000 );
  fprintf( stderr, "%lf ", Yhigh / 1000000 );
  fprintf( stderr, "\n" );
#endif

  sweep = 1;
  sumX = 0.0;
  sumY = 0.0;
  dCount = 0;
  Dstep = (double)((int32_t)delPos);
  for (dblX = *dMinX; dblX < *dMaxX; dblX +=  Dstep ) {
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      {
#ifdef SZDEBUG
	fprintf( stderr, "2328 kStopWasDone %d %d\n", gSearchFlag, gStopFlag );
#endif
	return kStopWasDone;
      }
    startX  = kMask & ((uint32_t)((int32_t)dblX ));
    startY1 = kMask & ((uint32_t)((int32_t)(*dMinY)));
    startY2 = kMask & ((uint32_t)((int32_t)(*dMaxY)));
    for ( i=1; i<nYsteps; i++ )
      {
	gLout1[i] = 0;
	gLout2[i] = 0;
      }
    // do first set of scans twice
    if (firstSuper == 1)
      {
	firstSuper = 0;
	result = DoLevelSearch(pLgMaster, startX, startY2, 0, delNeg, nYsteps, gLout2);
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;

	if ( result == kStopWasDone ) return result;

	startX  = kMask & ((uint32_t)((int32_t)dblX ));
	startY1 = kMask & ((uint32_t)((int32_t)(*dMinY)));
	result = DoLevelSearch(pLgMaster, startX, startY1, 0, delPos, nYsteps, gLout1);
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;
	if ( result == kStopWasDone ) return result;
      }

    startX  = kMask & ((uint32_t)((int32_t)dblX ));
    startY2 = kMask & ((uint32_t)((int32_t)(*dMaxY)));
    result = DoLevelSearch(pLgMaster, startX, startY2, 0, delNeg, nYsteps, gLout2);
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return kStopWasDone;
    if ( result == kStopWasDone ) return result;
    if ( gLout2[1] > minlevel )
      {
	for ( i=0; i<NZERO && i<nYsteps; i++ )
	  gLout2[i] = 0;
      }
             
    startX  = kMask & ((uint32_t)((int32_t)dblX ));
    startY1 = kMask & ((uint32_t)((int32_t)(*dMinY)));
    result = DoLevelSearch(pLgMaster, startX, startY1, 0, delPos, nYsteps, gLout1);
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return kStopWasDone;
    if ( gLout1[1] > minlevel )
      {
	for ( i=0; i<NZERO && i<nYsteps; i++ )
	  gLout1[i] = 0;
      }
    if ( result == kStopWasDone ) return result;

    for ( i=1; i<nYsteps; i++ )
      {
	i1 = i;
	i2 = nYsteps  - i;
	dblY1 = (double)((int32_t)startY1)
	  + (double)i * (double)((int32_t)delPos);
	dblY2 = (double)((int32_t)startY2)
	  + (double)i * (double)((int32_t)delNeg);
	gSaveMatch[gLoutIndex] = 0;
	gSaveSweep[gLoutIndex] = 0;
	if ( gLout1[i1] > minlevel )
	  {
	    gSaveMatch[gLoutIndex] = 11;
	    gSaveSweep[gLoutIndex] = sweep;
	    gXsuperSave[gSuperIndex] = (int32_t)dblX;
	    gYsuperSave[gSuperIndex] = (int32_t)dblY1;
	    gSuperIndex++;
	  }
	if ( gLout2[i2] > minlevel )
	  {
	    gSaveMatch[gLoutIndex] = 12;
	    gSaveSweep[gLoutIndex] = sweep;
	    gXsuperSave[gSuperIndex] = (int32_t)dblX;
	    gYsuperSave[gSuperIndex] = (int32_t)dblY2;
	    gSuperIndex++;
	  }
	if ( gLout1[i1] > minlevel && gLout2[i2] > minlevel )
	  {
	    gSaveMatch[gLoutIndex] = 1;
	    gSaveSweep[gLoutIndex] = sweep;
	    sumX += dblX;
	    sumY += dblY1;
	    dCount += 1.0;
	    sumX += dblX;
	    sumY += dblY2;
	    dCount += 1.0;
	  }
	gSaveLout1[gLoutIndex] = gLout1[i1];
	gSaveLout2[gLoutIndex] = gLout2[i2];
	gSaveDblX1[gLoutIndex]  = dblX;
	gSaveDblX2[gLoutIndex]  = dblX;
	gSaveDblY1[gLoutIndex]  = dblY1;
	gSaveDblY2[gLoutIndex]  = dblY2;
	gLoutIndex++;
#ifdef SZDEBUG
	fprintf( stderr
		 , "ss3337xyd  %lf %lf %d start %x %x  del %x i %d \n "
		 , dblX / 1000000
		 , dblY1 / 1000000
		 , gLout1[i]
		 , startX
		 , startY1
		 , delPos
		 , i1
		 );
	fprintf( stderr
		 , "ss3338xyd  %lf %lf %d start %x %x  del %x i %d \n "
		 , dblX / 1000000
		 , dblY2 / 1000000
		 , gLout2[i]
		 , startX
		 , startY2
		 , delPos
		 , i2
		 );
#endif
      }
  }

  if ( gMultipleSweeps == 0 ) {
    if ( dCount < 3 ) return kSuperFineNotFound;
  }

  if ( gMultipleSweeps >= 1 ) {
    sweep = 2;
    Dstep = (double)((int32_t)delPos);
    for (dblY = *dMinY; dblY < *dMaxY; dblY +=  Dstep ) {
      if (IfStopThenStopAndNeg1Else0(pLgMaster))
	{
#ifdef SZDEBUG
	  fprintf( stderr, "2328 kStopWasDone %d %d\n", gSearchFlag, gStopFlag );
#endif
	  return kStopWasDone;
	}

      startX2  = kMask & ((uint32_t)((int32_t)(*dMaxX)));
      startY   = kMask & ((uint32_t)((int32_t)   dblY ));
      for ( i=1; i<nXsteps; i++ ) {
	gLout1[i] = 0;
	gLout2[i] = 0;
      }
      result = DoLevelSearch(pLgMaster, startX2, startY, delNeg, 0, nXsteps, gLout2);
      if (IfStopThenStopAndNeg1Else0(pLgMaster))
	return kStopWasDone;
      if ( result == kStopWasDone ) return result;
      if ( gLout2[1] > minlevel ) {
	for ( i=0; i<NZERO && i<nYsteps; i++ ) {
	  gLout2[i] = 0;
	}
      }

      startX1  = kMask & ((uint32_t)((int32_t)(*dMinX)));
      startY   = kMask & ((uint32_t)((int32_t)   dblY ));
      result = DoLevelSearch(pLgMaster, startX1, startY, delPos, 0, nXsteps, gLout1);
      if (IfStopThenStopAndNeg1Else0(pLgMaster))
	return kStopWasDone;
      if ( result == kStopWasDone ) return result;
      if ( gLout1[1] > minlevel ) {
	for ( i=0; i<NZERO && i<nYsteps; i++ ) {
	  gLout1[i] = 0;
	}
      }
      for ( i=1; i<nXsteps; i++ )
	{
	  i1 = i;
	  i2 = nXsteps  - i;
	  dblX1 = (double)((int32_t)startX1)
	    + (double)i * (double)((int32_t)delPos);
	  dblX2 = (double)((int32_t)startX2)
	    + (double)i * (double)((int32_t)delNeg);
	  gSaveMatch[gLoutIndex] = 0;
	  gSaveSweep[gLoutIndex] = 0;
	  if ( gLout1[i1] > minlevel )
	    {
	      gSaveMatch[gLoutIndex] = 11;
	      gSaveSweep[gLoutIndex] = sweep;
	      gXsuperSave[gSuperIndex] = (int32_t)dblX1;
	      gYsuperSave[gSuperIndex] = (int32_t)dblY;
	      gSuperIndex++;
	    }
	  if ( gLout2[i2] > minlevel )
	    {
	      gSaveMatch[gLoutIndex] = 12;
	      gSaveSweep[gLoutIndex] = sweep;
	      gXsuperSave[gSuperIndex] = (int32_t)dblX2;
	      gYsuperSave[gSuperIndex] = (int32_t)dblY;
	      gSuperIndex++;
	    }
	  if ( gLout1[i1] > minlevel && gLout2[i2] > minlevel )
	    {
	      gSaveMatch[gLoutIndex] = 1;
	      gSaveSweep[gLoutIndex] = sweep;
	      sumX += dblX1;
	      sumY += dblY;
	      dCount += 1.0;
	      sumX += dblX2;
	      sumY += dblY;
	      dCount += 1.0;
	    }
	  gSaveLout1[gLoutIndex] = gLout1[i1];
	  gSaveLout2[gLoutIndex] = gLout2[i2];
	  gSaveDblX1[gLoutIndex]  = dblX1;
	  gSaveDblX2[gLoutIndex]  = dblX2;
	  gSaveDblY1[gLoutIndex]  = dblY;
	  gSaveDblY2[gLoutIndex]  = dblY;
	  gLoutIndex++;
#ifdef SZDEBUG
	  fprintf( stderr
		   , "ss3337xyd  %lf %lf %d start %x %x  del %x i %d \n "
		   , dblX / 1000000
		   , dblY1 / 1000000
		   , gLout1[i]
		   , startX
		   , startY1
		   , delPos
		   , i1
		   );
	  fprintf( stderr
		   , "ss3338xyd  %lf %lf %d start %x %x  del %x i %d \n "
		   , dblX / 1000000
		   , dblY2 / 1000000
		   , gLout2[i]
		   , startX
		   , startY2
		   , delPos
		   , i2
		   );
#endif
	}
    }
  }
  
  if ( gMultipleSweeps >= 3 ) {
    sweep = 3;

    // sumX = 0.0;
    // sumY = 0.0;
    // dCount = 0;
    Dstep = (double)((int32_t)delPos);
    for (dblX = *dMaxX; dblX > *dMinX; dblX -=  Dstep ) {
      if (IfStopThenStopAndNeg1Else0(pLgMaster))
	{
#ifdef SZDEBUG
	  fprintf( stderr, "2328 kStopWasDone %d %d\n", gSearchFlag, gStopFlag );
#endif
	  return kStopWasDone;
	}
      startX  = kMask & ((uint32_t)((int32_t)dblX ));
      startY1 = kMask & ((uint32_t)((int32_t)(*dMinY)));
      startY2 = kMask & ((uint32_t)((int32_t)(*dMaxY)));
      for ( i=1; i<nYsteps; i++ ) {
	gLout1[i] = 0;
	gLout2[i] = 0;
      }
      result = DoLevelSearch(pLgMaster, startX, startY2, 0, delNeg, nYsteps, gLout2);
      if (IfStopThenStopAndNeg1Else0(pLgMaster))
	return kStopWasDone;
      if ( result == kStopWasDone ) return result;
      if ( gLout2[1] > minlevel ) {
	for ( i=0; i<NZERO && i<nYsteps; i++ ) {
	  gLout2[i] = 0;
	}
      }
      startX  = kMask & ((uint32_t)((int32_t)dblX ));
      startY1 = kMask & ((uint32_t)((int32_t)(*dMinY)));
      result = DoLevelSearch(pLgMaster, startX, startY1, 0, delPos, nYsteps, gLout1);
      if (IfStopThenStopAndNeg1Else0(pLgMaster))
	return kStopWasDone;
      if ( result == kStopWasDone ) return result;
      if ( gLout1[1] > minlevel ) {
	for ( i=0; i<NZERO && i<nYsteps; i++ ) {
	  gLout1[i] = 0;
	}
      }
      for ( i=1; i<nYsteps; i++ ) {
	i1 = i;
	i2 = nYsteps  - i;
	dblY1 = (double)((int32_t)startY1)
	  + (double)i * (double)((int32_t)delPos);
	dblY2 = (double)((int32_t)startY2)
	  + (double)i * (double)((int32_t)delNeg);
	gSaveMatch[gLoutIndex] = 0;
	gSaveSweep[gLoutIndex] = 0;
	if ( gLout1[i1] > minlevel ) {
	  gSaveMatch[gLoutIndex] = 11;
	  gSaveSweep[gLoutIndex] = sweep;
	  gXsuperSave[gSuperIndex] = (int32_t)dblX;
	  gYsuperSave[gSuperIndex] = (int32_t)dblY1;
	  gSuperIndex++;
	}
	if ( gLout2[i2] > minlevel ) {
	  gSaveMatch[gLoutIndex] = 12;
	  gSaveSweep[gLoutIndex] = sweep;
	  gXsuperSave[gSuperIndex] = (int32_t)dblX;
	  gYsuperSave[gSuperIndex] = (int32_t)dblY2;
	  gSuperIndex++;
	}
	if ( gLout1[i1] > minlevel && gLout2[i2] > minlevel ) {
	  gSaveMatch[gLoutIndex] = 1;
	  gSaveSweep[gLoutIndex] = sweep;
	  sumX += dblX;
	  sumY += dblY1;
	  dCount += 1.0;
	  sumX += dblX;
	  sumY += dblY2;
	  dCount += 1.0;
	}
	gSaveLout1[gLoutIndex] = gLout1[i1];
	gSaveLout2[gLoutIndex] = gLout2[i2];
	gSaveDblX1[gLoutIndex]  = dblX;
	gSaveDblX2[gLoutIndex]  = dblX;
	gSaveDblY1[gLoutIndex]  = dblY1;
	gSaveDblY2[gLoutIndex]  = dblY2;
	gLoutIndex++;
#ifdef SZDEBUG
	fprintf( stderr
		 , "ss3337xyd  %lf %lf %d start %x %x  del %x i %d \n "
		 , dblX / 1000000
		 , dblY1 / 1000000
		 , gLout1[i]
		 , startX
		 , startY1
		 , delPos
		 , i1
		 );
	fprintf( stderr
		 , "ss3338xyd  %lf %lf %d start %x %x  del %x i %d \n "
		 , dblX / 1000000
		 , dblY2 / 1000000
		 , gLout2[i]
		 , startX
		 , startY2
		 , delPos
		 , i2
		 );
#endif
      }
    }
  }
  if ( gMultipleSweeps >= 4 ) {
    sweep = 4;
    Dstep = (double)((int32_t)delPos);
    for (dblY = *dMaxY; dblY > *dMinY; dblY -=  Dstep ) {
      if (IfStopThenStopAndNeg1Else0(pLgMaster))
	{
#ifdef SZDEBUG
	  fprintf( stderr, "2328 kStopWasDone %d %d\n", gSearchFlag, gStopFlag );
#endif
	  return kStopWasDone;
	}

      startX2  = kMask & ((uint32_t)((int32_t)(*dMaxX)));
      startY   = kMask & ((uint32_t)((int32_t)   dblY ));
      for ( i=1; i<nYsteps; i++ ) {
	gLout1[i] = 0;
	gLout2[i] = 0;
      }
      result = DoLevelSearch(pLgMaster, startX2, startY,
			     delNeg, 0, nXsteps, gLout2);
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;
	if ( result == kStopWasDone ) return result;
	if ( gLout2[1] > minlevel ) {
	  for ( i=0; i<NZERO && i<nYsteps; i++ ) {
	    gLout2[i] = 0;
	  }
	}

	startX1  = kMask & ((uint32_t)((int32_t)(*dMinX)));
	startY   = kMask & ((uint32_t)((int32_t)   dblY ));
	result = DoLevelSearch(pLgMaster, startX1
				, startY
				, delPos
				, 0
				, nXsteps
				, gLout1
				);
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;
	if ( result == kStopWasDone ) return result;
	if ( gLout1[1] > minlevel ) {
	  for ( i=0; i<NZERO && i<nYsteps; i++ ) {
	    gLout1[i] = 0;
	  }
	}

	for ( i=1; i<nXsteps; i++ ) {
	  i1 = i;
	  i2 = nXsteps  - i;
	  dblX1 = (double)((int32_t)startX1)
	    + (double)i * (double)((int32_t)delPos);
	  dblX2 = (double)((int32_t)startX2)
	    + (double)i * (double)((int32_t)delNeg);
	  gSaveMatch[gLoutIndex] = 0;
	  gSaveSweep[gLoutIndex] = 0;
	  if ( gLout1[i1] > minlevel ) {
	    gSaveMatch[gLoutIndex] = 11;
	    gSaveSweep[gLoutIndex] = sweep;
	    gXsuperSave[gSuperIndex] = (int32_t)dblX1;
	    gYsuperSave[gSuperIndex] = (int32_t)dblY;
	    gSuperIndex++;
	  }
	  if ( gLout2[i2] > minlevel ) {
	    gSaveMatch[gLoutIndex] = 12;
	    gSaveSweep[gLoutIndex] = sweep;
	    gXsuperSave[gSuperIndex] = (int32_t)dblX2;
	    gYsuperSave[gSuperIndex] = (int32_t)dblY;
	    gSuperIndex++;
	  }
	  if ( gLout1[i1] > minlevel && gLout2[i2] > minlevel ) {
	    gSaveMatch[gLoutIndex] = 1;
	    gSaveSweep[gLoutIndex] = sweep;
	    sumX += dblX1;
	    sumY += dblY;
	    dCount += 1.0;
	    sumX += dblX2;
	    sumY += dblY;
	    dCount += 1.0;
	  }
	  gSaveLout1[gLoutIndex] = gLout1[i1];
	  gSaveLout2[gLoutIndex] = gLout2[i2];
	  gSaveDblX1[gLoutIndex]  = dblX1;
	  gSaveDblX2[gLoutIndex]  = dblX2;
	  gSaveDblY1[gLoutIndex]  = dblY;
	  gSaveDblY2[gLoutIndex]  = dblY;
	  gLoutIndex++;
#ifdef SZDEBUG
	  fprintf( stderr
		   , "ss3337xyd  %lf %lf %d start %x %x  del %x i %d \n "
		   , dblX / 1000000
		   , dblY1 / 1000000
		   , gLout1[i]
		   , startX
		   , startY1
		   , delPos
		   , i1
		   );
	  fprintf( stderr
		   , "ss3338xyd  %lf %lf %d start %x %x  del %x i %d \n "
		   , dblX / 1000000
		   , dblY2 / 1000000
		   , gLout2[i]
		   , startX
		   , startY2
		   , delPos
		   , i2
		   );
#endif
	}
    }
  }
  if ( gMultipleSweeps >= 5 )
    {
      sweep = 5;
      //  start of diagonals
      sumX = 0.0;
      sumY = 0.0;
      Dstep = (double)((int32_t)delPos);
      for ( dblX =   Xlow,   dblY = Ymid 
	      ; dblX <   Xmid && dblY < Yhigh
	      ; dblX +=  Dstep,  dblY += Dstep
	    )
	{
#ifdef SZDEBUG
	  fprintf( stderr, "2989 xy %lf %lf %lf\n", dblX, dblY, Dstep );
#endif
	  if (IfStopThenStopAndNeg1Else0(pLgMaster))
	    {
#ifdef SZDEBUG
	      fprintf( stderr, "2328 kStopWasDone %d %d\n", gSearchFlag, gStopFlag );
#endif
	      return kStopWasDone;
	    }

	  startX = kMask & ((uint32_t)((int32_t)dblX));
	  startY = kMask & ((uint32_t)((int32_t)dblY));
#ifdef SZDEBUG
	  fprintf( stderr, "2398 xy %x %x\n", startX, startY );
#endif
	for ( i=1; i<nYsteps; i++ )
	  {
	    gLout1[i] = 0;
	    gLout2[i] = 0;
	  }
	result = DoLevelSearch(pLgMaster, startX
				, startY
				, delPos
				, delNeg
				, nYsteps
				, gLout1
				);
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;
	if ( result == kStopWasDone ) return result;
	if ( gLout1[1] > minlevel )
	  {
	    for ( i=0; i<NZERO && i<nYsteps; i++ )
	    gLout1[i] = 0;
	  }
         
	for ( i=1; i<nYsteps; i++ )
	  {
	    i1 = i;
	    i2 = nYsteps  - i;
	    dblX1  = (double)((int32_t)startX)
	      + (double)i * (double)((int32_t)delPos);
	    dblY1  = (double)((int32_t)startY)
	      + (double)i * (double)((int32_t)delNeg);
	    gSaveMatch[gLoutIndex] = 0;
	    gSaveSweep[gLoutIndex] = 0;
	    if ( gLout1[i1] > minlevel )
	      {
		gSaveMatch[gLoutIndex] = 11;
		gSaveSweep[gLoutIndex] = sweep;
		gXsuperSave[gSuperIndex] = (int32_t)dblX1;
		gYsuperSave[gSuperIndex] = (int32_t)dblY1;
		gSuperIndex++;
	      }

	    gSaveLout1[gLoutIndex] = gLout1[i1];
	    gSaveLout2[gLoutIndex] = gLout1[i1];
	    gSaveDblX1[gLoutIndex]  = dblX1;
	    gSaveDblY1[gLoutIndex]  = dblY1;
	    gSaveDblX2[gLoutIndex]  = dblX1;
	    gSaveDblY2[gLoutIndex]  = dblY1;
	    gLoutIndex++;
	  }

	endX = kMask & ((uint32_t)((int32_t)(dblX+Dstep*nYsteps)));
	endY = kMask & ((uint32_t)((int32_t)(dblY-Dstep*nYsteps)));
	result = DoLevelSearch(pLgMaster, endX
				, endY
				, delNeg
				, delPos
				, nYsteps
				, gLout1
				);
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;
	if ( gLout1[1] > minlevel )
	  {
	    for ( i=0; i<NZERO && i<nYsteps; i++ )
	    gLout1[i] = 0;
	  }
	if ( result == kStopWasDone ) return result;
	for ( i=1; i<nYsteps; i++ )
	  {
	    i1 = i;
	    i2 = nYsteps  - i;
	    dblX1  = (double)((int32_t)endX)
	      + (double)i * (double)((int32_t)delNeg);
	    dblY1  = (double)((int32_t)endY)
	      + (double)i * (double)((int32_t)delPos);
	    gSaveMatch[gLoutIndex] = 0;
	    gSaveSweep[gLoutIndex] = 0;
	    if ( gLout1[i1] > minlevel )
	      {
		gSaveMatch[gLoutIndex] = 11;
		gSaveSweep[gLoutIndex] = sweep;
		gXsuperSave[gSuperIndex] = (int32_t)dblX1;
		gYsuperSave[gSuperIndex] = (int32_t)dblY1;
		gSuperIndex++;
	      }
	  
	    gSaveLout1[gLoutIndex] = gLout1[i1];
	    gSaveDblX1[gLoutIndex]  = dblX1;
	    gSaveDblY1[gLoutIndex]  = dblY1;
	    gSaveLout2[gLoutIndex] = gLout1[i1];
	    gSaveDblX2[gLoutIndex]  = dblX1;
	    gSaveDblY2[gLoutIndex]  = dblY1;
	    gLoutIndex++;
	  }
	}
    }

  if ( gMultipleSweeps >= 6 )
    {
      sweep = 6;
      for ( dblX =   Xhigh,  dblY = Ymid 
	      ; dblX >   Xmid && dblY < Yhigh
	      ; dblX -=  Dstep,  dblY -= Dstep
            )
	{
#ifdef SZDEBUG
	  fprintf( stderr, "3042 xy %lf %lf %lf\n", dblX, dblY, Dstep );
#endif
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  {
#ifdef SZDEBUG
	    fprintf( stderr, "2328 kStopWasDone %d %d\n", gSearchFlag, gStopFlag );
#endif
	    return kStopWasDone;
	  }

	startX = kMask & ((uint32_t)((int32_t)dblX));
	startY = kMask & ((uint32_t)((int32_t)dblY));
#ifdef SZDEBUG
	fprintf( stderr, "3039 xy %x %x\n", startX, startY );
#endif
	for ( i=1; i<nYsteps; i++ )
	  {
	    gLout1[i] = 0;
	    gLout2[i] = 0;
	  }
	result = DoLevelSearch(pLgMaster, startX
				, startY
				, delNeg
				, delPos
				, nYsteps
				, gLout1
				);
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;
	if ( gLout1[1] > minlevel )
	  {
	    for ( i=0; i<NZERO && i<nYsteps; i++ )
	      gLout1[i] = 0;
	  }
	if ( result == kStopWasDone ) return result;
	for ( i=1; i<nYsteps; i++ )
	  {
	    i1 = i;
	    i2 = nYsteps  - i;
	    dblX1  = (double)((int32_t)startX)
	      + (double)i * (double)((int32_t)delNeg);
	    dblY1  = (double)((int32_t)startY)
	      + (double)i * (double)((int32_t)delPos);
	    gSaveMatch[gLoutIndex] = 0;
	    gSaveSweep[gLoutIndex] = 0;
	    if ( gLout1[i1] > minlevel )
	      {
		gSaveMatch[gLoutIndex] = 11;
		gSaveSweep[gLoutIndex] = sweep;
		gXsuperSave[gSuperIndex] = (int32_t)dblX1;
		gYsuperSave[gSuperIndex] = (int32_t)dblY1;
		gSuperIndex++;
	      }

	    gSaveLout1[gLoutIndex] = gLout1[i1];
	    gSaveDblX1[gLoutIndex]  = dblX1;
	    gSaveDblY1[gLoutIndex]  = dblY1;
	    gSaveLout2[gLoutIndex] = gLout1[i1];
	    gSaveDblX2[gLoutIndex]  = dblX1;
	    gSaveDblY2[gLoutIndex]  = dblY1;
	    gLoutIndex++;
	  }

	endX = kMask & ((uint32_t)((int32_t)(dblX-Dstep*nYsteps)));
	endY = kMask & ((uint32_t)((int32_t)(dblY+Dstep*nYsteps)));
	result = DoLevelSearch(pLgMaster, endX
				, endY
				, delPos
				, delNeg
				, nYsteps
				, gLout1
				);
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;
	if ( gLout1[1] > minlevel )
	  {
	    for ( i=0; i<NZERO && i<nYsteps; i++ )
	      gLout1[i] = 0;
	  }
	if ( result == kStopWasDone ) return result;
	for ( i=1; i<nYsteps; i++ )
	  {
	    i1 = i;
	    i2 = nYsteps  - i;
	    dblX1  = (double)((int32_t)endX)
	      + (double)i * (double)((int32_t)delPos);
	    dblY1  = (double)((int32_t)endY)
	      + (double)i * (double)((int32_t)delNeg);
	    gSaveMatch[gLoutIndex] = 0;
	    gSaveSweep[gLoutIndex] = 0;
	    if ( gLout1[i1] > minlevel )
	      {
		gSaveMatch[gLoutIndex] = 11;
		gSaveSweep[gLoutIndex] = sweep;
		gXsuperSave[gSuperIndex] = (int32_t)dblX1;
		gYsuperSave[gSuperIndex] = (int32_t)dblY1;
		gSuperIndex++;
	      }

	    gSaveLout1[gLoutIndex] = gLout1[i1];
	    gSaveDblX1[gLoutIndex]  = dblX1;
	    gSaveDblY1[gLoutIndex]  = dblY1;
	    gSaveLout2[gLoutIndex] = gLout1[i1];
	    gSaveDblX2[gLoutIndex]  = dblX1;
	    gSaveDblY2[gLoutIndex]  = dblY1;
	    gLoutIndex++;
	  }
        }
    }

  if ( gMultipleSweeps >= 7 )
    {
      sweep = 7;
      for (dblX = Xmid, dblY = Yhigh; ((dblX < Xhigh) && (dblY > Ymid)); dblX += Dstep, dblY -= Dstep)
	{
#ifdef SZDEBUG
	  fprintf( stderr, "3095 xy %lf %lf %lf\n", dblX, dblY, Dstep );
#endif
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  {
#ifdef SZDEBUG
	    fprintf( stderr, "2328 kStopWasDone %d %d\n", gSearchFlag, gStopFlag );
#endif
	    return kStopWasDone;
	  }

	startX = kMask & ((uint32_t)((int32_t)dblX));
	startY = kMask & ((uint32_t)((int32_t)dblY));
#ifdef SZDEBUG
	fprintf( stderr, "3089 xy %x %x\n", startX, startY );
#endif
	for ( i=1; i<nYsteps; i++ )
	  {
	    gLout1[i] = 0;
	    gLout2[i] = 0;
	  }
	result = DoLevelSearch(pLgMaster, startX
				, startY
				, delNeg
				, delNeg
				, nYsteps
				, gLout1
				);
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;
	if ( gLout1[1] > minlevel )
	  {
	    for ( i=0; i<NZERO && i<nYsteps; i++)
	      gLout1[i] = 0;
	  }
	if ( result == kStopWasDone ) return result;
	for ( i=1; i<nYsteps; i++ )
	  {
	    i1 = i;
	    i2 = nYsteps  - i;
	    dblX1  = (double)((int32_t)startX)
	      + (double)i * (double)((int32_t)delNeg);
	    dblY1  = (double)((int32_t)startY)
	      + (double)i * (double)((int32_t)delNeg);
	    gSaveMatch[gLoutIndex] = 0;
	    gSaveSweep[gLoutIndex] = 0;
	    if ( gLout1[i1] > minlevel )
	      {
		gSaveMatch[gLoutIndex] = 11;
		gSaveSweep[gLoutIndex] = sweep;
		gXsuperSave[gSuperIndex] = (int32_t)dblX1;
		gYsuperSave[gSuperIndex] = (int32_t)dblY1;
		gSuperIndex++;
	      }

	    gSaveLout1[gLoutIndex] = gLout1[i1];
	    gSaveDblX1[gLoutIndex]  = dblX1;
	    gSaveDblY1[gLoutIndex]  = dblY1;
	    gSaveLout2[gLoutIndex] = gLout1[i1];
	    gSaveDblX2[gLoutIndex]  = dblX1;
	    gSaveDblY2[gLoutIndex]  = dblY1;
	    gLoutIndex++;
	  }

	endX = kMask & ((uint32_t)((int32_t)(dblX-Dstep*nYsteps)));
	endY = kMask & ((uint32_t)((int32_t)(dblY-Dstep*nYsteps)));
	result = DoLevelSearch(pLgMaster, endX
				, endY
				, delPos
				, delPos
				, nYsteps
				, gLout1
				);
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;
	if ( gLout1[1] > minlevel )
	  {
	    for ( i=0; i<NZERO && i<nYsteps; i++ )
	      gLout1[i] = 0;
	  }
	if ( result == kStopWasDone ) return result;
	for ( i=1; i<nYsteps; i++ )
	  {
	    i1 = i;
	    i2 = nYsteps  - i;
	    dblX1  = (double)((int32_t)endX)
	      + (double)i * (double)((int32_t)delPos);
	    dblY1  = (double)((int32_t)endY)
	      + (double)i * (double)((int32_t)delPos);
	    gSaveMatch[gLoutIndex] = 0;
	    gSaveSweep[gLoutIndex] = 0;
	    if ( gLout1[i1] > minlevel )
	      {
		gSaveMatch[gLoutIndex] = 11;
		gSaveSweep[gLoutIndex] = sweep;
		gXsuperSave[gSuperIndex] = (int32_t)dblX1;
		gYsuperSave[gSuperIndex] = (int32_t)dblY1;
		gSuperIndex++;
	      }

	    gSaveLout1[gLoutIndex] = gLout1[i1];
	    gSaveDblX1[gLoutIndex]  = dblX1;
	    gSaveDblY1[gLoutIndex]  = dblY1;
	    gSaveLout2[gLoutIndex] = gLout1[i1];
	    gSaveDblX2[gLoutIndex]  = dblX1;
	    gSaveDblY2[gLoutIndex]  = dblY1;
	    gLoutIndex++;
	  }
        }
    }

  if ( gMultipleSweeps >= 8 )
    {
      sweep = 8;

      for (dblX = Xmid, dblY = Ylow; ((dblX >Xlow) && (dblY<Ymid)); dblX -= Dstep, dblY += Dstep)
	{
#ifdef SZDEBUG
	  fprintf( stderr, "3148 xy %lf %lf %lf\n", dblX, dblY, Dstep );
#endif
	  if (IfStopThenStopAndNeg1Else0(pLgMaster))
	    {
#ifdef SZDEBUG
	      fprintf( stderr, "2328 kStopWasDone %d %d\n", gSearchFlag, gStopFlag );
#endif
	      return kStopWasDone;
	    }

	  startX = kMask & ((uint32_t)((int32_t)dblX));
	  startY = kMask & ((uint32_t)((int32_t)dblY));
#ifdef SZDEBUG
	  fprintf( stderr, "3138 xy %x %x\n", startX, startY );
#endif
	  for ( i=1; i<nYsteps; i++ )
	    {
	      gLout1[i] = 0;
	      gLout2[i] = 0;
	    }
	  result = DoLevelSearch(pLgMaster, startX
				  , startY
				  , delPos
				  , delPos
				  , nYsteps
				  , gLout1
				  );
	  if (IfStopThenStopAndNeg1Else0(pLgMaster))
	    return kStopWasDone;

	  if ( gLout1[1] > minlevel)
	    {
	      for ( i=0; i<NZERO && i<nYsteps; i++ )
	      gLout1[i] = 0;
	    }
	  if ( result == kStopWasDone ) return result;
	  for ( i=1; i<nYsteps; i++ )
	    {
	      i1 = i;
	      i2 = nYsteps  - i;
	      dblX1  = (double)((int32_t)startX)
		+ (double)i * (double)((int32_t)delPos);
	      dblY1  = (double)((int32_t)startY)
		+ (double)i * (double)((int32_t)delPos);
	      gSaveMatch[gLoutIndex] = 0;
	      gSaveSweep[gLoutIndex] = 0;
	      if ( gLout1[i1] > minlevel )
		{
		  gSaveMatch[gLoutIndex] = 11;
		  gSaveSweep[gLoutIndex] = sweep;
		  gXsuperSave[gSuperIndex] = (int32_t)dblX1;
		  gYsuperSave[gSuperIndex] = (int32_t)dblY1;
		  gSuperIndex++;
		}

	      gSaveLout1[gLoutIndex] = gLout1[i1];
	      gSaveDblX1[gLoutIndex]  = dblX1;
	      gSaveDblY1[gLoutIndex]  = dblY1;
	      gSaveLout2[gLoutIndex] = gLout1[i1];
	      gSaveDblX2[gLoutIndex]  = dblX1;
	      gSaveDblY2[gLoutIndex]  = dblY1;
	      gLoutIndex++;
	    }

	  endX = kMask & ((uint32_t)((int32_t)(dblX+Dstep*nYsteps)));
	  endY = kMask & ((uint32_t)((int32_t)(dblY+Dstep*nYsteps)));
	  result = DoLevelSearch(pLgMaster, endX
				  , endY
				  , delNeg
				  , delNeg
				  , nYsteps
				  , gLout1
				  );
	  if (IfStopThenStopAndNeg1Else0(pLgMaster))
	    return kStopWasDone;

	  if ( gLout1[1] > minlevel ) {
	    for ( i=0; i<NZERO && i<nYsteps; i++ )
                     gLout1[i] = 0;
	  }
	  if ( result == kStopWasDone ) return result;
	  for ( i=1; i<nYsteps; i++ )
	    {
	      i1 = i;
	      i2 = nYsteps  - i;
	      dblX1  = (double)((int32_t)endX)
		+ (double)i * (double)((int32_t)delNeg);
	      dblY1  = (double)((int32_t)endY)
		+ (double)i * (double)((int32_t)delNeg);
	      gSaveMatch[gLoutIndex] = 0;
	      gSaveSweep[gLoutIndex] = 0;
	      if ( gLout1[i1] > minlevel)
		{
		  gSaveMatch[gLoutIndex] = 11;
		  gSaveSweep[gLoutIndex] = sweep;
		  gXsuperSave[gSuperIndex] = (int32_t)dblX1;
		  gYsuperSave[gSuperIndex] = (int32_t)dblY1;
		  gSuperIndex++;
		}

	      gSaveLout1[gLoutIndex] = gLout1[i1];
	      gSaveDblX1[gLoutIndex]  = dblX1;
	      gSaveDblY1[gLoutIndex]  = dblY1;
	      gSaveLout2[gLoutIndex] = gLout1[i1];
	      gSaveDblX2[gLoutIndex]  = dblX1;
	      gSaveDblY2[gLoutIndex]  = dblY1;
	      gLoutIndex++;
	    }
        }
    }

  if ( dCount < 3 ) return kSuperFineNotFound;
  avgX = sumX / dCount;
  avgY = sumY / dCount;

  *foundX = (uint32_t)((int32_t)avgX);
  *foundY = (uint32_t)((int32_t)avgY);

  if ( gSuperIndex > 3 )
    {
      halfIndex = gSuperIndex / 2;
      qsort(gXsuperSave, gSuperIndex, sizeof(int32_t), int32_tsort );
      qsort(gYsuperSave, gSuperIndex, sizeof(int32_t), int32_tsort );
      avgX = (double)(gXsuperSave[halfIndex]);
      avgY = (double)(gYsuperSave[halfIndex]);
#if defined(LOUTDEBUG) || defined(SZDEBUG)
      fprintf( stderr, "super2655 %d %lf %lf\n", gSearchCurrentSensor, avgX, avgY );
#endif
      *foundX = (uint32_t)((int32_t)avgX);
      *foundY = (uint32_t)((int32_t)avgY);
    }
        
  return 0;
}

int
int32_tsort( const void *elem1, const void *elem2 )
{
    uint32_t numone, numtwo;

    numone = *(const uint32_t *)elem1;
    numtwo = *(const uint32_t *)elem2;

    if ( numone < numtwo ) return -1;
    if ( numone > numtwo ) return  1;

    return 0;
}
