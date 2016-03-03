/*
static char rcsid[] = "$Id$";

*/
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <syslog.h>
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
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "AppCommon.h"
#include "SensorSearch.h"
#include "QuickCheckManager.h"
#include "Protocol.h"
#include "comm_loop.h"
#include "DoCoarseScan.h"
#include "DoCoarseScan2.h"
#include "LaserInterface.h"

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
	      int32_t *firstX,
	      int32_t *lastX,
	      int32_t *firstY,
	      int32_t *lastY,
	      int32_t *medianX,
	      int32_t *medianY,
	      int32_t currentX,
	      int32_t currentY,
	      uint32_t xStep,
	      uint32_t yStep,
	      uint32_t nSteps);

static int DoFineLevel(struct lg_master *pLgMaster, int32_t *foundX,
		       int32_t *foundY, double *outMinX, double *outMinY,
		       double *outMaxX, double *outMaxY)
;

static int
DoSuperLevelSearch(struct lg_master *pLgMaster, int32_t *foundX, int32_t *foundY,
		     double * dXmin, double * dYmin, double * dXmax, double * dYmax);

static int        
DoRedundantSuperFineSearch(struct lg_master *pLgMaster, int32_t *foundX, int32_t *foundY,
			    double * dMinX, double * dMinY,
			    double * dMaxX, double * dMaxY);

static int SuperSearch(struct lg_master *pLgMaster, int32_t *foundX, int32_t *foundY,
			double *dMinX, double *dMinY, double *dMaxX, double * dMaxY);

void limitCalc(int32_t centerX, int32_t centerY, int32_t *eolXNeg,
	       int32_t *eolXPos, int32_t *eolYNeg, int32_t *eolYPos);

static int  CoarseLeg(struct lg_master *pLgMaster,
		      int32_t X,    int32_t Y,
		      int32_t delX, int32_t delY,
		      uint32_t nSteps,
		      int32_t * foundX, int32_t * foundY );
static int  FakeLeg( struct lg_master *pLgMaster, int32_t X,    int32_t Y,
                    int32_t delX, int32_t delY,
                    uint32_t nSteps,
                    int32_t * foundX, int32_t * foundY );

static int DoQuickFineSearch(struct lg_master *pLgMaster, int32_t *foundX, int32_t *foundY );
int compare ( const void * a, const void * b );

int32_t gQCerrors[6] = {0,0,0,0,0,0};
double gFixX[ kNumberDrift ];
double gFixY[ kNumberDrift ];
double gDriftX[ kNumberDrift ];
double gDriftY[ kNumberDrift ];

#define kMaxOutLength                       0x80000
#define kMask                               0xFFFF
#define kMaxNumberX                         0x7FFF
#define kMinNumberX                         0x8000
#define kMaxNumberY                         0x7FFF
#define kMinNumberY                         0x8000
#define kFineSearchStep                     0x0002
#define kFineSearchSpanSteps                70
#define kSuperFineSearchStep                0x8000
#define kSuperFineSearchSpanSteps           256
#define kNumberOfCrossesToAverage           7
#define kNumberOfCrosses                    7
#define kNumberOfSuperCrossesToAverage      2

int32_t gDELLEV =  60;
unsigned char * gOut;
int32_t * gLout;
int32_t * gLout1;
int32_t * gLout2;
int32_t * gLsort;
int32_t * xPosSave;
int32_t * xNegSave;
int32_t * yPosSave;
int32_t * yNegSave;
int32_t * xPosEdge;
int32_t * xNegEdge;
int32_t * yPosEdge;
int32_t * yNegEdge;
int32_t * xPosSuper;
int32_t * xNegSuper;
int32_t * yPosSuper;
int32_t * yNegSuper;
int32_t * xPosESuper;
int32_t * xNegESuper;
int32_t * yPosESuper;
int32_t * yNegESuper;
int32_t * xPosPosition;
int32_t * yPosPosition;
int32_t * xNegPosition;
int32_t * yNegPosition;
double * xpResArr;
double * xnResArr;
double * ypResArr;
double * ynResArr;
double g_aX = 0.0;
double g_bX = 1.0;
double g_aY = 0.0;
double g_bY = 1.0;
double xPosAvg;
double xNegAvg;
double yPosAvg;
double yNegAvg;
int32_t  xPosESum;
int32_t  xNegESum;
int32_t  yPosESum;
int32_t  yNegESum;
int32_t  XpEdge;
int32_t  XnEdge;
int32_t  YpEdge;
int32_t  YnEdge;
int gTargetDrift = 0;
int gCentroid;
uint32_t gSuperFineSearchStep = 0x8000;
uint32_t gSuperFineFactor = 1;

int QuickCheckASensor(struct lg_master *pLgMaster, int32_t centerX, int32_t centerY)
{
	struct lg_xydata  xydata;
        int theResult;
        int i;
	int nTries;
        int nSuccess;
        int nSteps;
        int32_t tempX, tempY;
	
	memset((char *)&xydata, 0, sizeof(struct lg_xydata));
	
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
	xydata.xdata = (centerX & kMaxSigned);
	xydata.ydata = (centerY & kMaxSigned);
        move_dark(pLgMaster, (struct lg_xydata *)&xydata);
        SearchBeamOn(pLgMaster);
#ifdef DEBUG
	GoToRaw(pLgMaster, (struct lg_xydata *)&xydata);
        usleep( 1000000U );
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

int QuickCheckOne(struct lg_master *pLgMaster, int32_t centerX, int32_t centerY,
		  int32_t *foundX, int32_t *foundY)
{
	struct lg_xydata  xydata;
        int theResult;
        int i;
	int nTries;
        int nSuccess;
        int nSteps;
        int32_t tempX, tempY;

	memset((char *)&xydata, 0, sizeof(struct lg_xydata));
	
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
	xydata.xdata = (centerX & kMaxSigned);
	xydata.ydata = (centerY & kMaxSigned);
        move_dark(pLgMaster, (struct lg_xydata *)&xydata);
        SearchBeamOn  (pLgMaster);
#ifdef DEBUG
	GoToRaw(pLgMaster, (struct lg_xydata *)&xydata);
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
        int32_t startX, int32_t startY,
        int32_t *foundX, int32_t *foundY )
{
    struct lg_xydata   xydata;
    struct lg_xydata   xydelta;
    int32_t f1x, f1y, f2x, f2y;
    int32_t tempX, tempY;
    int32_t currentX, currentY;
    int32_t eolXPos, eolYPos;
    int32_t eolXNeg, eolYNeg;
    int32_t posStepSize, negStepSize;
    uint32_t xSteps, ySteps;
    int32_t delX, delY, centX, centY;
    uint32_t nSteps;
    int32_t avgX, avgY;
    int32_t HatchCount;
    int32_t i;
    int int32_tFactor;
    int theResult;
    double dMinX;
    double dMinY;
    double dMaxX;
    double dMaxY;
        
    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    memset((char *)&xydelta, 0, sizeof(struct lg_xydata));
	
    firstCoarse = 1;
    currentX = startX & kMask;
    currentY = startY & kMask;

    xydata.xdata = (currentX & kMaxSigned);
    xydata.ydata = (currentY & kMaxSigned);
    move_dark(pLgMaster, (struct lg_xydata *)&xydata);
    *foundX = 0x0;
    *foundY = 0x0;
    eolXPos = currentX;
    eolYPos = currentY;
    eolXNeg = currentX;
    eolYNeg = currentY;
    SearchBeamOn(pLgMaster);
    doRestartTimer(pLgMaster);
    pLgMaster->gHeaderSpecialByte = 0x40;
    if (LongOrShortThrowSearch == 1)
      {
	for (int32_tFactor = 1; int32_tFactor <= 4 ; int32_tFactor *= 2)
	  {
	    minlevel = gDELLEV;
	    theResult = DoCoarseScan(pLgMaster, currentX, currentY, 0x40000,
                                     (32 * int32_tFactor), &f1x, &f1y);
	    if (theResult == kStopWasDone)
	      {
		SearchBeamOff(pLgMaster);
		return(theResult);
	      }
	    if (theResult == kCoarseNotFound)
	      continue;

	    theResult = DoCoarseScan2(pLgMaster, currentX, currentY, 0x40000,
				      (32 * int32_tFactor), &f2x, &f2y);
	    if (theResult == kStopWasDone)
	      {
		SearchBeamOff(pLgMaster);
		return(theResult);
	      }
	    if (theResult == kCoarseNotFound)
	      continue;

	    avgX = f2x;
	    avgY = f1y;
	    theResult = DoFineLevel(pLgMaster, &avgX, &avgY, &dMinX, &dMinY,
				    &dMaxX, &dMaxY);
	    if (theResult == kStopWasDone)
	      {
		SearchBeamOff(pLgMaster);
		return(theResult);
	      }
	    if (theResult == kFineNotFound)
	      continue;
	    theResult = DoRedundantSuperFineSearch(pLgMaster, &avgX, &avgY, &dMinX,
						   &dMinY, &dMaxX, &dMaxY);
	    if (theResult == kStopWasDone)
	      {
		SearchBeamOff(pLgMaster);
		return(theResult);
	      }
	    if (theResult == kSuperFineNotFound)
	      continue;
	    if (theResult == 0)
	      break;
	  }
	if (theResult)
	  return(theResult);
	*foundX = avgX;
	*foundY = avgY;
	return 0;
      }
    HatchCount       =  gHatchFactor  / gCoarse2Factor;
    gNumberOfSpirals =  gSpiralFactor / gCoarse2Factor;

    theResult = kCoarseNotFound;
    if (gCentroid == 1)
      {
	if (currentX >= (int32_t)(kMaxNumberX - (HatchCount * pLgMaster->gCoarse2SearchStep)))
	  eolXPos = kMaxNumberX;
	else
	  eolXPos =  currentX + (HatchCount * pLgMaster->gCoarse2SearchStep);
	if (currentX <= (int32_t)(kMinNumberX + (HatchCount * pLgMaster->gCoarse2SearchStep)))
	  eolXNeg   = kMinNumberX;
	else
	  eolXNeg =  currentX - (HatchCount * pLgMaster->gCoarse2SearchStep);
	if (currentY >= (int32_t)(kMaxNumberY - (HatchCount * pLgMaster->gCoarse2SearchStep)))
	  eolYPos   =  kMaxNumberY;
	else
	  eolYPos =  currentY + (HatchCount * pLgMaster->gCoarse2SearchStep);
	if (currentY <= (int32_t)(kMinNumberY + (HatchCount * pLgMaster->gCoarse2SearchStep)))
	  eolYNeg   =  kMinNumberY;
	else
	  eolYNeg =  currentY - (HatchCount * pLgMaster->gCoarse2SearchStep);
	i = 0;
	// do a fake search to see if there is a false trigger problem 
	tempX = currentX + ((((i&1)<<1)-1)*(i>>1)*pLgMaster->gCoarse2SearchStep);
	tempY = currentY + ((((i&1)<<1)-1)*(i>>1)*pLgMaster->gCoarse2SearchStep);

	posStepSize = pLgMaster->gCoarse2SearchStep;
	negStepSize = -pLgMaster->gCoarse2SearchStep;
	xSteps = (eolXPos - eolXNeg) / posStepSize;
	ySteps = (eolYPos - eolYNeg) / posStepSize;
	theResult = FakeLeg(pLgMaster, eolXNeg, tempY, posStepSize, 0, xSteps,
			    foundX, foundY );
	if (theResult == kStopWasDone)
	  {
	    SearchBeamOff(pLgMaster);
	    return(theResult);
	  }
	theResult = FakeLeg(pLgMaster, tempX, eolYNeg, 0, posStepSize, ySteps,
			    foundX, foundY );
	if (theResult == kStopWasDone)
	  {
	    SearchBeamOff(pLgMaster);
	    return theResult;
	  }
	// end of fake search
	while ((++i < (2*(HatchCount-2))) && theResult)
	  {
	    tempX = currentX + ((((i&1)<<1)-1)*(i>>1)*pLgMaster->gCoarse2SearchStep);
	    tempY = currentY + ((((i&1)<<1)-1)*(i>>1)*pLgMaster->gCoarse2SearchStep);
	    posStepSize = pLgMaster->gCoarse2SearchStep;
	    negStepSize = -pLgMaster->gCoarse2SearchStep;
	    xSteps = (eolXPos - eolXNeg) / posStepSize;
	    ySteps = (eolYPos - eolYNeg) / posStepSize;
	    theResult = CoarseLeg(pLgMaster, eolXNeg, tempY, posStepSize, 0, xSteps,
				  foundX, foundY);
	    if (theResult == kStopWasDone)
	      {
		SearchBeamOff(pLgMaster);
		return theResult;
	      }
	    if (theResult == 0)
	      break;
	    theResult = CoarseLeg(pLgMaster, tempX, eolYNeg, 0, posStepSize, ySteps,
				  foundX, foundY);
	    if (theResult == kStopWasDone)
	      {
		SearchBeamOff(pLgMaster);
		return theResult;
	      }
	    if (theResult == 0)
	      break;
          }
	i = gNumberOfSpirals - (HatchCount/2);
      }
    else
      i = gNumberOfSpirals;

    while (i-- && theResult)
      {
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  {
	    SearchBeamOff(pLgMaster);
	    return(kStopWasDone);
	  }

	if (eolXPos >= (int32_t)(kMaxNumberX - pLgMaster->gCoarse2SearchStep))
	  eolXPos  = kMaxNumberX;
	else
	  eolXPos += pLgMaster->gCoarse2SearchStep;
	if (eolXNeg <= (int32_t)(kMinNumberX + pLgMaster->gCoarse2SearchStep))
	  eolXNeg  = kMinNumberX;
	else
	  eolXNeg -= pLgMaster->gCoarse2SearchStep;
	if (eolYPos >= (int32_t)(kMaxNumberY - pLgMaster->gCoarse2SearchStep))
	  eolYPos  = kMaxNumberY;
	else
	  eolYPos += pLgMaster->gCoarse2SearchStep;
	if (eolYNeg <= (int32_t)(kMinNumberY + pLgMaster->gCoarse2SearchStep))
	  eolYNeg  = kMinNumberY;
	else
	  eolYNeg -= pLgMaster->gCoarse2SearchStep;

	posStepSize = pLgMaster->gCoarse2SearchStep;
	negStepSize = -pLgMaster->gCoarse2SearchStep;
	xSteps = (eolXPos - eolXNeg) / posStepSize;
	ySteps = (eolYPos - eolYNeg) / posStepSize;

	theResult = CoarseLeg(pLgMaster, eolXNeg, eolYNeg, posStepSize, 0, xSteps,
			      foundX, foundY);
	if (theResult == kStopWasDone)
	  {
	    SearchBeamOff(pLgMaster);
	    return(theResult);
	  }
	if (theResult == 0)
	  break;
	theResult = CoarseLeg(pLgMaster, eolXPos, eolYNeg, 0, posStepSize, ySteps,
			      foundX, foundY);
	if (theResult == kStopWasDone)
	  {
	    SearchBeamOff(pLgMaster);
	    return theResult;
	  }
	if (theResult == 0)
	  break;
	theResult = CoarseLeg(pLgMaster, eolXPos, eolYPos, negStepSize, 0, xSteps,
			      foundX, foundY);
	if (theResult == kStopWasDone)
	  {
	    SearchBeamOff(pLgMaster);
	    return(theResult);
	  }
	if (theResult == 0)
	  break;
	theResult = CoarseLeg(pLgMaster, eolXNeg, eolYPos, 0, negStepSize, ySteps,
			      foundX, foundY);
	if (theResult == kStopWasDone)
	  {
	    SearchBeamOff(pLgMaster);
	    return theResult;
	  }
	if (theResult == 0)
	  break;
      }
    doRestartTimer(pLgMaster);
    if (gDwell > 0)
      {
	centX = *foundX | 0x40;
	centY = *foundY;
	for (i=0; i<gDwell; i++)
	  {
	    delY = 8*kSuperFineSearchStep;
	    delX = 8*kSuperFineSearchStep;
	    nSteps =  32; 
	    centX = *foundX;
	    centY = *foundY - 64*delY;
	    xydata.xdata = (centX & kMaxSigned);
	    xydata.ydata = (centY & kMaxSigned);
	    xydelta.xdata = 0;
	    xydelta.ydata = (delY & kMaxSigned);
	    if (DoLineSearch(pLgMaster, (struct lg_xydata *)&xydata,
			     (struct lg_xydata *)&xydelta, nSteps, gOut))
	      {
		SearchBeamOff(pLgMaster);
		return kStopWasDone;
	      }
	    centX = *foundX - 64*delX;
	    centY = *foundY;
	    xydata.xdata = (centX & kMaxSigned);
	    xydata.ydata = (centY & kMaxSigned);
	    xydelta.xdata = (delX & kMaxSigned);
	    xydelta.ydata = 0;
	    if (DoLineSearch(pLgMaster, (struct lg_xydata *)&xydata,
			     (struct lg_xydata *)&xydelta, nSteps, gOut))
	      {
		SearchBeamOff(pLgMaster);
		return kStopWasDone;
	      }
	  }
      }
    SearchBeamOff(pLgMaster);
    centX = *foundX & 0x00;
    centY = *foundY;
    xydata.xdata = (centX & kMaxSigned);
    xydata.ydata = (centY & kMaxSigned);
    move_dark(pLgMaster, (struct lg_xydata *)&xydata);
    return(theResult);
}

static int CoarseLeg(struct lg_master *pLgMaster, int32_t Xin, int32_t Yin,
		     int32_t delX, int32_t delY, uint32_t nStepsIn,
		     int32_t *foundX, int32_t *foundY)
{
    struct lg_xydata xydata;
    struct lg_xydata xydelta;
    int32_t tmpX, tmpY;
    int32_t eolX, eolY;
    int32_t negX, negY;
    int32_t avgX, avgY;
    int32_t *ptr;
    double dMinX;
    double dMinY;
    double dMaxX;
    double dMaxY;
    double sumX, sumY, count;
    uint32_t i;
    int theResult;

    theResult = kCoarseNotFound;
    ptr = gLout;
    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    memset((char *)&xydelta, 0, sizeof(struct lg_xydata));
    memset((char *)&gLout[0], 0, (nStepsIn * sizeof(int32_t)));

    xydata.xdata = Xin & kMaxSigned;
    xydata.ydata = Yin & kMaxSigned;
    xydelta.xdata = delX & kMaxSigned;
    xydelta.ydata = delY & kMaxSigned;
    if (DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
			  (struct lg_xydata*)&xydelta, nStepsIn, gLout))
      return(kStopWasDone);
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return(kStopWasDone);
    ptr  = &(gLout[0]);
    memset((char *)gLsort, 0, nStepsIn*sizeof(int32_t));
    for (i = 0; i < nStepsIn; i++)
      gLsort[i] = gLout[i];
    qsort( gLsort, nStepsIn, sizeof(uint32_t), int32_tsort );
    minlevel = gDELLEV;
    count = 0.0;
    sumX  = 0.0;
    sumY  = 0.0;
    for (i = 0; i < 10; i++)
      {
	tmpX = Xin + i * delX;
	tmpY = Yin + i * delY;
	if ( gLout[i] >= minlevel ) {
	  sumX += (double)((int32_t)tmpX);
	  sumY += (double)((int32_t)tmpY);
	  count += 1.0;
	}
      }
    if (count >= 1.0)
      {
	avgX = (int32_t)(sumX/count);
	avgY = (int32_t)(sumY/count);
	tmpX = avgX & kMaxSigned;
	tmpY = avgY & kMaxSigned;
	//
	// center back-search on found point
	//
	eolX = tmpX + (nStepsIn / 2) * delX;
	eolY = tmpY + (nStepsIn / 2) * delY;
	negX = -delX;
	negY = -delY;
	xydata.xdata = eolX & kMaxSigned;
	xydata.ydata = eolY & kMaxSigned;
	xydelta.xdata = negX & kMaxSigned;
	xydelta.ydata = negY & kMaxSigned;
	if (DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
			  (struct lg_xydata*)&xydelta, nStepsIn, gLout))
	  return(kStopWasDone);
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
                 
	// zero out first ten points
	memset((char *)&gLout[0], 0, (sizeof(int32_t) * 10)); 
	ptr  = &(gLout[0]);
	tmpX = eolX;
	tmpY = eolY;
	i = 0;
	sumX = 0.0;
	sumY = 0.0;
	count = 0.0;
	while (i < nStepsIn)
	  {
	    tmpX += negX;
	    tmpY += negY;
	    if (*ptr > minlevel)
	      {
		sumX += (double)((int32_t)tmpX);
		sumY += (double)((int32_t)tmpY);
		count += 1.0;
	      }
	    ptr++;
	    i++;
	  }
	if (count >= 1)
	  {
	    avgX = (int32_t)(sumX/count);
	    avgY = (int32_t)(sumY/count);
	    theResult = DoFineLevel(pLgMaster, &avgX, &avgY, &dMinX, &dMinY,
				    &dMaxX, &dMaxY);
	    if (theResult)
	      return(theResult);
	    theResult = DoRedundantSuperFineSearch(pLgMaster, &avgX, &avgY,
						   &dMinX, &dMinY, &dMaxX,
						   &dMaxY);
	    if (theResult == 0)
	      {
		*foundX = avgX & kMask;
		*foundY = avgY & kMask;
		return(0);
	      }
	  }
      }

    i = 10;
    tmpX = Xin + i * delX;
    tmpY = Yin + i * delY;
    ptr  = &(gLout[10]);
    while ((i < nStepsIn) && (*ptr < minlevel))
      tmpX += delX; tmpY += delY; ptr++; i++;

    if ((*ptr >= minlevel) && (i < nStepsIn))
      {
	//
	// center back-search on found point
	//
	eolX = tmpX + (nStepsIn / 2) * delX;
	eolY = tmpY + (nStepsIn / 2) * delY;
	negX = -delX;
	negY = -delY;
	xydata.xdata = eolX & kMaxSigned;
	xydata.ydata = eolY & kMaxSigned;
	xydelta.xdata = negX & kMaxSigned;
	xydelta.ydata = negY & kMaxSigned;
	if (DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
			  (struct lg_xydata*)&xydelta, nStepsIn, gLout))
	  return(kStopWasDone);
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
	// zero out first ten points
	memset((char *)&gLout[0], 0, (sizeof(int32_t) * 10)); 
	ptr  = &(gLout[0]);
	tmpX = eolX;
	tmpY = eolY;
	i = 0;
	sumX = 0.0;
	sumY = 0.0;
	count = 0.0;
	while (i < nStepsIn)
	  {
	    tmpX += negX;
	    tmpY += negY;
	    if (*ptr > minlevel)
	      {
		sumX += (double)((int32_t)tmpX);
		sumY += (double)((int32_t)tmpY);
		count += 1.0;
	      }
	    ptr++;
	    i++;
	  }
	if (count >= 1)
	  {
	    avgX = sumX/count;
	    avgY = sumY/count;
	    theResult = DoFineLevel(pLgMaster, &avgX, &avgY, &dMinX, &dMinY, &dMaxX, &dMaxY);
	    if (theResult)
	      return(theResult);
	    theResult = DoRedundantSuperFineSearch (pLgMaster, &avgX, &avgY, &dMinX, &dMinY, &dMaxX, &dMaxY);
	    if (theResult == 0)
	      {
		*foundX = avgX & kMask;
		*foundY = avgY & kMask;
		return 0;
	      }
	  }
      }
    return(theResult);
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



static int DoQuickFineSearch(struct lg_master *pLgMaster, int32_t *foundX, int32_t *foundY)
{
    struct lg_xydata xydata;
    struct lg_xydata xydelta;
    int32_t currentX, currentY;
    int32_t centerX, centerY, theSpan;
    int32_t delX;
    int32_t tmpX;
    int32_t delY;
    int32_t tmpY;
    uint32_t nSteps;
    uint32_t finetest;
    short numberOfXScansToAverage;
    short numberOfYScansToAverage;
    uint32_t i, j, index;
    int32_t *ptr;
        
    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    memset((char *)&xydelta, 0, sizeof(struct lg_xydata));
    theSpan = kSuperFineSearchStep * kSuperFineSearchSpanSteps;

    centerX = *foundX;
    centerY = *foundY;
        
    *foundX = 0x0;
    *foundY = 0x0;

    j = 1;
    numberOfXScansToAverage = 0;
    numberOfYScansToAverage = 0;
    while (j <= 3)
      {
	finetest = 0;
	j++;
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;

	/* Go right */
	currentX = centerX - theSpan;
	currentY = centerY;
	delX = 2*gSuperFineSearchStep;
	nSteps = theSpan / gSuperFineSearchStep;
	xydata.xdata = currentX & kMaxSigned;
	xydata.ydata = currentY & kMaxSigned;
	xydelta.xdata = delX & kMaxSigned;
	xydelta.ydata = 0;
	DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
		      (struct lg_xydata*)&xydelta, nSteps, gLout);
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);

	if (j == 1)
	  {
	    ptr  = &(gLout[0]);
	    memset((char *)gLsort, 0, nSteps*sizeof(int32_t));
	    for (i = 0; i < nSteps; i++)
	      gLsort[i] = gLout[i];
	    qsort( gLsort, nSteps, sizeof(int32_t), int32_tsort );
	    minlevel  = gDELLEV;
	  }

	memset((char *)gLout, 0, (7 * sizeof(int32_t)));
	index = 0; ptr = &(gLout[0]);   tmpX = currentX;
	if (j > 1)
	  {
	    while (index<nSteps)
	      {
		if (gLout[index] >= minlevel)
		  {
		    xNegSuper[index]++;
		    numberOfXScansToAverage++;
		    finetest = finetest | 0x01;
		  }
		xNegPosition[index] = tmpX;
		index++; ptr++;  tmpX += delX;
	      }
	  }
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);

	/* Go left */
	currentX = centerX + theSpan;
	currentY = centerY;
	delX = -2 * gSuperFineSearchStep;
	nSteps = theSpan / gSuperFineSearchStep;
	xydata.xdata = currentX & kMaxSigned;
	xydata.ydata = currentY & kMaxSigned;
	xydelta.xdata = delX & kMaxSigned;
	xydelta.ydata = 0;
	DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
		      (struct lg_xydata*)&xydelta, nSteps, gLout);
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);

	memset((char *)gLout, 0, (7 * sizeof(uint32_t)));
	index = 0; ptr = &(gLout[0]);  tmpX = currentX;
	if (j > 1)
	  {
	    while (index < nSteps)
	      {
		if (gLout[index] >= minlevel)
		  {
		    xPosSuper[index]++;
		    numberOfXScansToAverage++;
		    finetest = finetest | 0x02;
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
	xydata.xdata = currentX & kMaxSigned;
	xydata.ydata = currentY & kMaxSigned;
	xydelta.ydata = delY & kMaxSigned;
	xydelta.xdata = 0;
	DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
		      (struct lg_xydata*)&xydelta, nSteps, gLout);
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
	if (j == 1)
	  {
	    ptr  = &(gLout[0]);
	    memset((char *)gLsort, 0, (nSteps*sizeof(int32_t)));
	    for (i = 0 ; i < nSteps; i++)
	      gLsort[i] = gLout[i];
	    qsort( gLsort, nSteps, sizeof(int32_t), int32_tsort );
	    minlevel  = gDELLEV;
	  }

	    memset((char *)gLout, 0, (7 * sizeof(int32_t)));
	    index = 0; ptr = &(gLout[0]);   tmpY = currentY;
	    if (j > 1)
	      {
		while (index<nSteps)
		  {
		    if (gLout[index] >= minlevel)
		      {
			yNegSuper[index]++;
			numberOfYScansToAverage++;
			finetest = finetest | 0x04;
		      }
		    yNegPosition[index] = tmpY;
		    index++; ptr++;  tmpY += delY;
		  }
	      }
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);

	    /* Go down */
	    currentX = centerX;
	    currentY = centerY + theSpan;
	    delY = -2 * gSuperFineSearchStep;
	    nSteps = theSpan / gSuperFineSearchStep;
	    xydata.xdata = currentX & kMaxSigned;
	    xydata.ydata = currentY & kMaxSigned;
	    xydelta.ydata = delY & kMaxSigned;
	    xydelta.xdata = 0;
	    DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
			  (struct lg_xydata*)&xydelta, nSteps, gLout);
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);

	    memset((char *)gLout, 0, (7 * sizeof(int32_t)));
	    index = 0; ptr = &(gLout[0]);  tmpY = currentY;
	    if (j > 1)
	      {
		while (index < nSteps)
		  {
		    if (gLout[index] >= minlevel)
		      {
			yPosSuper[index]++;
			numberOfYScansToAverage++;
			finetest = finetest | 0x08;
		      }
		    yPosPosition[index] = tmpY;
		    index++; ptr++;   tmpY += delY;
		  }
	      }
	    if (finetest == 0x0f)
	      break;
      }
    if (finetest == 0x0f)
      return(0);
    if ((numberOfXScansToAverage <= 2) && (numberOfYScansToAverage <= 2))
      return(kSuperFineNotFound);
    return(kSuperFineNotFound);
}

// FIME--PAH--NEED TO CHECK EVERY SINGLE POINTER FROM CALLOC/MALLOC
//  AND THEN MEMSET TO 0 TO INITIALIZE.  NOT GUARANTEED MEMSET IS DONE.
void InitSensorSearch(void)
{
  int i;

  gOut = (unsigned char *)calloc((size_t)kMaxOutLength,(size_t)1);
  gLout = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  gLout1 = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  gLout2 = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  gLsort = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));

  xPosSave = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  xNegSave = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  yPosSave = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  yNegSave = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));

  xPosEdge = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  xNegEdge = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  yPosEdge = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  yNegEdge = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));

  xPosSuper = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  xNegSuper = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  yPosSuper = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  yNegSuper = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));

  xPosESuper = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  xNegESuper = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  yPosESuper = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  yNegESuper = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));

  xPosPosition = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  yPosPosition = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));

  xNegPosition = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));
  yNegPosition = (int32_t *)calloc((size_t)kMaxOutLength,sizeof(int32_t));

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
  return;
}
// FIME--PAH--NEED TO CHECK EVERY SINGLE POINTER BEFORE FREEING
void CloseSensorSearch(void)
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
  return;
}

void CorrectDrift(int32_t cX, int32_t cY, int32_t *fX, int32_t *fY)
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

     *fX = tmpX & kMaxSigned;
     *fY = tmpY & kMaxSigned;
     return;
}

void FindDrift(int32_t cX, int32_t cY, int32_t fX, int32_t fY)
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
     syslog(LOG_NOTICE, "aXbXaYbY %12.2f %12.6f %12.2f %12.6f", aX, g_bX, aY, g_bY);
     return;
}

void InitDrift(int32_t *Xarr, int32_t *Yarr)
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

int  FakeLeg(struct lg_master *pLgMaster, int32_t X, int32_t Y,
	     int32_t delX, int32_t delY, uint32_t nSteps,
	     int32_t *foundX, int32_t *foundY)
{
	struct lg_xydata xydata;
	struct lg_xydata xydelta;
        int32_t tmpX, tmpY;
        int32_t eolX, eolY;
        int32_t negX, negY;
        int32_t *ptr;
        int32_t i;
        int theResult;
	
        theResult = kCoarseNotFound;
        ptr  = gLout;
	memset((char *)&xydata, 0, sizeof(struct lg_xydata));
	memset((char *)&xydelta, 0, sizeof(struct lg_xydata));
	memset((char *)&ptr[0], 0, nSteps * (sizeof(int32_t)));
	xydata.xdata = X & kMaxSigned;
	xydata.ydata = Y & kMaxSigned;
	move_dark(pLgMaster, (struct lg_xydata *)&xydata);
	xydelta.xdata = delX & kMaxSigned;
	xydelta.ydata = delY & kMaxSigned;
	if (DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
			  (struct lg_xydata*)&xydelta, nSteps, gLout))
	  return kStopWasDone;
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
                negX = -delX;
                negY = -delY;
		xydata.xdata = eolX & kMaxSigned;
		xydata.ydata = eolY & kMaxSigned;
		move_dark(pLgMaster, (struct lg_xydata *)&xydata);

		xydelta.xdata = delX & kMaxSigned;
		xydelta.ydata = delY & kMaxSigned;
		if (DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
				  (struct lg_xydata*)&xydelta, nSteps, gLout))
		  return kStopWasDone;
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


static int DoFineLevel(struct lg_master *pLgMaster, int32_t *foundX,
		       int32_t *foundY, double *outMinX, double *outMinY,
		       double *outMaxX, double *outMaxY)
{
      struct lg_xydata  xydata;
      int32_t eolXPos, eolYPos;
      int32_t currentX, currentY;
      int32_t eolXNeg, eolYNeg;
      int32_t centerX, centerY;
      int32_t tmpX, tmpY;
      int32_t firstX, firstY, lastX, lastY;
      int32_t stepSize, negStep;
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
        
      memset((char *)&xydata, 0, sizeof(struct lg_xydata));
      nSteps = kFineSearchSpanSteps << 1;
                
      centerX = *foundX;
      centerY = *foundY;
      
      xydata.xdata = (centerX & kMaxSigned);
      xydata.ydata = (centerY & kMaxSigned);
      move_dark(pLgMaster, (struct lg_xydata *)&xydata);

      stepSize = kFineSearchStep;
      negStep  = -kFineSearchStep;
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
      dLX = (double)((int32_t)lastX);
      dLY = (double)((int32_t)lastY);
      if ( dLX < dMinX ) dMinX = dLX;
      if ( dLY < dMinY ) dMinY = dLY;
      if ( dLX > dMaxX ) dMaxX = dLX;
      if ( dLY > dMaxY ) dMaxY = dLY;
      avX = (double)Xm;
      avY = (double)Ym;
      tmpX = (int32_t)avX;
      tmpY = (int32_t)avY;
      centerX = tmpX;
      centerY = tmpY;
	
      if (IfStopThenStopAndNeg1Else0(pLgMaster))
	return kStopWasDone;

      /* Go up */
      limitCalc(centerX, centerY, &eolXNeg, &eolXPos, &eolYNeg, &eolYPos);

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
      tmpX = (int32_t)avX;
      tmpY = (int32_t)avY;
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
      tmpX = (int32_t)avX;
      tmpY = (int32_t)avY;
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
	avX = Xm;
	avY = Ym;
	tmpX = (int32_t)avX;
	tmpY = (int32_t)avY;
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
	tmpX = (int32_t)avX;
	tmpY = (int32_t)avY;
	centerX = tmpX;
	centerY = tmpY;
	*outMinX = dMinX - 8 * (double)((int32_t)stepSize);
        *outMinY = dMinY - 8 * (double)((int32_t)stepSize);
        *outMaxX = dMaxX + 8 * (double)((int32_t)stepSize);
        *outMaxY = dMaxY + 8 * (double)((int32_t)stepSize);
        *foundX = centerX & kMask;
        *foundY = centerY & kMask;
        return(0);
}

static int        
DoRedundantSuperFineSearch (struct lg_master *pLgMaster,
			    int32_t *foundX,
			    int32_t *foundY,
			    double * dMinX,
			    double * dMinY,
			    double * dMaxX,
			    double * dMaxY)
{
        int32_t tempX;
        int32_t tempY;
        int result;

        tempX = *foundX;
        tempY = *foundY;

	result = DoSuperLevelSearch(pLgMaster, &tempX,
				    &tempY, dMinX, dMinY,
				    dMaxX, dMaxY);
	if (result)
	  return result;

        *foundX = kMask & tempX;
        *foundY = kMask & tempY;
        return 0;
}

static int
DoSuperLevelSearch ( struct lg_master *pLgMaster,
		     int32_t *foundX,
		     int32_t *foundY,
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
        int32_t tmpX, tmpY;
        int count;
        int localCount;
        int result;
        int i,j;
        int ix, jy;
        int ixsum, jysum, nsum;
        char * ptr;
        
        *foundX = 0x0;
        *foundY = 0x0;
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

        *foundX = kMask & (int32_t)avgX;
        *foundY = kMask & (int32_t)avgY;
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
                   } else
		     syslog(LOG_ERR, "error SS2208 xy %d %d", ix, jy );


                   ((int32_t *)ptr)[0] = (int32_t)(gSaveDblX1[i]);
                   ((int32_t *)ptr)[1] = (int32_t)(gSaveDblY1[i]);
                   ptr      += 2 * sizeof(int32_t);
                   gLoutPtr += 2 * sizeof(int32_t);
                   *((int16_t *)ptr) = gSaveLout1[i];
                   ptr      += sizeof(short);
                   gLoutPtr += sizeof(short);
                   ((unsigned char *)ptr)[0] = (unsigned char)(10*gSearchCurrentSensor+gSaveSweep[i]);

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
                   if ((ix>=0) && (jy>=0) && (ix<1024) && (jy<1024))
		     gSuperReturn[ix][jy] = 1;
                   else
		     syslog(LOG_ERR, "error SS2208 xy %d %d", ix, jy );
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
        for (i = 0; i < 1024; i++)
	  {
            for (j = 0; j < 1024; j++)
	      {
		if (gSuperReturn[i][j] == 1)
		  {
		    ixsum += i;
		    jysum += j;
		    nsum++;
		  }
	      }
	  }
        if (nsum > 0)
	  {
            Dix = (double)ixsum / (double)nsum;
            Djy = (double)jysum / (double)nsum;
            avgX =  Dix * gSuperFineSearchStep + *dMinX;
            avgY =  Djy * gSuperFineSearchStep + *dMinY;
            *foundX = kMask & (int32_t)avgX;
            *foundY = kMask & (int32_t)avgY;
	  }
        if ((gMultipleSweeps == 2) && (sweepNumX > 1) && (sweepNumY > 1))
	  {
            sweepAvgX = sweepSumX / sweepNumX;
            sweepAvgY = sweepSumY / sweepNumY;
            *foundX = kMask & (int32_t)sweepAvgX;
            *foundY = kMask & (int32_t)sweepAvgY;
	  }

        ptr = gLoutBase + 1 * sizeof(int32_t);
        *ptr = gLoutSize;
        ptr = gLoutBase + 2 * sizeof(int32_t);
        *ptr = gLoutCount;

	// zero based current target (sensor)
        ptr = gLoutBase + (3+gSearchCurrentSensor) * sizeof(int32_t);
        *ptr = localCount;
        return(0);
}


static int findFirstLast(struct lg_master *pLgMaster, int32_t *firstX, int32_t *firstY,
			 int32_t *lastX, int32_t *lastY, int32_t *medianX,
			 int32_t *medianY, int32_t currentX, int32_t currentY,
			 uint32_t xStep, uint32_t yStep, uint32_t nSteps)
{
	struct lg_xydata xydata;
	struct lg_xydata xydelta;
        int i;
        int32_t *ptr;
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
	memset((char *)&xydata, 0, sizeof(struct lg_xydata));
	memset((char *)&xydelta, 0, sizeof(struct lg_xydata));
	memset((char *)&dXMedian, 0, sizeof(dXMedian));
	memset((char *)&dYMedian, 0, sizeof(dYMedian));

	xydata.xdata = currentX & kMaxSigned;
	xydata.ydata = currentY & kMaxSigned;
	xydelta.xdata = xStep & kMaxSigned;
	xydelta.ydata = yStep & kMaxSigned;
	if (DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
			  (struct lg_xydata *)&xydelta, nSteps, gLout))
	  return kStopWasDone;
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;

	// zero out the first twelve entries
	memset((char *)&gLout[0], 0, (12 * sizeof(int32_t)));
        ptr = &(gLout[0]);
        iCountMedian = 0;
        startX = (double)currentX;
        startY = (double)currentY;
        dX = (double)((int32_t)xStep);
        dY = (double)((int32_t)yStep);
        while (i < nSteps)
	  {
	    dI = (double)i;
	    tmpX = startX + dI * dX;
	    tmpY = startY + dI * dY;
	    // throw away first few 
	    if ((*ptr >= minlevel) && (i > NZERO))
	      {
		if (firstFlag == 1)
		  {
		    firstFlag = 0;
		    *firstX = (int32_t)tmpX;
		    *firstY = (int32_t)tmpY;
                  }
		*lastX = (int32_t)tmpX;
		*lastY = (int32_t)tmpY;
		dXMedian[iCountMedian] = (int32_t)tmpX;
		dYMedian[iCountMedian] = (int32_t)tmpY;
		iCountMedian++;
	      }
	    i++; ptr++;
	  }
        if (iCountMedian == 0)
	  return(kFineNotFound);
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;

        qsort( dXMedian, iCountMedian, sizeof(int32_t), int32_tsort );
        qsort( dYMedian, iCountMedian, sizeof(int32_t), int32_tsort );
        ihalf = iCountMedian / 2;
        Xm  = dXMedian[ihalf];
        Ym  = dYMedian[ihalf];
        *medianX = Xm;
        *medianY = Ym;
        return 0;
}

void limitCalc(int32_t centerX, int32_t centerY, int32_t *eolXNeg,
	       int32_t *eolXPos, int32_t *eolYNeg, int32_t *eolYPos)
{
    int32_t theSpan, twiceSpan;

    theSpan = kFineSearchStep * kFineSearchSpanSteps;
    twiceSpan = theSpan << 1;

    if (centerX <= (kMinNumberX + theSpan))
      *eolXNeg = kMinNumberX + theSpan;
    else
      {
	*eolXNeg = centerX - theSpan;
	if (*eolXNeg > (kMaxNumberX - twiceSpan))
	  *eolXNeg = kMaxNumberX - twiceSpan;
      }
    *eolXPos = *eolXNeg + twiceSpan;

    if (centerY <= (kMinNumberY + theSpan))
      *eolYNeg = kMinNumberY + theSpan;
    else
      {
	*eolYNeg = centerY - theSpan;
	if (*eolYNeg > (kMaxNumberY - twiceSpan))
	  *eolYNeg = kMaxNumberY - twiceSpan;
      }
    *eolYPos = *eolYNeg + twiceSpan;
    return;
}

void
SetSuperFineFactor ( uint32_t n )
{
    if (n > 0x100)
      n = 0x100;
    if (n < 0x1)
      n = 0x1;
    gSuperFineFactor  = n;
    gSuperFineSearchStep =  gSuperFineFactor * kSuperFineSearchStep;
    return;
}

int SuperSearch(struct lg_master *pLgMaster, int32_t *foundX, int32_t *foundY,
		double *dMinX, double *dMinY, double *dMaxX, double *dMaxY)
{
    struct lg_xydata xydata;
    struct lg_xydata xydelta;
    int i;
    int result;
    double sumX;
    double sumY;
    double avgX;
    double avgY;
    int32_t startX;
    int32_t startY;
    int32_t endX;
    int32_t endY;
    int32_t startX1;
    int32_t startX2;
    int32_t startY1;
    int32_t startY2;
    int32_t delNeg;
    int32_t delPos;
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

    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    memset((char *)&xydelta, 0, sizeof(struct lg_xydata));
  
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

    delNeg = -gSuperFineSearchStep;
    delPos = gSuperFineSearchStep;
  
    sweep = 1;
    sumX = 0.0;
    sumY = 0.0;
    dCount = 0;
    Dstep = (double)delPos;
    for (dblX = *dMinX; dblX < *dMaxX; dblX +=  Dstep )
      {
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
	startX  = kMask & (int32_t)dblX;
	startY1 = kMask & (int32_t)*dMinY;
	startY2 = kMask & (int32_t)*dMaxY;
	for (i=1; i<nYsteps; i++)
	  {
	    gLout1[i] = 0;
	    gLout2[i] = 0;
	  }
	// do first set of scans twice
	if (firstSuper == 1)
	  {
	    firstSuper = 0;
	    xydata.xdata = startX & kMaxSigned;
	    xydata.ydata = startY2 & kMaxSigned;
	    xydelta.xdata = 0;
	    xydelta.ydata = delNeg & kMaxSigned;
	    result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
				   (struct lg_xydata *)&xydelta, nYsteps, gLout2);
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);
	    if (result == kStopWasDone)
	      return(result);
	    startX  = kMask & (int32_t)dblX;
	    startY1 = kMask & (int32_t)*dMinY;
	    xydata.xdata = startX & kMaxSigned;
	    xydata.ydata = startY1 & kMaxSigned;
	    xydelta.xdata = 0;
	    xydelta.ydata = delPos & kMaxSigned;
	    result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
				   (struct lg_xydata *)&xydelta, nYsteps, gLout1);
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);
	    if (result == kStopWasDone)
	      return(result);
	  }
	startX  = kMask & (int32_t)dblX;
	startY2 = kMask & ((uint32_t)((int32_t)(*dMaxY)));
	xydata.xdata = startX & kMaxSigned;
	xydata.ydata = startY2 & kMaxSigned;
	xydelta.xdata = 0;
	xydelta.ydata = delNeg & kMaxSigned;
	result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
			       (struct lg_xydata *)&xydelta, nYsteps, gLout2);
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
	if (result == kStopWasDone)
	  return(result);
	if (gLout2[1] > minlevel)
	  memset((char *)gLout2, 0, (NZERO * nYsteps * sizeof(int32_t)));
	startX  = kMask & (int32_t)dblX;
	startY1 = kMask & (int32_t)*dMinY;
	xydata.xdata = startX & kMaxSigned;
	xydata.ydata = startY1 & kMaxSigned;
	xydelta.xdata = 0;
	xydelta.ydata = delPos & kMaxSigned;
	result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
			       (struct lg_xydata *)&xydelta, nYsteps, gLout1);
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
	if (gLout1[1] > minlevel)
	  {
	    for ( i=0; i<NZERO && i<nYsteps; i++ )
	      gLout1[i] = 0;
	  }
	if (result == kStopWasDone)
	  return(result);
	for (i=1; i<nYsteps; i++)
	  {
	    i1 = i;
	    i2 = nYsteps  - i;
	    dblY1 = (double)startY1 + (double)(i * delPos);
	    dblY2 = (double)startY2 + (double)(i * delNeg);
	    gSaveMatch[gLoutIndex] = 0;
	    gSaveSweep[gLoutIndex] = 0;
	    if (gLout1[i1] > minlevel)
	      {
		gSaveMatch[gLoutIndex] = 11;
		gSaveSweep[gLoutIndex] = sweep;
		gXsuperSave[gSuperIndex] = (int32_t)dblX;
		gYsuperSave[gSuperIndex] = (int32_t)dblY1;
		gSuperIndex++;
	      }
	    if (gLout2[i2] > minlevel)
	      {
		gSaveMatch[gLoutIndex] = 12;
		gSaveSweep[gLoutIndex] = sweep;
		gXsuperSave[gSuperIndex] = (int32_t)dblX;
		gYsuperSave[gSuperIndex] = (int32_t)dblY2;
		gSuperIndex++;
	      }
	    if ((gLout1[i1] > minlevel) && (gLout2[i2] > minlevel))
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
	  }
      }
    if (gMultipleSweeps == 0)
      {
	if (dCount < 3)
	  return(kSuperFineNotFound);
      }
    if (gMultipleSweeps >= 1)
      {
	sweep = 2;
	Dstep = (double)delPos;
	for (dblY = *dMinY; dblY < *dMaxY; dblY +=  Dstep)
	  {
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return kStopWasDone;
	    startX2  = kMask & (int32_t)*dMaxX;
	    startY   = kMask & (int32_t)dblY;
	    for (i=1; i<nXsteps; i++)
	      {
		gLout1[i] = 0;
		gLout2[i] = 0;
	      }
	    xydata.xdata = startX2 & kMaxSigned;
	    xydata.ydata = startY & kMaxSigned;
	    xydelta.ydata = 0;
	    xydelta.xdata = delNeg & kMaxSigned;
	    result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
				   (struct lg_xydata *)&xydelta, nXsteps, gLout2);
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);
	    if (result == kStopWasDone)
	      return(result);
	    if (gLout2[1] > minlevel)
	      memset((char *)gLout2, 0, (NZERO * nYsteps * sizeof(int32_t)));
	    startX1  = kMask & (int32_t)*dMinX;
	    startY   = kMask & (int32_t)dblY;
	    xydata.xdata = startX1 & kMaxSigned;
	    xydata.ydata = startY & kMaxSigned;
	    xydelta.ydata = 0;
	    xydelta.xdata = delPos & kMaxSigned;
	    result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
				   (struct lg_xydata *)&xydelta, nXsteps, gLout1);
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);
	    if (result == kStopWasDone)
	      return(result);
	    if (gLout1[1] > minlevel)
	      memset((char *)gLout1, 0, (NZERO * nYsteps * sizeof(int32_t)));
	    for (i=1; i<nXsteps; i++)
	      {
		i1 = i;
		i2 = nXsteps  - i;
		dblX1 = (double)startX1 + (double)(i * delPos);
		dblX2 = (double)startX2 + (double)(i * delNeg);
		gSaveMatch[gLoutIndex] = 0;
		gSaveSweep[gLoutIndex] = 0;
		if (gLout1[i1] > minlevel)
		  {
		    gSaveMatch[gLoutIndex] = 11;
		    gSaveSweep[gLoutIndex] = sweep;
		    gXsuperSave[gSuperIndex] = (int32_t)dblX1;
		    gYsuperSave[gSuperIndex] = (int32_t)dblY;
		    gSuperIndex++;
		  }
		if (gLout2[i2] > minlevel)
		  {
		    gSaveMatch[gLoutIndex] = 12;
		    gSaveSweep[gLoutIndex] = sweep;
		    gXsuperSave[gSuperIndex] = (int32_t)dblX2;
		    gYsuperSave[gSuperIndex] = (int32_t)dblY;
		    gSuperIndex++;
		  }
		if ((gLout1[i1] > minlevel) && (gLout2[i2] > minlevel))
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
	      }
	  }
      }
  
    if (gMultipleSweeps >= 3)
      {
	sweep = 3;
	Dstep = (double)delPos;
	for (dblX = *dMaxX; dblX > *dMinX; dblX -=  Dstep)
	  {
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);
	    startX  = kMask & (int32_t)dblX;
	    startY1 = kMask & (int32_t)*dMinY;
	    startY2 = kMask & (int32_t)*dMaxY;
	    for (i=1; i<nYsteps; i++)
	      {
		gLout1[i] = 0;
		gLout2[i] = 0;
	      }
	    xydata.xdata = startX & kMaxSigned;
	    xydata.ydata = startY2 & kMaxSigned;
	    xydelta.xdata = 0;
	    xydelta.ydata = delNeg & kMaxSigned;
	    result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
				   (struct lg_xydata *)&xydelta, nYsteps, gLout2);
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);
	    if (result == kStopWasDone)
	      return(result);
	    if (gLout2[1] > minlevel)
	      memset((char *)gLout2, 0, (NZERO * nYsteps * sizeof(int32_t)));
	    startX  = kMask & (int32_t)dblX;
	    startY1 = kMask & (int32_t)*dMinY;
	    xydata.xdata = startX & kMaxSigned;
	    xydata.ydata = startY1 & kMaxSigned;
	    xydelta.xdata = 0;
	    xydelta.ydata = delPos & kMaxSigned;
	    result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
				   (struct lg_xydata *)&xydelta, nYsteps, gLout1);
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);
	    if (result == kStopWasDone)
	      return(result);
	    if (gLout1[1] > minlevel)
	      memset((char *)gLout1, 0, (NZERO * nYsteps * sizeof(int32_t)));
	    for (i=1; i<nYsteps; i++)
	      {
		i1 = i;
		i2 = nYsteps  - i;
		dblY1 = (double)startY1 + (double)(i * delPos);
		dblY2 = (double)startY2 + (double)(i * delNeg);
		gSaveMatch[gLoutIndex] = 0;
		gSaveSweep[gLoutIndex] = 0;
		if (gLout1[i1] > minlevel)
		  {
		    gSaveMatch[gLoutIndex] = 11;
		    gSaveSweep[gLoutIndex] = sweep;
		    gXsuperSave[gSuperIndex] = (int32_t)dblX;
		    gYsuperSave[gSuperIndex] = (int32_t)dblY1;
		    gSuperIndex++;
		  }
		if (gLout2[i2] > minlevel)
		  {
		    gSaveMatch[gLoutIndex] = 12;
		    gSaveSweep[gLoutIndex] = sweep;
		    gXsuperSave[gSuperIndex] = (int32_t)dblX;
		    gYsuperSave[gSuperIndex] = (int32_t)dblY2;
		    gSuperIndex++;
		  }
		if ((gLout1[i1] > minlevel) && (gLout2[i2] > minlevel))
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
	      }
	  }
      }
    if (gMultipleSweeps >= 4)
      {
	sweep = 4;
	Dstep = (double)delPos;
	for (dblY = *dMaxY; dblY > *dMinY; dblY -=  Dstep)
	  {
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);
	    startX2  = kMask & (int32_t)*dMaxX;
	    startY   = kMask & (int32_t)dblY;
	    for (i=1; i<nYsteps; i++)
	      {
		gLout1[i] = 0;
		gLout2[i] = 0;
	      }
	    xydata.xdata = startX2 & kMaxSigned;
	    xydata.ydata = startY & kMaxSigned;
	    xydelta.ydata = 0;
	    xydelta.xdata = delNeg & kMaxSigned;
	    result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
				   (struct lg_xydata *)&xydelta, nXsteps, gLout2);
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);
	    if (result == kStopWasDone)
	      return(result);
	    if (gLout2[1] > minlevel)
	      memset((char *)gLout2, 0, (NZERO * nYsteps * sizeof(int32_t)));
	    startX1  = kMask & (int32_t)*dMinX;
	    startY   = kMask & (int32_t)dblY;
	    xydata.xdata = startX1 & kMaxSigned;
	    xydata.ydata = startY & kMaxSigned;
	    xydelta.ydata = 0;
	    xydelta.xdata = delPos & kMaxSigned;
	    result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
				   (struct lg_xydata *)&xydelta, nXsteps, gLout1);
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);
	    if (result == kStopWasDone)
	      return(result);
	    if (gLout1[1] > minlevel)
	      memset((char *)gLout1, 0, (NZERO * nYsteps * sizeof(int32_t)));
	    for (i=1; i<nXsteps; i++)
	      {
		i1 = i;
		i2 = nXsteps  - i;
		dblX1 = (double)startX1 + (double)(i * delPos);
		dblX2 = (double)startX2 + (double)(i * delNeg);
		gSaveMatch[gLoutIndex] = 0;
		gSaveSweep[gLoutIndex] = 0;
		if (gLout1[i1] > minlevel)
		  {
		    gSaveMatch[gLoutIndex] = 11;
		    gSaveSweep[gLoutIndex] = sweep;
		    gXsuperSave[gSuperIndex] = (int32_t)dblX1;
		    gYsuperSave[gSuperIndex] = (int32_t)dblY;
		    gSuperIndex++;
		  }
		if (gLout2[i2] > minlevel)
		  {
		    gSaveMatch[gLoutIndex] = 12;
		    gSaveSweep[gLoutIndex] = sweep;
		    gXsuperSave[gSuperIndex] = (int32_t)dblX2;
		    gYsuperSave[gSuperIndex] = (int32_t)dblY;
		    gSuperIndex++;
		  }
		if ((gLout1[i1] > minlevel) && (gLout2[i2] > minlevel))
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
	      }
	  }
      }
    if (gMultipleSweeps >= 5)
      {
	sweep = 5;
	//  start of diagonals
	sumX = 0.0;
	sumY = 0.0;
	Dstep = (double)delPos;
	for (dblX=Xlow,dblY=Ymid; ((dblX<Xmid) && (dblY<Yhigh)); (dblX+=Dstep), (dblY+=Dstep))
	  {
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return kStopWasDone;

	    startX = kMask & (int32_t)dblX;
	    startY = kMask & (int32_t)dblY;
	    for (i=1; i<nYsteps; i++)
	      {
		gLout1[i] = 0;
		gLout2[i] = 0;
	      }
	    xydata.xdata = startX & kMaxSigned;
	    xydata.ydata = startY & kMaxSigned;
	    xydelta.xdata = delPos & kMaxSigned;
	    xydelta.ydata = delNeg & kMaxSigned;
	    result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
				   (struct lg_xydata *)&xydelta, nYsteps, gLout1);
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);
	    if (result == kStopWasDone)
	      return(result);
	if (gLout1[1] > minlevel)
	    memset((char *)gLout1, 0, (NZERO * nYsteps * sizeof(int32_t)));
	for (i=1; i<nYsteps; i++)
	  {
	    i1 = i;
	    i2 = nYsteps - i;
	    dblX1  = (double)startX + (double)(i * delPos);
	    dblY1  = (double)startY + (double)(i * delNeg);
	    gSaveMatch[gLoutIndex] = 0;
	    gSaveSweep[gLoutIndex] = 0;
	    if (gLout1[i1] > minlevel)
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
	endX = kMask & (int32_t)(dblX + (Dstep * nYsteps));
	endY = kMask & (int32_t)(dblY - (Dstep * nYsteps));
	xydata.xdata = endX & kMaxSigned;
	xydata.ydata = endY & kMaxSigned;
	xydelta.xdata = delNeg & kMaxSigned;
	xydelta.ydata = delPos & kMaxSigned;
	result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
			       (struct lg_xydata *)&xydelta, nYsteps, gLout1);
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
	if (gLout1[1] > minlevel)
	    memset((char *)gLout1, 0, (NZERO * nYsteps * sizeof(int32_t)));
	if (result == kStopWasDone)
	  return(result);
	for (i=1; i<nYsteps; i++)
	  {
	    i1 = i;
	    i2 = nYsteps  - i;
	    dblX1  = (double)endX + (double)(i * delNeg);
	    dblY1  = (double)endY + (double)(i * delPos);
	    gSaveMatch[gLoutIndex] = 0;
	    gSaveSweep[gLoutIndex] = 0;
	    if (gLout1[i1] > minlevel)
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
  if (gMultipleSweeps >= 6)
    {
      sweep = 6;
      for (dblX=Xhigh,dblY=Ymid; ((dblX>Xmid) && (dblY<Yhigh)); dblX-=Dstep,dblY-=Dstep)
	{
	  if (IfStopThenStopAndNeg1Else0(pLgMaster))
	    return(kStopWasDone);
	  startX = kMask & (int32_t)dblX;
	  startY = kMask & (int32_t)dblY;
	  for ( i=1; i<nYsteps; i++ )
	    {
	      gLout1[i] = 0;
	      gLout2[i] = 0;
	    }
	  xydata.xdata = startX & kMaxSigned;
	  xydata.ydata = startY & kMaxSigned;
	  xydelta.xdata = delNeg & kMaxSigned;
	  xydelta.ydata = delPos & kMaxSigned;
	  result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
				 (struct lg_xydata *)&xydelta, nYsteps, gLout1);
	  if (IfStopThenStopAndNeg1Else0(pLgMaster))
	    return(kStopWasDone);
	  if (gLout1[1] > minlevel)
	    memset((char *)gLout1, 0, (NZERO * nYsteps * sizeof(int32_t)));
	  if (result == kStopWasDone)
	    return(result);
	  for (i=1; i<nYsteps; i++)
	    {
	      i1 = i;
	      i2 = nYsteps  - i;
	      dblX1  = (double)startX + (double)(i * delNeg);
	      dblY1  = (double)startY + (double)(i * delPos);
	      gSaveMatch[gLoutIndex] = 0;
	      gSaveSweep[gLoutIndex] = 0;
	      if (gLout1[i1] > minlevel)
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
	  endX = kMask & (int32_t)(dblX - (Dstep * nYsteps));
	  endY = kMask & (int32_t)(dblY + (Dstep * nYsteps));
	  xydata.xdata = endX & kMaxSigned;
	  xydata.ydata = endY & kMaxSigned;
	  xydelta.xdata = delPos & kMaxSigned;
	  xydelta.ydata = delNeg & kMaxSigned;
	  result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
				 (struct lg_xydata *)&xydelta, nYsteps, gLout1);
	  if (IfStopThenStopAndNeg1Else0(pLgMaster))
	    return(kStopWasDone);
	if (gLout1[1] > minlevel)
	    memset((char *)gLout1, 0, (NZERO * nYsteps * sizeof(int32_t)));
	if (result == kStopWasDone)
	  return(result);
	for (i=1; i<nYsteps; i++)
	  {
	    i1 = i;
	    i2 = nYsteps  - i;
	    dblX1  = (double)endX + (double)(i * delPos);
	    dblY1  = (double)endY + (double)(i * delNeg);
	    gSaveMatch[gLoutIndex] = 0;
	    gSaveSweep[gLoutIndex] = 0;
	    if (gLout1[i1] > minlevel)
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

  if (gMultipleSweeps >= 7)
    {
      sweep = 7;
      for (dblX = Xmid, dblY = Yhigh; ((dblX < Xhigh) && (dblY > Ymid)); dblX += Dstep, dblY -= Dstep)
	{
	  if (IfStopThenStopAndNeg1Else0(pLgMaster))
	    return(kStopWasDone);
	  startX = kMask & (int32_t)dblX;
	  startY = kMask & (int32_t)dblY;
	  for ( i=1; i<nYsteps; i++ )
	    {
	      gLout1[i] = 0;
	      gLout2[i] = 0;
	    }
	xydata.xdata = startX & kMaxSigned;
	xydata.ydata = startY & kMaxSigned;
	xydelta.xdata = delNeg & kMaxSigned;
	xydelta.ydata = delNeg & kMaxSigned;
	result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
			       (struct lg_xydata *)&xydelta, nYsteps, gLout1);
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
	if (gLout1[1] > minlevel)
	  memset((char *)gLout1, 0, (NZERO * nYsteps * sizeof(int32_t)));
	if (result == kStopWasDone)
	  return(result);
	for (i=1; i<nYsteps; i++)
	  {
	    i1 = i;
	    i2 = nYsteps  - i;
	    dblX1  = (double)startX + (double)(i * delNeg);
	    dblY1  = (double)startY + (double)(i * delNeg);
	    gSaveMatch[gLoutIndex] = 0;
	    gSaveSweep[gLoutIndex] = 0;
	    if (gLout1[i1] > minlevel)
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
	endX = kMask & (int32_t)(dblX - (Dstep * nYsteps));
	endY = kMask & (int32_t)(dblY - (Dstep * nYsteps));
	xydata.xdata = endX & kMaxSigned;
	xydata.ydata = endY & kMaxSigned;
	xydelta.xdata = delPos & kMaxSigned;
	xydelta.ydata = delPos & kMaxSigned;
	result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
			       (struct lg_xydata *)&xydelta, nYsteps, gLout1);
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
	if (gLout1[1] > minlevel)
	  memset((char *)gLout1, 0, (NZERO * nYsteps * sizeof(int32_t)));
	if (result == kStopWasDone)
	  return(result);
	for (i=1; i<nYsteps; i++)
	  {
	    i1 = i;
	    i2 = nYsteps  - i;
	    dblX1  = (double)endX + (double)(i * delPos);
	    dblY1  = (double)endY + (double)(i * delPos);
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

  if (gMultipleSweeps >= 8)
    {
      sweep = 8;
      for (dblX = Xmid, dblY = Ylow; ((dblX >Xlow) && (dblY<Ymid)); dblX -= Dstep, dblY += Dstep)
	{
	  if (IfStopThenStopAndNeg1Else0(pLgMaster))
	    return(kStopWasDone);
	  startX = kMask & (int32_t)dblX;
	  startY = kMask & (int32_t)dblY;
	  for (i=1; i<nYsteps; i++)
	    {
	      gLout1[i] = 0;
	      gLout2[i] = 0;
	    }
	  xydata.xdata = startX & kMaxSigned;
	  xydata.ydata = startY & kMaxSigned;
	  xydelta.xdata = delPos & kMaxSigned;
	  xydelta.ydata = delPos & kMaxSigned;
	  result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
				 (struct lg_xydata *)&xydelta, nYsteps, gLout1);
	  if (IfStopThenStopAndNeg1Else0(pLgMaster))
	    return(kStopWasDone);
	  if (gLout1[1] > minlevel)
	    memset((char *)gLout1, 0, (NZERO * nYsteps * sizeof(int32_t)));
	  if (result == kStopWasDone)
	    return(result);
	  for (i=1; i<nYsteps; i++)
	    {
	      i1 = i;
	      i2 = nYsteps  - i;
	      dblX1  = (double)startX + (double)(i * delPos);
	      dblY1  = (double)startY + (double)(i * delPos);
	      gSaveMatch[gLoutIndex] = 0;
	      gSaveSweep[gLoutIndex] = 0;
	      if (gLout1[i1] > minlevel)
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
	  endX = kMask & (int32_t)(dblX + (Dstep * nYsteps));
	  endY = kMask & (int32_t)(dblY + (Dstep * nYsteps));
	  xydata.xdata = endX & kMaxSigned;
	  xydata.ydata = endY & kMaxSigned;
	  xydelta.xdata = delNeg & kMaxSigned;
	  xydelta.ydata = delNeg & kMaxSigned;
	  result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
				 (struct lg_xydata *)&xydelta, nYsteps, gLout1);
	  if (IfStopThenStopAndNeg1Else0(pLgMaster))
	    return(kStopWasDone);
	  if (gLout1[1] > minlevel)
	    memset((char *)gLout1, 0, (NZERO * nYsteps * sizeof(int32_t)));
	  if (result == kStopWasDone)
	    return(result);
	  for (i=1; i<nYsteps; i++)
	    {
	      i1 = i;
	      i2 = nYsteps  - i;
	      dblX1  = (double)endX + (double)(i * delNeg);
	      dblY1  = (double)endY + (double)(i * delNeg);
	      gSaveMatch[gLoutIndex] = 0;
	      gSaveSweep[gLoutIndex] = 0;
	      if (gLout1[i1] > minlevel)
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

  if (dCount < 3)
    return(kSuperFineNotFound);
  avgX = sumX / dCount;
  avgY = sumY / dCount;
  *foundX = (int32_t)avgX;
  *foundY = (int32_t)avgY;
  if (gSuperIndex > 3)
    {
      halfIndex = gSuperIndex / 2;
      qsort(gXsuperSave, gSuperIndex, sizeof(int32_t), int32_tsort );
      qsort(gYsuperSave, gSuperIndex, sizeof(int32_t), int32_tsort );
      avgX = (double)(gXsuperSave[halfIndex]);
      avgY = (double)(gYsuperSave[halfIndex]);
      *foundX = (int32_t)avgX;
      *foundY = (int32_t)avgY;
    }
  return(0);
}

int int32_tsort( const void *elem1, const void *elem2 )
{
    int32_t numone, numtwo;

    numone = *(const int32_t *)elem1;
    numtwo = *(const int32_t *)elem2;

    if ( numone < numtwo ) return -1;
    if ( numone > numtwo ) return  1;

    return 0;
}
