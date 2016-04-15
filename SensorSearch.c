/*
static char rcsid[] = "$Id$";

*/
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
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
#include "parse_data.h"
#include "DoCoarseScan.h"
#include "DoCoarseScan2.h"
#include "LaserInterface.h"
#include "LaserCmds.h"

#define NZERO 5
#define kFineSearchStep                     0x2
#define kFineSearchSpanSteps                70
#define kSuperFineSearchStep                0x1
#define kSuperFineSearchSpanSteps           256
#define kNumberOfCrossesToAverage           7
#define kNumberOfCrosses                    7
#define kNumberOfSuperCrossesToAverage      2
#define kDELLEV                             60

int     gNumberOfSensorSearchAttempts  = 2;
int gSuperIndex = 0;
int16_t *gXsuperSave;
int16_t *gYsuperSave;
uint8_t *gSaveMatch;
uint8_t *gSaveSweep;
uint16_t *gSaveLout1;
uint16_t *gSaveLout2;
double * gSaveDblX1;
double * gSaveDblX2;
double * gSaveDblY1;
double * gSaveDblY2;
char ** gSuperReturn;
int    gLoutIndex = 0;
int32_t   gLoutCount = 0;
int32_t   gLoutSize = 0;
char * gLoutBase;
char * gLoutPtr;
double gFixX[ kNumberDrift ];
double gFixY[ kNumberDrift ];
double gDriftX[ kNumberDrift ];
double gDriftY[ kNumberDrift ];
uint16_t * gLout;
uint16_t * gLout1;
uint16_t * gLout2;
double g_aX = 0.0;
double g_bX = 1.0;
double g_aY = 0.0;
double g_bY = 1.0;
int gTargetDrift = 0;
uint32_t gSuperFineSearchStep = kSuperFineSearchStep;
uint32_t gSuperFineFactor = 1;

//static int sensor_sort( const void *elem1, const void *elem2 );
static int
findFirstLast(struct lg_master *pLgMaster, int16_t *firstX, int16_t *lastX,
	      int16_t *firstY, int16_t *lastY, int16_t *foundX, int16_t *foundY,
	      int16_t currentX, int16_t currentY, int16_t xStep, int16_t yStep,
	      uint16_t nSteps);
static int DoFineLevel(struct lg_master *pLgMaster, int16_t inpX, int16_t inpY,
		       int16_t *foundX, int16_t *foundY, double *outMinX,
		       double *outMinY, double *outMaxX, double *outMaxY);
static int SuperSearch(struct lg_master *pLgMaster, int16_t *foundX,
		       int16_t *foundY, double *dMinX, double *dMinY,
		       double *dMaxX, double * dMaxY);
void limitCalc(int16_t centerX, int16_t centerY, int16_t *eolXNeg,
	       int16_t *eolXPos, int16_t *eolYNeg, int16_t *eolYPos, int32_t nSteps);
static int CoarseLeg(struct lg_master *pLgMaster, int16_t Xin, int16_t Yin,
		     int16_t delX, int16_t delY, uint32_t nStepsIn,
		     int16_t *foundX, int16_t *foundY);
static int DoQuickFineSearch(struct lg_master *pLgMaster, int16_t *foundX, int16_t *foundY);
static void AdjustXYLimit(int16_t *eolXPos, int16_t *eolXNeg, int16_t *eolYPos, int16_t *eolYNeg, int16_t delta);
int compare ( const void * a, const void * b );

int QuickCheckASensor(struct lg_master *pLgMaster, int16_t centerX, int16_t centerY)
{
	struct lg_xydata  xydata;
        int theResult;
	int nTries;
        int nSuccess;
        int16_t tempX, tempY;
	
	memset((char *)&xydata, 0, sizeof(struct lg_xydata));
	
        if ((centerX == kMaxUnsigned) || (centerY == kMaxUnsigned))
	  return 0;

        tempX = centerX;
        tempY = centerY;

	xydata.xdata =  centerX;
	xydata.ydata =  centerY;
        move_dark(pLgMaster, (struct lg_xydata *)&xydata);
	usleep(250);
/*
 *  a more "robusty" quick check
 *    - certain percentage of quick checks must succeed (desire 30%)
 *    - minimum of 2 quick checks must succeed
 *    try up to 30 times.
 */
#ifdef FINESEARCH
        theResult = DoFineSearch( &tempX, &tempY );
        if(theResult == kStopWasDone) {
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
	while( nTries < kMaxQuickSearches ) {
		nTries++;
        	tempX = centerX;
        	tempY = centerY;
        	theResult = DoQuickFineSearch (pLgMaster, &tempX, &tempY );
                if(theResult == kStopWasDone) {
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
        return theResult;
}

int QuickCheckOne(struct lg_master *pLgMaster, int16_t centerX, int16_t centerY,
		  int16_t *foundX, int16_t *foundY)
{
	struct lg_xydata  xydata;
        int theResult;
	int nTries;
        int nSuccess;
        int16_t tempX, tempY;

	memset((char *)&xydata, 0, sizeof(struct lg_xydata));
	
        if ((centerX == kMaxUnsigned) || (centerY == kMaxUnsigned))
	  return 0;

        tempX = centerX;
        tempY = centerY;

	xydata.xdata =  centerX;
	xydata.ydata =  centerY;
        move_dark(pLgMaster, (struct lg_xydata *)&xydata);
	usleep(250);
	
	//        SearchBeamOn  (pLgMaster);
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
	while( nTries < kMaxQuickSearches ) {
		nTries++;
        	tempX = centerX;
        	tempY = centerY;
        	theResult = DoQuickFineSearch(pLgMaster, &tempX, &tempY );

                if(theResult == kStopWasDone) {
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
        
        *foundX = tempX;
        *foundY = tempY;
        return theResult;
}
int SearchForASensor(struct lg_master *pLgMaster, int16_t startX, int16_t startY,
		      int16_t *foundX, int16_t *foundY)
{
    struct lg_xydata   xydata;
    struct lg_xydelta  xydelta;
    double   dMinX=0.0;
    double   dMinY=0.0;
    double   dMaxX=0.0;
    double   dMaxY=0.0;
    int32_t  posStepSize, negStepSize;
    int32_t  xSteps, ySteps;
    uint32_t nSteps;
    uint32_t longFactor;
    int32_t  i;
    int16_t tempX, tempY;
    int16_t eolXPos, eolYPos;
    int16_t eolXNeg, eolYNeg;
    int16_t f1x=0,f1y=0;
    int16_t f2x=0,f2y=0;
    int16_t avgX, avgY;
    int16_t centX, centY;
    int16_t sense_step;
    int16_t new_step;
    int16_t HatchCount;
    int theResult;
        
    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    memset((char *)&xydelta, 0, sizeof(struct lg_xydata));
	
    xydata.xdata =  startX;
    xydata.ydata =  startY;
    // Move mirrors to XY in the dark to avoid ghost/tail
    move_dark(pLgMaster, (struct lg_xydata *)&xydata);
    usleep(250);
    eolXPos = startX;
    eolYPos = startY;
    eolXNeg = startX;
    eolYNeg = startY;
    pLgMaster->gHeaderSpecialByte = 0x40;
    if (pLgMaster->LongOrShortThrowSearch == 1)
      {
	for (longFactor = 1; longFactor <= 4 ; longFactor *= 2)
	  {
	    theResult = DoCoarseScan(pLgMaster, xydata.xdata, xydata.ydata, 0x4,
				     32 * longFactor, &f1x, &f1y);
	    if (theResult == kStopWasDone)
	      return theResult;
	    if (theResult == kCoarseNotFound)
	      continue;
	    theResult = DoCoarseScan2(pLgMaster, xydata.xdata, xydata.ydata,
				      0x4, 32 * longFactor, &f2x, &f2y);
	    if (theResult == kStopWasDone)
	      return theResult;
	    if (theResult == kCoarseNotFound)
	      continue;
	    avgX = f2x;
	    avgY = f1y;
	    theResult = DoFineLevel(pLgMaster, avgX, avgY, foundX, foundY, &dMinX, &dMinY,
				    &dMaxX, &dMaxY);
	    if (theResult == kStopWasDone)
	      return theResult;
	    if (theResult == kFineNotFound)
	      continue;
	    theResult = SuperSearch(pLgMaster, foundX, foundY, &dMinX, &dMinY, &dMaxX, &dMaxY);
	    if (theResult)
	      return(theResult);
	    if (theResult == kStopWasDone)
	      return theResult;
	    if (theResult == kSuperFineNotFound)
	      continue;
	    if (theResult == 0)
	      {
		*foundX = avgX;
		*foundY = avgY;
		return(0);
	      }
	  }
	if (theResult)
	  return(theResult);
	*foundX = avgX;
	*foundY = avgY;
	return(0);
      }
    HatchCount       =  pLgMaster->gHatchFactor / pLgMaster->gCoarse2Factor;
    pLgMaster->gNumberOfSpirals =  pLgMaster->gSpiralFactor / pLgMaster->gCoarse2Factor;

    theResult = kCoarseNotFound;
    sense_step = HatchCount * pLgMaster->gCoarse2SearchStep;
    limitCalc(startX, startY, &eolXNeg, &eolXPos, &eolYNeg, &eolYPos, sense_step);
    for (i = 0; i < (2*(HatchCount-2)); i++)
      {
	new_step = ((2*i) + 1) * pLgMaster->gCoarse2SearchStep;
	tempX = startX + new_step;
	tempY = startY + new_step;
	// Make sure we don't go out of bounds to fault hardware
	limitCalc(tempX, tempY,&eolXNeg, &eolXPos, &eolYNeg, &eolYPos, new_step);
	posStepSize = pLgMaster->gCoarse2SearchStep;
	negStepSize = -pLgMaster->gCoarse2SearchStep;
	xSteps = (eolXPos - eolXNeg) / posStepSize;
	ySteps = (eolYPos - eolYNeg) / posStepSize;
	// Search around XNeg & Y
	theResult = CoarseLeg(pLgMaster, eolXNeg, tempY, posStepSize, 0, xSteps,
			      foundX, foundY);
	if (theResult == kStopWasDone)
	  return theResult;
	if (theResult == 0)
	  break;
	// Search around X & NegY
	theResult = CoarseLeg(pLgMaster, tempX, eolYNeg, 0, posStepSize, ySteps,
			      foundX, foundY);
	if (theResult == kStopWasDone)
	  return theResult;
	if (theResult == 0)
	  break;
      }
    if (theResult != 0)
      {
	for (i= 0; i < (pLgMaster->gNumberOfSpirals - (HatchCount/2)); i++)
	  {
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);

	    AdjustXYLimit(&eolXPos, &eolXNeg, &eolYPos, &eolYNeg, pLgMaster->gCoarse2SearchStep);
	    posStepSize = pLgMaster->gCoarse2SearchStep;
	    negStepSize = -pLgMaster->gCoarse2SearchStep;
	    xSteps = (eolXPos - eolXNeg) / posStepSize;
	    ySteps = (eolYPos - eolYNeg) / posStepSize;
 
	    // Search around NegX/NegY
	    theResult = CoarseLeg(pLgMaster, eolXNeg, eolYNeg, posStepSize, 0, xSteps,
				  foundX, foundY);
	    if (theResult == kStopWasDone)
	      return(theResult);
	    if (theResult == 0)
	      break;
	    // Search around PosX/NegY
	    theResult = CoarseLeg(pLgMaster, eolXPos, eolYNeg, 0, posStepSize, ySteps,
				  foundX, foundY);
	    if (theResult == kStopWasDone)
	      return theResult;
	    if (theResult == 0)
	      break;
	    // Search around PosX/PosY
	    theResult = CoarseLeg(pLgMaster, eolXPos, eolYPos, negStepSize, 0, xSteps,
				  foundX, foundY);
	    if (theResult == kStopWasDone)
	      return(theResult);
	    if (theResult == 0)
	      break;
	    // Search around NegX/PosY
	    theResult = CoarseLeg(pLgMaster, eolXNeg, eolYPos, 0, negStepSize, ySteps,
				  foundX, foundY);
	    if (theResult == kStopWasDone)
	      return theResult;
	    if (theResult == 0)
	      break;
	  }
      }
    if ((theResult == 0) && ((*foundX == 0) || (*foundY == 0)))
      {
	syslog(LOG_ERR, "SearchForASensor:  Indicates found but XY pair invalid");
	return(kCoarseNotFound);
      }
    if ((theResult == 0) && (pLgMaster->gDwell > 0))
      {
	for (i=0; i<pLgMaster->gDwell; i++)
	  {
	    nSteps =  32; 
	    centX = *foundX;
	    centY = *foundY - 64;
	    xydata.xdata =  centX;
	    xydata.ydata =  centY;
	    xydelta.xdata = 0;
	    xydelta.ydata = 1;
	    if (DoLineSearch(pLgMaster, (struct lg_xydata *)&xydata,
			     (struct lg_xydelta *)&xydelta, nSteps))
	      {
		return kStopWasDone;
	      }
	    centX = *foundX - 64;
	    centY = *foundY;
	    xydata.xdata =  centX;
	    xydata.ydata =  centY;
	    xydelta.xdata = 1;
	    xydelta.ydata = 0;
	    if (DoLineSearch(pLgMaster, (struct lg_xydata *)&xydata,
			     (struct lg_xydelta *)&xydelta, nSteps))
	      {
		return kStopWasDone;
	      }
	  }
      }
    if (theResult == 0)
      {
	xydata.xdata =  *foundX;
	xydata.ydata =  *foundY;
	syslog(LOG_NOTICE, "SearchForASensor: Found target startX=%x,startY=%x,foundX=%x,foundY=%x", startX,startY,*foundX,*foundY);
      }
    else
      {
	syslog(LOG_NOTICE, "SearchForASensor: Not Found for startX=%x,startY=%x, rc=%x", startX,startY,theResult);
	xydata.xdata =  startX;
	xydata.ydata =  startY;
      }
    syslog(LOG_NOTICE, "SearchForASensor: move-dark x=%x,y=%x", xydata.xdata,xydata.ydata);
    move_one_dark(pLgMaster, (struct lg_xydata *)&xydata);
    return(theResult);
}

static int CoarseLeg(struct lg_master *pLgMaster, int16_t Xin, int16_t Yin,
		     int16_t delX, int16_t delY, uint32_t nStepsIn,
		     int16_t *foundX, int16_t *foundY)
{
    struct lg_xydata  xydata;
    struct lg_xydelta xydelta;
    int16_t           tmpX, tmpY;
    int16_t           eolX, eolY;
    int16_t           avgX, avgY;
    double dMinX;
    double dMinY;
    double dMaxX;
    double dMaxY;
    int32_t      sumX=0, sumY=0;
    uint32_t     count=0;
    uint32_t     i;
    int          theResult;

    // Initialize variables and buffers to be used
    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    memset((char *)&xydelta, 0, sizeof(struct lg_xydata));
    theResult = kCoarseNotFound;

    // Start searching
    xydata.xdata =  Xin;
    xydata.ydata =  Yin;
    xydelta.xdata = delX;
    xydelta.ydata = delY;
    if (DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
		      (struct lg_xydelta*)&xydelta, nStepsIn, gLout, 1))
      {
	return(kStopWasDone);
      }
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return(kStopWasDone);
    //    memcpy((char *)pLgMaster->gLsort, (char *)gLout, nStepsIn*sizeof(int16_t));
    //    qsort(pLgMaster->gLsort, nStepsIn, sizeof(int16_t), sensor_sort);
    for (i = 0; i < nStepsIn; i++)
      {
	tmpX = Xin + (i * delX);
	tmpY = Yin + (i * delY);
	if (gLout[i] < SENSE_MAX_THRESHOLD)
	  {
	    sumX += tmpX;
	    sumY += tmpY;
	    count++;
	  }
      }
    if (count >= 1)
      {
	avgX = (int16_t)(sumX/count);
	avgY = (int16_t)(sumY/count);
	tmpX = avgX;
	tmpY = avgY;
	//
	// center back-search on found point
	//
	eolX = tmpX - (nStepsIn / 2 * delX);
	eolY = tmpY - (nStepsIn / 2 * delY);
	if (eolX <= kMinSigned)
	  eolX = kMinSigned;
	if (eolY <= kMinSigned)
	  eolY = kMinSigned;
	xydata.xdata =  eolX;
	xydata.ydata =  eolY;
	xydelta.xdata = delX;
	xydelta.ydata = delY;
	if (DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
			  (struct lg_xydelta*)&xydelta, nStepsIn, gLout, 1))
	  return(kStopWasDone);
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
                 
	tmpX = eolX;
	tmpY = eolY;
	sumX = 0;
	sumY = 0;
	count = 0;
	for (i=0; i < nStepsIn; i++)
	  {
	    tmpX += delX;
	    tmpY += delY; 
	    if (gLout[i] < SENSE_MAX_THRESHOLD)
	      {
		sumX += tmpX;
		sumY += tmpY;
		count++;
	      }
	  }
	if (count >= 1)
	  {
	    // Coarse search found target approximate location.  Finer search will hone on more-specific location
	    avgX = (int16_t)(sumX/count);
	    avgY = (int16_t)(sumY/count);
	    syslog(LOG_NOTICE,"CoarseLeg Found %d points, avgX=%x,avgY=%x.  Move on to DoFineLevel.",count,avgX,avgY);
	    theResult = DoFineLevel(pLgMaster, avgX, avgY, foundX, foundY, &dMinX, &dMinY,
				    &dMaxX, &dMaxY);
	    if (theResult)
	      return(theResult);

	    // Coarse & Fine searches found target where-abouts.  Now time to get more exact location
	    avgX = *foundX;
	    avgY = *foundY;
	    syslog(LOG_NOTICE,"CoarseLeg->SuperSearch: DoFineLevel Found target at x=%x,y=%x,rc=%x",avgX,avgY,theResult);
	    theResult = SuperSearch(pLgMaster, foundX, foundY, &dMinX, &dMinY, &dMaxX, &dMaxY);
	    if (theResult == 0)
	      {
		syslog(LOG_NOTICE,"SuperSearch:  Found target at x=%x,y=%x",avgX,avgY);
		*foundX = avgX;
		*foundY = avgY;
		return(0);
	      }
	  }
      }
    return(theResult);
}        
static int DoQuickFineSearch(struct lg_master *pLgMaster, int16_t *foundX, int16_t *foundY)
{
    struct lg_xydata xydata;
    struct lg_xydelta xydelta;
    uint32_t         j, rc, index, nSteps,finetest, theSpan;
    int16_t          currentX, currentY;
    int16_t          centerX, centerY;
    int16_t          delX, delY;
    int16_t          tmpX, tmpY;
    uint16_t         numberOfXScansToAverage;
    uint16_t         numberOfYScansToAverage;
        
    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    memset((char *)&xydelta, 0, sizeof(struct lg_xydata));
    theSpan = kSuperFineSearchStep * kSuperFineSearchSpanSteps;

    centerX = *foundX;
    centerY = *foundY;

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
#ifdef AGS_DEBUG
	syslog(LOG_DEBUG,"QKFINESRCH: DOLVL1 for x=%x,y=%x,delta x=%x,y=%x,nSteps %d",
	       currentX,currentY,delX,0,nSteps);
#endif
	xydata.xdata =  currentX;
	xydata.ydata =  currentY;
	xydelta.xdata = delX;
	xydelta.ydata = 0;
	rc = DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
		      (struct lg_xydelta*)&xydelta, nSteps, gLout,1);
	if (rc)
	  return(kStopWasDone);
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);

	tmpX = currentX;
	if (j > 1)
	  {
	    for (index=0; index<nSteps; index++)
	      {
		if (gLout[index] < SENSE_MAX_THRESHOLD)
		  {
		    numberOfXScansToAverage++;
		    finetest = finetest | 0x01;
		  }
		tmpX += delX;
	      }
	  }
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);

	/* Go left */
	currentX = centerX + theSpan;
	currentY = centerY;
	delX = -2 * gSuperFineSearchStep;
	nSteps = theSpan / gSuperFineSearchStep;
	xydata.xdata =  currentX;
	xydata.ydata =  currentY;
	xydelta.xdata = delX;
	xydelta.ydata = 0;
	rc = DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
		      (struct lg_xydelta*)&xydelta, nSteps, gLout,1);
	if (rc)
	  return(kStopWasDone);
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
	tmpX = currentX;
	if (j > 1)
	  {
	    for (index=0; index < nSteps; index++)
	      {
		if (gLout[index] < SENSE_MAX_THRESHOLD)
		  {
		    numberOfXScansToAverage++;
		    finetest = finetest | 0x02;
		  }
		tmpX += delX;
	      }
	  }

	/* Go up    */
	currentX = centerX;
	currentY = centerY - theSpan;
	delY = 2*gSuperFineSearchStep;
	nSteps = theSpan / gSuperFineSearchStep;
	xydata.xdata =  currentX;
	xydata.ydata =  currentY;
	xydelta.ydata = delY;
	xydelta.xdata = 0;
#ifdef AGS_DEBUG
	syslog(LOG_DEBUG,"QKFINESRCH: DOLVL3 for x=%x,y=%x,delta x=%x,y=%x,nSteps %d",
	       currentX,currentY,0,delY,nSteps);
#endif
	rc = DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
		      (struct lg_xydelta*)&xydelta, nSteps, gLout,1);
	if (rc)
	  return(kStopWasDone);
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	    return(kStopWasDone);
	tmpY = currentY;
	if (j > 1)
	  {
	    for (index = 0; index<nSteps; index++)
	      {
		if (gLout[index] < SENSE_MAX_THRESHOLD)
		  {
		    numberOfYScansToAverage++;
		    finetest = finetest | 0x04;
		  }
	        tmpY += delY;
	      }
	  }
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);

	/* Go down */
	currentX = centerX;
	currentY = centerY + theSpan;
	delY = -2 * gSuperFineSearchStep;
	nSteps = theSpan / gSuperFineSearchStep;
	xydata.xdata =  currentX;
	xydata.ydata =  currentY;
	xydelta.ydata = delY;
	xydelta.xdata = 0;
#ifdef AGS_DEBUG
	syslog(LOG_DEBUG,"QKFINESRCH: DOLVL4 for x=%x,y=%x,delta x=%x,y=%x,nSteps %d",
	       currentX,currentY,0,delY,nSteps);
#endif
	rc = DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
		      (struct lg_xydelta*)&xydelta, nSteps, gLout, 1);
	if (rc)
	  return(kStopWasDone);
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
	tmpY = currentY;
	if (j > 1)
	  {
	    for (index=0; index < nSteps; index++)
		  {
		    if (gLout[index] < SENSE_MAX_THRESHOLD)
		      {
			numberOfYScansToAverage++;
			finetest = finetest | 0x08;
		      }
		    tmpY += delY;
		  }
	      }
	    if (finetest == 0x0f)
	      break;
      }
    if (finetest == 0x0f)
      {
	*foundX = currentX;
	*foundY = currentY;
	return(0);
      }
    if ((numberOfXScansToAverage <= 2) && (numberOfYScansToAverage <= 2))
      return(kSuperFineTooFew);
    return(kSuperFineNotFound);
}

// FIME--PAH--NEED TO CHECK EVERY SINGLE POINTER FROM CALLOC
void InitSensorSearch(void)
{
    int i;

    gLout = (uint16_t *)calloc((size_t)MAX_TGFIND_BUFFER,sizeof(uint16_t));
    gLout1 = (uint16_t *)calloc((size_t)MAX_TGFIND_BUFFER,sizeof(uint16_t));
    gLout2 = (uint16_t *)calloc((size_t)MAX_TGFIND_BUFFER,sizeof(uint16_t));
    gSaveMatch = (uint8_t *)calloc((size_t)500000,sizeof(uint8_t)); 
    gSaveSweep = (uint8_t *)calloc((size_t)500000,sizeof(uint8_t)); 
    gSaveLout1 = (uint16_t *)calloc((size_t)500000,sizeof(uint16_t)); 
    gSaveLout2 = (uint16_t *)calloc((size_t)500000,sizeof(uint16_t)); 
    gSaveDblX1 = (double *)calloc((size_t)500000,sizeof(double)); 
    gSaveDblX2 = (double *)calloc((size_t)500000,sizeof(double)); 
    gSaveDblY1 = (double *)calloc((size_t)500000,sizeof(double)); 
    gSaveDblY2 = (double *)calloc((size_t)500000,sizeof(double)); 
    gXsuperSave = (int16_t *)calloc((size_t)500000,sizeof(int16_t)); 
    gYsuperSave = (int16_t *)calloc((size_t)500000,sizeof(int16_t)); 
    gLoutBase   = (char *)calloc((size_t)6000000,sizeof(int16_t)); 
    gSuperReturn   = (char **)calloc((size_t)1024,sizeof(char *)); 
    if (gSuperReturn)
      {
	for ( i=0; i<1024; i++)
	  gSuperReturn[i]   = (char *)calloc((size_t)1024,sizeof(char));
      }
    return;
}
void ClearSensorBuffers(void)
{
    if (gSaveMatch)
      memset((char *)gSaveMatch, 0, (500000 * sizeof(uint8_t)));
    if (gSaveSweep)
      memset((char *)gSaveSweep, 0, (500000 * sizeof(uint8_t)));
    if (gXsuperSave)
      memset((char *)gXsuperSave, 0, (500000 * sizeof(int16_t)));
    if (gYsuperSave)
      memset((char *)gYsuperSave, 0, (500000 * sizeof(int16_t)));
    if (gSaveLout1)
      memset((char *)gSaveLout1, 0, (500000 * sizeof(uint16_t)));
    if (gSaveLout2)
      memset((char *)gSaveLout2, 0, (500000 * sizeof(uint16_t)));
    if (gSaveDblX1)
      memset((char *)gSaveDblX1, 0, (500000 * sizeof(double)));
    if (gSaveDblX2)
      memset((char *)gSaveDblX2, 0, (500000 * sizeof(double)));
    if (gSaveDblY1)
      memset((char *)gSaveDblY1, 0, (500000 * sizeof(double)));
    if (gSaveDblY2)
      memset((char *)gSaveDblY2, 0, (500000 * sizeof(double)));
    if (gLoutBase)
      memset((char *)gLoutBase, 0, (6000000 * sizeof(int16_t)));
    return;
}
void CloseSensorSearch(void)
{
    int i;

    if (gLout)  free(gLout);
    if (gLout1) free(gLout1);
    if (gLout2) free(gLout2);
    if (gLoutBase) free(gLoutBase);
    if (gSaveMatch) free(gSaveMatch);
    if (gSaveSweep) free(gSaveSweep);
    if (gSaveLout1) free(gSaveLout1);
    if (gSaveLout2) free(gSaveLout2);
    if (gSaveDblX1) free(gSaveDblX1);
    if (gSaveDblX2) free(gSaveDblX2);
    if (gSaveDblY1) free(gSaveDblY1);
    if (gSaveDblY2) free(gSaveDblY2);
    if (gXsuperSave) free(gXsuperSave);
    if (gYsuperSave) free(gYsuperSave);
    if (gSuperReturn)
      {
	for ( i=0; i<1024; i++)
	  if (gSuperReturn[i]) free(gSuperReturn[i]);
	free(gSuperReturn);
      }
    return;
}

void CorrectDrift(int16_t cX, int16_t cY, int16_t *fX, int16_t *fY)
{
     double doubleX, doubleY;
     double correctX, correctY;

     doubleX = (double)cX;
     doubleY = (double)cY;

     correctX = g_aX + (g_bX * doubleX);
     correctY = g_aY + (g_bY * doubleY);
     *fX = (int16_t)correctX;
     *fY = (int16_t)correctY;
     return;
}

void FindDrift(int16_t cX, int16_t cY, int16_t fX, int16_t fY)
{
     double driftX, driftY;
     double fixX, fixY;
     int i;
     double sumD, sumF, sumFF, sumFD;
     double delta;
     double aX, aY;

     fixX = (double)cX;
     fixY = (double)cY;
     driftX = (double)fX;
     driftY = (double)fY;

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

     aX = g_aX / 65536.0;
     aY = g_aY / 65536.0;
     syslog(LOG_NOTICE, "aXbXaYbY %12.2f %12.6f %12.2f %12.6f", aX, g_bX, aY, g_bY);
     return;
}

void InitDrift(int16_t *Xarr, int16_t *Yarr)
{
    int i, j;

    g_aX = 0.0;  g_bX = 1.0;  g_aY = 0.0; g_bY = 1.0;
    for( i = 0; i < kNumberDrift; i++ ) {
       j = i % kNumberOfRegPoints;
       gFixX[i]   = (double)Xarr[j];
       gDriftX[i] = (double)Xarr[j];
       gFixY[i]   = (double)Yarr[j];
       gDriftY[i] = (double)Yarr[j];
    }
    return;
}

static void SetFineLevelResults(int16_t firstX,int16_t firstY, int16_t lastX, int16_t lastY,
				double *dMinX, double *dMinY, double *dMaxX, double *dMaxY)
{
    double dFX;
    double dFY;
    double dLX;
    double dLY;

    dFX = (double)firstX;
    dFY = (double)firstY;
    dLX = (double)lastX;
    dLY = (double)lastY;
    if (dFX < *dMinX)
      *dMinX = dFX;
    if (dFY < *dMinY)
      *dMinY = dFY;
    if (dFX > *dMaxX)
      *dMaxX = dFX;
    if (dFY > *dMaxY)
      *dMaxY = dFY;
    if (dLX < *dMinX)
      *dMinX = dLX;
    if (dLY < *dMinY)
      *dMinY = dLY;
    if (dLX > *dMaxX)
      *dMaxX = dLX;
    if (dLY > *dMaxY)
      *dMaxY = dLY;
      return;
}
static int DoFineLevel(struct lg_master *pLgMaster, int16_t inpX, int16_t inpY,
		       int16_t *foundX, int16_t *foundY, double *outMinX,
		       double *outMinY, double *outMaxX, double *outMaxY)
{
    double dMinX=0.0, dMinY=0.0, dMaxX=0.0, dMaxY=0.0; 
    int    result;
    uint32_t stepSize, negStep, nSteps;
    int16_t eolXPos=0, eolYPos=0, eolXNeg=0, eolYNeg=0;
    int16_t currentX, currentY, centerX, centerY;
    int16_t firstX, firstY, lastX, lastY;
    int16_t Xm, Ym;
        
    nSteps = 140;
    // Seed with starting input XY pair
    centerX = inpX;
    centerY = inpY;
    dMinX   = (double)inpX;
    dMaxX   = (double)inpX;
    dMinY   = (double)inpY;
    dMaxY   = (double)inpY;

    stepSize = kFineSearchStep;
    negStep  = -kFineSearchStep;
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return kStopWasDone;
	
    // Start with NegX/NegY pair
    // starting Y, NegX
    limitCalc(centerX, centerY, &eolXNeg, &eolXPos, &eolYNeg, &eolYPos, stepSize * nSteps);
    currentX = eolXNeg;
    currentY = centerY;
    result =  findFirstLast(pLgMaster, &firstX, &firstY, &lastX, &lastY, &Xm, &Ym, currentX,
			    currentY, stepSize, 0, nSteps);
    if (result == kStopWasDone)
      return(result);
    // Set up for next round, look for min/max of current XY pair
    SetFineLevelResults(firstX, firstY, lastX, lastY, &dMinX, &dMinY, &dMaxX, &dMaxY);
    // Starting X, NegY
    currentX = centerX;
    currentY = eolYNeg;
    result =  findFirstLast(pLgMaster, &firstX, &firstY, &lastX, &lastY, &Xm, &Ym, currentX, currentY,
			    0, stepSize, nSteps);
    if (result == kStopWasDone)
      return(result);
    // Set up for next round, look for min/max of current XY pair
    SetFineLevelResults(firstX, firstY, lastX, lastY, &dMinX, &dMinY, &dMaxX, &dMaxY);
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return kStopWasDone;

    //  End of original XY pair, on to next set.
    centerX = Xm;
    centerY = Ym;
    limitCalc(centerX, centerY, &eolXNeg, &eolXPos, &eolYNeg, &eolYPos, stepSize * nSteps);
    // ctrX, NegY
    currentX = centerX;
    currentY = eolYNeg;
    result =  findFirstLast(pLgMaster, &firstX, &firstY, &lastX, &lastY,
			      &Xm, &Ym, currentX, currentY, 0, stepSize,
			      nSteps);
    if (result == kStopWasDone)
      return(result);
    // Set up for next round, look for min/max of current XY pair
    SetFineLevelResults(firstX, firstY, lastX, lastY, &dMinX, &dMinY, &dMaxX, &dMaxY);
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return kStopWasDone;

    centerX = Xm;
    centerY = Ym;
    limitCalc(centerX, centerY, &eolXNeg, &eolXPos, &eolYNeg, &eolYPos, stepSize * nSteps);
    // PosX, Ctr Y
    currentX = eolXPos;
    currentY = centerY;
    result =  findFirstLast(pLgMaster, &firstX, &firstY, &lastX, &lastY, &Xm,
			    &Ym, currentX, currentY, negStep, 0, nSteps);
    if (result == kStopWasDone)
      return(result);
    // Set up for next round, look for min/max of current XY pair
    SetFineLevelResults(firstX, firstY, lastX, lastY, &dMinX, &dMinY, &dMaxX, &dMaxY);
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return kStopWasDone;

    // Use new center XY pair
    centerX = Xm;
    centerY = Ym;
    limitCalc(centerX, centerY, &eolXNeg, &eolXPos, &eolYNeg, &eolYPos, stepSize * nSteps);
    // NegX, CtrY
    currentX = eolXNeg;
    currentY = centerY;
    result =  findFirstLast(pLgMaster, &firstX, &firstY, &lastX, &lastY, &Xm,
			    &Ym, currentX, currentY, stepSize, 0, nSteps);
    if (result == kStopWasDone)
      return(result);
    // Set up for next round, look for min/max of current XY pair
    SetFineLevelResults(firstX, firstY, lastX, lastY, &dMinX, &dMinY, &dMaxX, &dMaxY);
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return kStopWasDone;

    // Use new center XY pair
    centerX = Xm;
    centerY = Ym;
    limitCalc(centerX, centerY, &eolXNeg, &eolXPos, &eolYNeg, &eolYPos, stepSize * nSteps);
    // PosY, CtrX
    currentX = centerX;
    currentY = eolYPos;
    result =  findFirstLast(pLgMaster, &firstX, &firstY, &lastX, &lastY, &Xm,
			    &Ym, currentX, currentY, 0, negStep, nSteps);
    if (result == kStopWasDone)
      return(result);
    // Last set of min/max calculations
    SetFineLevelResults(firstX, firstY, lastX, lastY, &dMinX, &dMinY, &dMaxX, &dMaxY);
    centerX = Xm;
    centerY = Ym;
    *outMinX = dMinX;
    *outMinY = dMinY;
    *outMaxX = dMaxX;
    *outMaxY = dMaxY;
    *foundX = centerX;
    *foundY = centerY;
    syslog(LOG_NOTICE,"DoFineLevel: Found x=%x,y=%x,minX=%x,maxX=%x,minY=%x,maxY=%x",
	   *foundX,*foundY,(int16_t)*outMinX,(int16_t)*outMaxX,(int16_t)*outMinY,(int16_t)*outMaxY);
    return(0);
}

static int findFirstLast(struct lg_master *pLgMaster, int16_t *firstX, int16_t *firstY,
			 int16_t *lastX, int16_t *lastY, int16_t *foundX,
			 int16_t *foundY, int16_t currentX, int16_t currentY,
			 int16_t xStep, int16_t yStep, uint16_t nSteps)
{
	struct lg_xydata xydata;
	struct lg_xydelta xydelta;
	double           sumX, sumY, count;
	int              i;
        int16_t          tmpX, tmpY;

	memset((char *)&xydata, 0, sizeof(struct lg_xydata));
	memset((char *)&xydelta, 0, sizeof(struct lg_xydata));

	// Coming into this function already have an approximate XY pair
	// so seed found XY with that set.
	// If new XY pair (ie. closer than original to pinpointed target
	// location, then will send that value.
	// Seed first/last.  First should always be first X/Y, since that's the starting point
	// They will be sorted later.
	*foundX = currentX;
	*foundY = currentY;
	*firstX = currentX;
	*firstY = currentY;
	*lastX = currentX;
	*lastY = currentY;
	
	xydata.xdata = currentX;
	xydata.ydata = currentY;
	xydelta.xdata = xStep;
	xydelta.ydata = yStep;
	if (DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
			  (struct lg_xydelta *)&xydelta, nSteps, gLout,1))
	  return kStopWasDone;
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;

	sumX = 0.0;
	sumY = 0.0;
	count = 0.0;
        for (i=0; i < nSteps; i++)
	  {
	    tmpX = currentX + (i * xStep);
	    tmpY = currentY + (i * yStep);
	    if (gLout[i] < SENSE_MAX_THRESHOLD)
	      {
		*lastX = tmpX;
		*lastY = tmpY;
		sumX += (double)tmpX;
		sumY += (double)tmpY;
		count++;
	      }
	  }
        if (count == 0)
	  return(kFineNotFound);
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;

	if (count >= 1)
	  {
	    *foundX = (int16_t)(sumX/count);
	    *foundY = (int16_t)(sumY/count);
	    syslog(LOG_NOTICE,"FindFirstLast: Found target at x=%x,y=%x",*foundX,*foundY);
	    return(0);
	  }
	else
	  syslog(LOG_NOTICE,"FindFirstLast: unable to find target for x=%x,y=%x",currentX,currentY);
        return kFineNotFound;
}

void limitCalc(int16_t centerX, int16_t centerY, int16_t *eolXNeg,
	       int16_t *eolXPos, int16_t *eolYNeg, int16_t *eolYPos, int32_t nSteps)
{
  if ((centerX - nSteps)  <= kMinSigned)
    {
      *eolXNeg = kMinSigned;
      *eolXPos = kMinSigned + nSteps;
    }
  else if ((centerX + nSteps) >= kMaxSigned)
    {
      *eolXPos = kMaxSigned;
      *eolXNeg = kMaxSigned - nSteps;
    }
  else
    {
      *eolXNeg = centerX - (nSteps/2);
      *eolXPos = centerX + (nSteps/2);
      if (*eolXPos >= kMaxSigned)
	*eolXPos = kMaxSigned;
    }
  if ((centerY - nSteps)  <= kMinSigned)
    {
      *eolYNeg = kMinSigned;
      *eolYPos = kMinSigned + nSteps;
    }
  else if ((centerY + nSteps) >= kMaxSigned)
    {
      *eolYPos = kMaxSigned;
      *eolYNeg = kMaxSigned - nSteps;
    }
  else
    {
      *eolYNeg = centerY - (nSteps/2);
      *eolYPos = centerY + (nSteps/2);
      if (*eolYPos >= kMaxSigned)
	*eolYPos = kMaxSigned;
    }
  return;
}

static void AdjustXYLimit(int16_t *eolXPos, int16_t *eolXNeg, int16_t *eolYPos, int16_t *eolYNeg, int16_t delta)
{
  if ((*eolXPos + delta) >= kMaxSigned)
      *eolXPos  = kMaxSigned;
  else
    *eolXPos += delta;
  if ((*eolXNeg - delta) <= kMinSigned)
      *eolXNeg  = kMinSigned;
  else
    *eolXNeg -= delta;
  if ((*eolYPos + delta) >= kMaxSigned)
    *eolYPos  = kMaxSigned;
  else
    *eolYPos += delta;
  if ((*eolYNeg - delta) <= kMinSigned)
    *eolYNeg  = kMinSigned;
  else
    *eolYNeg -= delta;
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

static int SearchWithTwoSetsOutY(struct lg_master *pLgMaster, int sweep, double inpX1,double inpY1, int16_t delX1, int16_t delY1,
				 double inpX2, double inpY2, int16_t delX2, int16_t delY2, uint16_t *inpBuf1, uint16_t *inpBuf2,
				 int steps, double *dCount, double *sumX, double *sumY) 
{
    struct lg_xydata  xydata;
    struct lg_xydelta xydelta;
    int               result;
    int16_t           inX1, inX2, inY1, inY2;
    int16_t           outY1, outY2;
    uint16_t          i;

    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    memset((char *)&xydelta, 0, sizeof(struct lg_xydata));

    inX1 = (int16_t)inpX1;
    inX2 = (int16_t)inpX2;
    inY1 = (int16_t)inpY1;
    inY2 = (int16_t)inpY2;
    xydata.xdata =  inX1;
    xydata.ydata =  inY1;
    xydelta.xdata = delX1;
    xydelta.ydata = delY1;
    result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
			   (struct lg_xydelta *)&xydelta, steps, inpBuf1,0);
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return(kStopWasDone);
    if (result == kStopWasDone)
      return(result);
    xydata.xdata =  inX2;
    xydata.ydata =  inY2;
    xydelta.xdata = delX2;
    xydelta.ydata = delY2;
    result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
			   (struct lg_xydelta *)&xydelta, steps, inpBuf2,0);
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return(kStopWasDone);
    if (result == kStopWasDone)
      return(result);
    for (i=0; i < steps; i++)
      {
	outY1 = inY1 + (i * delY1);
	outY2 = inY2 + (i * delY2);
	gSaveMatch[gLoutIndex] = 0;
	gSaveSweep[gLoutIndex] = 0;
	if (inpBuf2[i] < SENSE_MAX_THRESHOLD)
	  {
	    gSaveMatch[gLoutIndex] = 11;
	    gSaveSweep[gLoutIndex] = sweep;
	    gXsuperSave[gSuperIndex] = inX2;
	    gYsuperSave[gSuperIndex] = outY1;
	    gSuperIndex++;
	  }
	if (inpBuf1[i] < SENSE_MAX_THRESHOLD)
	  {
	    gSaveMatch[gLoutIndex] = 12;
	    gSaveSweep[gLoutIndex] = sweep;
	    gXsuperSave[gSuperIndex] = inX1;
	    gYsuperSave[gSuperIndex] = outY2;
	    gSuperIndex++;
	  }
	if ((inpBuf2[i] < SENSE_MAX_THRESHOLD) && (inpBuf1[i] < SENSE_MAX_THRESHOLD))
	  {
	    gSaveMatch[gLoutIndex] = 1;
	    gSaveSweep[gLoutIndex] = sweep;
	    *sumX += (double)inX1;
	    *sumY += (double)outY1;
	    *sumX += (double)inpX2;
	    *sumY += (double)outY2;
	    *dCount += 2.0;
#ifdef AGS_DEBUG
	    syslog(LOG_DEBUG,"SRCH2SETSY:  SWEEP%d JACKPOT sumX=%f, sumY=%f,i=%d",sweep,*sumX,*sumY,i);
#endif
	  }
	gSaveLout1[gLoutIndex] = inpBuf2[i];
	gSaveLout2[gLoutIndex] = inpBuf1[i];
	gSaveDblX1[gLoutIndex]  = inpX1;
	gSaveDblX2[gLoutIndex]  = inpX1;
	gSaveDblY1[gLoutIndex]  = outY1;
	gSaveDblY2[gLoutIndex]  = outY2;
	gLoutIndex++;
      }
    return(0);
}
static int SearchWithTwoSetsOutX(struct lg_master *pLgMaster, int sweep, double inpX1,double inpY1, int16_t delX1, int16_t delY1,
				 double inpX2, double inpY2, int16_t delX2, int16_t delY2, uint16_t *inpBuf1, uint16_t *inpBuf2,
				 int steps, double *dCount, double *sumX, double *sumY) 
{
    struct lg_xydata  xydata;
    struct lg_xydelta xydelta;
    int               result;
    int16_t           inX1, inX2, inY1, inY2;
    int16_t           outX1, outX2;
    uint16_t          i;

    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    memset((char *)&xydelta, 0, sizeof(struct lg_xydata));
  
    inX1 = (int16_t)inpX1;
    inX2 = (int16_t)inpX2;
    inY1 = (int16_t)inpY1;
    inY2 = (int16_t)inpY2;
    xydata.xdata =  inX1;
    xydata.ydata = inY1;
    xydelta.xdata = delX1;
    xydelta.ydata = delY1;
    result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
			   (struct lg_xydelta *)&xydelta, steps, inpBuf1,0);
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return(kStopWasDone);
    if (result == kStopWasDone)
      return(result);
    xydata.xdata =  inpX2;
    xydata.ydata =  inpY2;
    xydelta.xdata = delX2;
    xydelta.ydata = delY2;
    result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
			   (struct lg_xydelta *)&xydelta, steps, inpBuf2,0);
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return(kStopWasDone);
    if (result == kStopWasDone)
      return(result);
    for (i=0; i < steps; i++)
      {
	outX1 = inX1 + (i * delX1);
	outX2 = inX2 + (i * delX2);
	gSaveMatch[gLoutIndex] = 0;
	gSaveSweep[gLoutIndex] = 0;
	if (inpBuf2[i] < SENSE_MAX_THRESHOLD)
	  {
	    gSaveMatch[gLoutIndex] = 11;
	    gSaveSweep[gLoutIndex] = sweep;
	    gXsuperSave[gSuperIndex] = outX2;
	    gYsuperSave[gSuperIndex] = inY2;
	    gSuperIndex++;
	  }
	    if (inpBuf1[i] < SENSE_MAX_THRESHOLD)
	      {
		gSaveMatch[gLoutIndex] = 12;
		gSaveSweep[gLoutIndex] = sweep;
		gXsuperSave[gSuperIndex] = outX1;
		gYsuperSave[gSuperIndex] = inY1;
		gSuperIndex++;
	      }
	    if ((inpBuf2[i] < SENSE_MAX_THRESHOLD) && (inpBuf1[i] < SENSE_MAX_THRESHOLD))
	      {
		gSaveMatch[gLoutIndex] = 1;
		gSaveSweep[gLoutIndex] = sweep;
		*sumX += (double)outX1;
		*sumY += inpY1;
		*sumX += (double)outX2;
		*sumY += inpY2;
		*dCount += 2.0;
#ifdef AGS_DEBUG
		syslog(LOG_DEBUG,"SRCH2SETSX:  SWEEP%d JACKPOT sumX=%f, sumY=%f,i=%d",sweep,*sumX,*sumY,i);
#endif
	      }
	    gSaveLout1[gLoutIndex] = inpBuf2[i];
	    gSaveLout2[gLoutIndex] = inpBuf1[i];
	    gSaveDblX1[gLoutIndex]  = (double)outX1;
	    gSaveDblX2[gLoutIndex]  = (double)outX2;
	    gSaveDblY1[gLoutIndex]  = inpY1;
	    gSaveDblY2[gLoutIndex]  = inpY2;
	    gLoutIndex++;
      }
    return(0);
}
static int SearchSingleSetOutXY(struct lg_master *pLgMaster, double inpX, double inpY, int16_t delX, int16_t delY, uint16_t *inpBuf, int steps, double *dCount, double *sumX, double *sumY) 
{	      
    struct lg_xydata  xydata;
    struct lg_xydelta xydelta;
    int               result;
    int16_t           outX, outY;
    uint16_t          i;

    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    memset((char *)&xydelta, 0, sizeof(struct lg_xydata));
  
    xydata.xdata =  (int16_t)inpX;
    xydata.ydata =  (int16_t)inpY;
    xydelta.xdata = delX;
    xydelta.ydata = delY;
    result = DoLevelSearch(pLgMaster, &xydata, &xydelta, steps, inpBuf, 0);
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return(kStopWasDone);
    if (result == kStopWasDone)
      return(result);
    for (i=0; i < steps; i++)
      {
	outX  = xydata.xdata + (i * delX);
	outY  = xydata.ydata + (i * delY);
	gSaveMatch[gLoutIndex] = 0;
	gSaveSweep[gLoutIndex] = 0;
	if (inpBuf[i] < SENSE_MAX_THRESHOLD)
	  {
	    *sumX += (double)outX;
	    *sumY += (double)outY;
	    *dCount += 1.0;
	  }
	gSaveLout1[gLoutIndex] = inpBuf[i];
	gSaveLout2[gLoutIndex] = inpBuf[i];
	gSaveDblX1[gLoutIndex]  = (double)outX;
	gSaveDblY1[gLoutIndex]  = (double)outY;
	gSaveDblX2[gLoutIndex]  = (double)outX;
	gSaveDblY2[gLoutIndex]  = (double)outY;
	gLoutIndex++;
      }
    return(0);
}
int SuperSearch(struct lg_master *pLgMaster, int16_t *foundX, int16_t *foundY,
		double *dMinX, double *dMinY, double *dMaxX, double *dMaxY)
{
    double            sumX, sumY, avgX, avgY, Dstep;
    double            dblX, dblY, newX, newY;
    double            Xmid, Ymid, Xlow, Ylow, Xhigh, Yhigh;
    double            dCount, dXSpan, dYSpan;
    int               result, sweep;
    //    int               result, halfIndex, sweep;
    int16_t           delNeg, delPos;
    uint16_t          nXsteps, nYsteps;

    gLoutIndex  = 0;
    gSuperIndex = 0;

    dXSpan = *dMaxX - *dMinX;
    dYSpan = *dMaxY - *dMinY;
#if  0
    nXsteps = (uint16_t)(dXSpan / (double)gSuperFineSearchStep);
    nYsteps = (uint16_t)(dYSpan / (double)gSuperFineSearchStep);
    delNeg = -gSuperFineSearchStep;
    delPos = gSuperFineSearchStep;

#else
    nXsteps = nYsteps = 100;
    delNeg = -gSuperFineSearchStep/2;
    delPos = gSuperFineSearchStep/2;
#endif
    Xmid  = (*dMaxX + *dMinX)/2.0;
    Ymid  = (*dMaxY + *dMinY)/2.0;
    Xlow  = *dMinX - ((*dMaxY - *dMinY)/2.0);
    Xhigh = *dMaxX + ((*dMaxY - *dMinY)/2.0);
    Ylow  = *dMinY - ((*dMaxX - *dMinX)/2.0);
    Yhigh = *dMaxY + ((*dMaxX - *dMinX)/2.0);

#ifdef AGS_DEBUG
    syslog(LOG_DEBUG,"SuperSearch: dXspan=%f,dYspan=%f,steps=%x,delta=%x,xlo=%f,xhi=%f,ylo=%f,yhi=%f",
	   dXSpan,dYSpan,nXsteps,delPos,Xlow,Xhigh,Ylow,Yhigh);
#endif
    sweep = 1;
    sumX = 0.0;
    sumY = 0.0;
    dCount = 0.0;
    Dstep = (double)delPos;
    for (dblX = *dMinX; dblX < *dMaxX; dblX +=  Dstep )
      {
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
	// Sweep 1---Scanset is (dblX, dMaxY, delNeg) and (dblX, dMinY, delPos)
	result = SearchWithTwoSetsOutY(pLgMaster, sweep, dblX, *dMaxY, 0, delNeg, dblX, *dMinY, 0, delPos,
				       gLout2, gLout1, nYsteps, &dCount, &sumX, &sumY);
	if (result)
	  return(result);
	if (dCount >= 1.0)
	    syslog(LOG_NOTICE,"SuperSearch:  Sweep %d dcount %f, #samples %d,delPos=%x",sweep,dCount,nYsteps,delPos);
      }
    if (pLgMaster->gMultipleSweeps >= 1)
      {
	sweep = 2;
	Dstep = (double)delPos;
	for (dblY = *dMinY; dblY < *dMaxY; dblY +=  Dstep)
	  {
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return kStopWasDone;
	    // Sweep 2---Scanset is (dMaxX, dblY, delNeg) and (dMinX, dblY, delPos)
	    result = SearchWithTwoSetsOutX(pLgMaster, sweep, *dMaxX, dblY, delNeg, 0, *dMinX, dblY, delPos, 0,
					   gLout2, gLout1, nXsteps, &dCount, &sumX, &sumY);
	    if (result)
	      return(result);
	    if (dCount >= 1.0)
	      syslog(LOG_NOTICE,"SuperSearch:  Sweep %d dcount %f, #samples %d,delPos=%x",sweep,dCount,nYsteps,delPos);
	  }
      }
    if (pLgMaster->gMultipleSweeps >= 3)
      {
	sweep = 3;
	Dstep = (double)delPos;
	for (dblX = *dMaxX; dblX > *dMinX; dblX -=  Dstep)
	  {
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);
	    // Sweep 3---Scanset is (dblX, dMaxY, delNeg) and (dblX, dMinY, delPos)
	    result = SearchWithTwoSetsOutY(pLgMaster, sweep, dblX, *dMaxY, 0, delNeg, dblX, *dMinY, 0, delPos,
					   gLout2, gLout1, nYsteps, &dCount, &sumX, &sumY);
	    if (result)
	      return(result);
	    if (dCount >= 1.0)
	      syslog(LOG_NOTICE,"SuperSearch:  Sweep %d dcount %f, #samples %d,delPos=%x",sweep,dCount,nYsteps,delPos);
	  }
      }
    if (pLgMaster->gMultipleSweeps >= 4)
      {
	sweep = 4;
	Dstep = (double)delPos;
	for (dblY = *dMaxY; dblY > *dMinY; dblY -=  Dstep)
	  {
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);
	    // Sweep 4---Scanset is (dMaxX, dblY, delNeg) and (dMinX, dblY, delPos)
	    result = SearchWithTwoSetsOutX(pLgMaster, sweep, *dMaxX, dblY, delNeg, 0, *dMinX, dblY, delPos, 0,
					   gLout2, gLout1, nXsteps, &dCount, &sumX, &sumY);
	    if (result)
	      return(result);
	    if (dCount >= 1.0)
	      syslog(LOG_NOTICE,"SuperSearch:  Sweep %d dcount %f, #samples %d,delPos=%x",sweep,dCount,nYsteps,delPos);
	  }
      }
    if (pLgMaster->gMultipleSweeps >= 5)
      {
	sweep = 5;
	//  start of diagonals
	Dstep = (double)delPos;
	for (dblX=Xlow,dblY=Ymid; ((dblX<Xmid) && (dblY<Yhigh)); (dblX+=Dstep), (dblY+=Dstep))
	  {
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return kStopWasDone;
	    // Sweep 5---Scanset is (dbl, dblY, delPos, delNeg)
	    result = SearchSingleSetOutXY(pLgMaster, dblX, dblY, delPos, delNeg, gLout1, nYsteps, &dCount, &sumX, &sumY);
	    if (result)
	      return(result);
	    // Sweep 5---Scanset is (dblX+(delPos * steps), (dblY+(delPos*steps)), delNeg, delPos)
	    newX = dblX + (Dstep * nYsteps);
	    newY = dblY - (Dstep * nYsteps);
	    result = SearchSingleSetOutXY(pLgMaster, newX, newY, delNeg, delPos, gLout1, nYsteps, &dCount, &sumX, &sumY);
	    if (result)
	      return(result);
	    if (dCount >= 1.0)
	      syslog(LOG_NOTICE,"SuperSearch:  Sweep %d dcount %f, #samples %d,delPos=%x",sweep,dCount,nYsteps,delPos);
	  }
      }
  if (pLgMaster->gMultipleSweeps >= 6)
    {
      sweep = 6;
      for (dblX=Xhigh,dblY=Ymid; ((dblX>Xmid) && (dblY<Yhigh)); dblX-=Dstep,dblY-=Dstep)
	{
	  if (IfStopThenStopAndNeg1Else0(pLgMaster))
	    return(kStopWasDone);
	  // Sweep 6---Scanset is (dbl, dblY, delNeg, delPos)
	  result = SearchSingleSetOutXY(pLgMaster, dblX, dblY, delNeg, delPos, gLout1, nYsteps, &dCount, &sumX, &sumY);
	  if (result)
	    return(result);
	  // Sweep 6---Scanset is (dblX-(delPos * steps), (dblY+(delPos*steps)), delPos, delNeg)
	  newX = (int16_t)(dblX - (Dstep * nYsteps));
	  newY = (int16_t)(dblY + (Dstep * nYsteps));
	  result = SearchSingleSetOutXY(pLgMaster, newX, newY, delPos, delNeg, gLout1, nYsteps, &dCount, &sumX, &sumY);
	  if (result)
	    return(result);
	  if (dCount >= 1.0)
	      syslog(LOG_NOTICE,"SuperSearch:  Sweep %d dcount %f, #samples %d,delPos=%x",sweep,dCount,nYsteps,delPos);
        }
    }
  if (pLgMaster->gMultipleSweeps >= 7)
    {
      sweep = 7;
      for (dblX = Xmid, dblY = Yhigh; ((dblX < Xhigh) && (dblY > Ymid)); dblX += Dstep, dblY -= Dstep)
	{
	  if (IfStopThenStopAndNeg1Else0(pLgMaster))
	    return(kStopWasDone);
	  // Sweep 7---Scanset is (dbl, dblY, delNeg, delNeg)
	  result = SearchSingleSetOutXY(pLgMaster, dblX, dblY, delNeg, delNeg, gLout1, nYsteps, &dCount, &sumX, &sumY);
	  if (result)
	    return(result);
	  // Sweep 7---Scanset is (dblX-(delPos * steps), (dblY-(delPos*steps)), delPos, delPos)
	  newX = dblX - (Dstep * nYsteps);
	  newY = dblY - (Dstep * nYsteps);
	  result = SearchSingleSetOutXY(pLgMaster, newX, newY, delPos, delPos, gLout1, nYsteps, &dCount, &sumX, &sumY);
	  if (result)
	    return(result);
	  if (dCount >= 1.0)
	      syslog(LOG_NOTICE,"SuperSearch:  Sweep %d dcount %f, #samples %d,delPos=%x",sweep,dCount,nYsteps,delPos);
        }
    }
  if (pLgMaster->gMultipleSweeps >= 8)
    {
      sweep = 8;
      for (dblX = Xmid, dblY = Ylow; ((dblX >Xlow) && (dblY<Ymid)); dblX -= Dstep, dblY += Dstep)
	{
	  if (IfStopThenStopAndNeg1Else0(pLgMaster))
	    return(kStopWasDone);
	  // Sweep 8---Scanset is (dbl, dblY, delPos, delPos)
	  result = SearchSingleSetOutXY(pLgMaster, dblX, dblY, delPos, delPos, gLout1, nYsteps, &dCount, &sumX, &sumY);
	  if (result)
	    return(result);
	  // Sweep 7---Scanset is (dblX+(delPos * steps), (dblY+(delPos*steps)), delNeg, delNeg)
	  newX =  (int16_t)(dblX + (Dstep * nYsteps));
	  newY =  (int16_t)(dblY + (Dstep * nYsteps));
	  result = SearchSingleSetOutXY(pLgMaster, newX, newY, delNeg, delNeg, gLout1, nYsteps, &dCount, &sumX, &sumY);
	  if (result)
	    return(result);
	  if (dCount >= 1.0)
	      syslog(LOG_NOTICE,"SuperSearch:  Sweep %d dcount %f, #samples %d,delPos=%x",sweep,dCount,nYsteps,delPos);
        }
    }

  if (dCount < 3.0)
    {
      syslog(LOG_NOTICE,"SuperSearch: END: Not enough hits, Sweep %d dcount %f, #samples %d,delNeg=%x,delPos=%x",
	     sweep, dCount,nYsteps,delNeg,delPos);
      return(kSuperFineTooFew);
    }
  
  avgX = sumX / dCount;
  avgY = sumY / dCount;
  *foundX = (int16_t)avgX;
  *foundY = (int16_t)avgY;
#ifdef AGS_DEBUG
  syslog(LOG_DEBUG,"SuperSearch:  MEGAMILLIONS SWEEP %d dCount %f,foundX=%x,foundY=%x",sweep, dCount,*foundX,*foundY);
#endif
  return(0);
}

#if 0
 static int sensor_sort(const void *elem1, const void *elem2)
{
    int16_t numone, numtwo;

    numone = *(const int16_t *)elem1;
    numtwo = *(const int16_t *)elem2;

    if (numone < numtwo)
      return(-1);
    if (numone > numtwo)
      return(1);
    return(0);
}
#endif

void SensorInitLog(void)
{
    syslog(LOG_NOTICE, " triggerlevel %d\n", kDELLEV );
    return;
}

uint32_t   getSuperFineFactor(void)
{
    return(gSuperFineFactor);
}
