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
#define kFineExtend                         15
#define kSuperFineSearchStep                0x1
#define kSuperFineSearchSpanSteps           256
#define kNumberOfCrossesToAverage           7
#define kNumberOfCrosses                    7
#define kNumberOfSuperCrossesToAverage      2
#define kDELLEV                             60
#define kCoarseStepPeriod                   20
#define kFineStepPeriod                     50
#define kQuickStepPeriod                    30


int     gNumberOfSensorSearchAttempts  = 2;
uint32_t gSuperIndex = 0;
int16_t  *gXsuperSave;
int16_t  *gYsuperSave;
uint8_t  *gSaveMatch;
uint8_t  *gSaveSweep;
uint16_t *gSaveLout1;
uint16_t *gSaveLout2;
double   *gSaveAvgX;
double   *gSaveAvgY;
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

void SetFineLevelResults(int16_t firstX,int16_t firstY, int16_t lastX, int16_t lastY,
				int16_t *MinX, int16_t *MinY, int16_t *MaxX, int16_t *MaxY);

//static int sensor_sort( const void *elem1, const void *elem2 );
static int
findFirstLast(struct lg_master *pLgMaster, int16_t *firstX, int16_t *lastX,
	      int16_t *firstY, int16_t *lastY, int16_t *foundX, int16_t *foundY,
	      int16_t currentX, int16_t currentY, int16_t xStep, int16_t yStep,
	      uint16_t nSteps);
static int DoFineLevel(struct lg_master *pLgMaster, int16_t inpX, int16_t inpY,
		       int16_t *foundX, int16_t *foundY, int16_t *outMinX,
		       int16_t *outMinY, int16_t *outMaxX, int16_t *outMaxY);
static int SuperSearch(struct lg_master *pLgMaster, int16_t *foundX,
		       int16_t *foundY, int16_t *MinX, int16_t *MinY,
		       int16_t *MaxX, int16_t * MaxY);
void limitCalc(int16_t centerX, int16_t centerY, int16_t *eolXNeg,
	       int16_t *eolXPos, int16_t *eolYNeg, int16_t *eolYPos, int32_t nSteps);
static int CoarseLeg(struct lg_master *pLgMaster, int16_t Xin, int16_t Yin,
		     int16_t delX, int16_t delY, uint32_t nStepsIn,
		     int16_t *foundX, int16_t *foundY);
static int DoQuickFineSearch(struct lg_master *pLgMaster, int16_t *foundX, int16_t *foundY);
static void AdjustXYLimit(int16_t *eolXPos, int16_t *eolXNeg, int16_t *eolYPos, int16_t *eolYNeg, int16_t delta);
int compare ( const void * a, const void * b );
static void AdjustOneXYSet(int16_t tmpX, int16_t tmpY, int16_t *eolX, int16_t *eolY, int16_t delX, int16_t delY, uint16_t sample);

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
#ifdef DOFINESEARCH
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

           // turn off beam, also centerX/Y should be good and close
        xydata.xdata =  centerX;
        xydata.ydata =  centerY;
        move_dark(pLgMaster, (struct lg_xydata *)&xydata);
        usleep(250);

        return theResult;
}
int SearchForASensor(struct lg_master *pLgMaster, int16_t startX, int16_t startY,
		      int16_t *foundX, int16_t *foundY)
{
    struct lg_xydata   xydata;
    struct lg_xydelta  xydelta;
    int16_t            MinX=0,MinY=0;
    int16_t            MaxX=0, MaxY=0;
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
    int16_t hf2;
    int theResult;
    int16_t DwellHalfSpan;
    int16_t DwellStep;

        
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
            pLgMaster->gSearchType = COARSESEARCH;
	    theResult = DoCoarseScan(pLgMaster, xydata.xdata, xydata.ydata, 0x4,
				     32 * longFactor, &f1x, &f1y);
	    if (theResult == kStopWasDone)
	      return theResult;
	    if (theResult == kCoarseNotFound)
	      continue;
            pLgMaster->gSearchType = COARSESEARCH;
	    theResult = DoCoarseScan2(pLgMaster, xydata.xdata, xydata.ydata,
				      0x4, 32 * longFactor, &f2x, &f2y);
	    if (theResult == kStopWasDone)
	      return theResult;
	    if (theResult == kCoarseNotFound)
	      continue;
	    avgX = f2x;
	    avgY = f1y;
	    theResult = DoFineLevel(pLgMaster, avgX, avgY, foundX, foundY, &MinX, &MinY,
				    &MaxX, &MaxY);
	    if (theResult == kStopWasDone)
	      return theResult;
	    if (theResult == kFineNotFound)
	      continue;
	    theResult = SuperSearch(pLgMaster, foundX, foundY, &MinX, &MinY, &MaxX, &MaxY);
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
	// new_step = ((2*i) + 1) * pLgMaster->gCoarse2SearchStep;
	hf2 = (((i&1)<<1)-1)*(i>>1);
	new_step = hf2 * pLgMaster->gCoarse2SearchStep;
	tempX = startX + new_step;
	tempY = startY + new_step;
	// limitCalc(tempX, tempY,&eolXNeg, &eolXPos, &eolYNeg, &eolYPos, sense_step);
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
         // start the out-going spirals
         // slightly inside the outermost "hatch"
        eolXPos -= 2 * pLgMaster->gCoarse2SearchStep;
        eolYPos -= 2 * pLgMaster->gCoarse2SearchStep;
        eolXNeg += 2 * pLgMaster->gCoarse2SearchStep;
        eolYNeg += 2 * pLgMaster->gCoarse2SearchStep;
	for (i= 0; i < 2 + (pLgMaster->gNumberOfSpirals - (HatchCount/2)); i++)
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
        DwellStep = 2;
        DwellHalfSpan = 64;
        nSteps = (2 * DwellHalfSpan) / DwellStep;

	for (i=0; i<pLgMaster->gDwell; i++)
	  {
	    centX = *foundX;
	    centY = *foundY - DwellHalfSpan;
	    xydata.xdata =  centX;
	    xydata.ydata =  centY;
	    xydelta.xdata = 0;
	    xydelta.ydata = DwellStep;
	    if (DoLineSearch(pLgMaster, (struct lg_xydata *)&xydata,
			     (struct lg_xydelta *)&xydelta, nSteps))
	      {
		return kStopWasDone;
	      }
	    centX = *foundX - DwellHalfSpan;
	    centY = *foundY;
	    xydata.xdata =  centX;
	    xydata.ydata =  centY;
	    xydelta.xdata = DwellStep;
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
    int16_t           MinX, MinY;
    int16_t           MaxX, MaxY;
    int16_t           exMinX, exMinY;
    int16_t           exMaxX, exMaxY;
    int32_t           sumX=0,sumY=0;
    int32_t           i, count=0;
    int32_t           count1, sumX1, sumY1;
    int32_t           count2, sumX2, sumY2;
    int               theResult;
    uint32_t          numStepsFixed = 40;  // for 2nd coarse set, fix number of steps



    // Initialize variables and buffers to be used
    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    memset((char *)&xydelta, 0, sizeof(struct lg_xydata));

    // Start searching
    xydata.xdata =  Xin;
    xydata.ydata =  Yin;
    xydelta.xdata = delX;
    xydelta.ydata = delY;
    pLgMaster->gSearchType = COARSESEARCH;
    theResult = DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
		       (struct lg_xydelta*)&xydelta, nStepsIn, gLout, 1);
    if (theResult)
      return(theResult);

    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return(kStopWasDone);

    count = 0;
    for (i = 0; i < nStepsIn; i++)
      {
	tmpX = Xin + (i * delX);
	tmpY = Yin + (i * delY);
	if (gLout[i] < pLgMaster->gSenseThreshold && gLout[i] > 1)
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
	*foundX = avgX;
	*foundY = avgY;
	//
	// center back-search on found point
        // because of scanner lag,
        // searches need to be done in both directions
        // and will probably not overlap
        //
        // finally, fix the number of steps for this last set
        //
        AdjustOneXYSet(tmpX, tmpY, &eolX, &eolY, delX, delY, numStepsFixed);

           //  search in +delX/+delY direction
	xydata.xdata =  eolX;
	xydata.ydata =  eolY;
	xydelta.xdata = delX;
	xydelta.ydata = delY;
        pLgMaster->gSearchType = COARSESEARCH;
	theResult = DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
				  (struct lg_xydelta*)&xydelta, 2*numStepsFixed, gLout, 1);
	if (theResult == kStopWasDone)
	  return(theResult);

	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
                 
	tmpX = eolX;
	tmpY = eolY;
	sumX1 = 0;
	sumY1 = 0;
	count1 = 0;
	for (i=0; i < 2*numStepsFixed; i++)
	  {
	    tmpX += delX;
	    tmpY += delY; 
	    if (gLout[i] < pLgMaster->gSenseThreshold && gLout[i] > 1)
	      {
		sumX1 += tmpX;
		sumY1 += tmpY;
		count1++;
	      }
	  }

           //  now search in -delX/-delY direction
	xydata.xdata =  eolX + 2*numStepsFixed * delX;
	xydata.ydata =  eolY + 2*numStepsFixed * delY;
	xydelta.xdata = -delX;
	xydelta.ydata = -delY;
        pLgMaster->gSearchType = COARSESEARCH;
	theResult = DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
				  (struct lg_xydelta*)&xydelta, 2*numStepsFixed, gLout, 1);
	if (theResult == kStopWasDone)
	  return(theResult);

	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
                 
	tmpX = eolX + 2*numStepsFixed * delX;
	tmpY = eolY + 2*numStepsFixed * delY;
	sumX2 = 0;
	sumY2 = 0;
	count2 = 0;
	for (i=0; i < 2*numStepsFixed; i++)
	  {
	    tmpX -= delX;
	    tmpY -= delY; 
	    if (gLout[i] < pLgMaster->gSenseThreshold && gLout[i] > 1)
	      {
		sumX2 += tmpX;
		sumY2 += tmpY;
		count2++;
	      }
	  }

	if (count1 >= 1 && count2 >= 1)
	  {
	    // Coarse search found target approximate location.
            //  Finer search will hone on more-specific location
            //  for debugging, the two sets of sums were separated
            //
	    avgX = (int16_t)((sumX1+sumX2)/(count1+count2));
	    avgY = (int16_t)((sumY1+sumY2)/(count1+count2));
	    syslog(LOG_NOTICE,"CoarseLeg Found %d points, avgX=%x,avgY=%x.  Move on to DoFineLevel.",count,avgX,avgY);
	    theResult = DoFineLevel(pLgMaster, avgX, avgY, foundX, foundY, &MinX, &MinY,
				    &MaxX, &MaxY);
	    if (theResult == kStopWasDone)
	      return(theResult);

	    // Coarse & Fine searches found target where-abouts.  Now time to get more exact location
	    *foundX = avgX;
	    *foundY = avgY;
	    syslog(LOG_NOTICE,"CoarseLeg->SuperSearch: DoFineLevel Found target at x=%x,y=%x,rc=%x",avgX,avgY,theResult);
            // extend search borders
            exMinX = MinX;
            exMaxX = MaxX;
            exMinY = MinY;
            exMaxY = MaxY;
            SetFineLevelResults( MinX, MinY, MaxX, MaxY
                      , &exMinX, &exMinY, &exMaxX, &exMaxY);
	    theResult = SuperSearch(pLgMaster, foundX, foundY, &exMinX, &exMinY, &exMaxX, &exMaxY);
	    if (theResult == 0)
	      {
		syslog(LOG_NOTICE,"SuperSearch:  Found target at x=%x,y=%x",*foundX, *foundY);
		return(0);
	      }
	    if (theResult == kStopWasDone)
	      return(theResult);
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
    int16_t          QuickFineStep;
        
    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    memset((char *)&xydelta, 0, sizeof(struct lg_xydata));
    theSpan = (kSuperFineSearchStep * kSuperFineSearchSpanSteps) / 2;

    QuickFineStep = 2 * kSuperFineSearchStep;

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

	delX = 2*QuickFineStep;
	nSteps = theSpan / QuickFineStep;
	xydata.xdata =  currentX;
	xydata.ydata =  currentY;
	xydelta.xdata = delX;
	xydelta.ydata = 0;
        pLgMaster->gSearchType = FINESEARCH;
	rc = DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
		      (struct lg_xydelta*)&xydelta, nSteps, gLout,1);
	if (rc == kStopWasDone)
	  return(rc);
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);

	tmpX = currentX;
	if (j > 1 && (rc == 0))
	  {
	    for (index=0; index<nSteps; index++)
	      {
		if (gLout[index] < pLgMaster->gSenseThreshold)
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
	delX = -2 * QuickFineStep;
	nSteps = theSpan / QuickFineStep;
	xydata.xdata =  currentX;
	xydata.ydata =  currentY;
	xydelta.xdata = delX;
	xydelta.ydata = 0;
        pLgMaster->gSearchType = FINESEARCH;
	rc = DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
		      (struct lg_xydelta*)&xydelta, nSteps, gLout,1);
	if (rc == kStopWasDone)
	  return(rc);
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
	tmpX = currentX;
	if (j > 1 && (rc == 0))
	  {
	    for (index=0; index < nSteps; index++)
	      {
		if (gLout[index] < pLgMaster->gSenseThreshold)
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
	delY = 2*QuickFineStep;
	nSteps = theSpan / QuickFineStep;
	xydata.xdata =  currentX;
	xydata.ydata =  currentY;
	xydelta.ydata = delY;
	xydelta.xdata = 0;
        pLgMaster->gSearchType = FINESEARCH;
	rc = DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
		      (struct lg_xydelta*)&xydelta, nSteps, gLout,1);
	if (rc == kStopWasDone)
	  return(rc);
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	    return(kStopWasDone);
	tmpY = currentY;
	if (j > 1 && (rc == 0))
	  {
	    for (index = 0; index<nSteps; index++)
	      {
		if (gLout[index] < pLgMaster->gSenseThreshold)
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
	delY = -2 * QuickFineStep;
	nSteps = theSpan / QuickFineStep;
	xydata.xdata =  currentX;
	xydata.ydata =  currentY;
	xydelta.ydata = delY;
	xydelta.xdata = 0;
        pLgMaster->gSearchType = FINESEARCH;
	rc = DoLevelSearch(pLgMaster, (struct lg_xydata*)&xydata,
		      (struct lg_xydelta*)&xydelta, nSteps, gLout, 1);
	if (rc == kStopWasDone)
	  return(rc);
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
	tmpY = currentY;
	if (j > 1 && (rc == 0))
	  {
	    for (index=0; index < nSteps; index++)
		  {
		    if (gLout[index] < pLgMaster->gSenseThreshold)
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
    gSaveAvgX = (double *)calloc((size_t)500000,sizeof(double)); 
    gSaveAvgY = (double *)calloc((size_t)500000,sizeof(double)); 
    gXsuperSave = (int16_t *)calloc((size_t)500000,sizeof(double)); 
    gYsuperSave = (int16_t *)calloc((size_t)500000,sizeof(double)); 
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
      memset((char *)gXsuperSave, 0, SENSE_BUF_SIZE);
    if (gYsuperSave)
      memset((char *)gYsuperSave, 0, SENSE_BUF_SIZE);
    if (gSaveLout1)
      memset((char *)gSaveLout1, 0, (500000 * sizeof(uint16_t)));
    if (gSaveLout2)
      memset((char *)gSaveLout2, 0, (500000 * sizeof(uint16_t)));
    if (gSaveAvgX)
      memset((char *)gSaveAvgX, 0, (500000 * sizeof(double)));
    if (gSaveAvgY)
      memset((char *)gSaveAvgY, 0, (500000 * sizeof(double)));
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
    if (gSaveAvgX) free(gSaveAvgX);
    if (gSaveAvgY) free(gSaveAvgY);
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

void SetFineLevelResults(int16_t firstX,int16_t firstY, int16_t lastX, int16_t lastY,
				int16_t *MinX, int16_t *MinY, int16_t *MaxX, int16_t *MaxY)
{
    int32_t  itest;

    if (firstX < *MinX)
      *MinX = firstX;
    if (firstY < *MinY)
      *MinY = firstY;
    if (firstX > *MaxX)
      *MaxX = firstX;
    if (firstY > *MaxY)
      *MaxY = firstY;
    if (lastX < *MinX)
      *MinX = lastX;
    if (lastY < *MinY)
      *MinY = lastY;
    if (lastX > *MaxX)
      *MaxX = lastX;
    if (lastY > *MaxY)
      *MaxY = lastY;
//  because of position lag,
//  the minimums and maximums need
//  to be extended
  
    itest = (int32_t)(*MinX) - kFineExtend;
    if (itest  <= kMinSigned)
    {
      *MinX  = kMinSigned;
    } else {
      *MinX -= kFineExtend;
    }

    itest = (int32_t)(*MinY) - kFineExtend;
    if (itest  <= kMinSigned)
    {
      *MinY  = kMinSigned;
    } else {
      *MinY -= kFineExtend;
    }

    itest = (int32_t)(*MaxX) + kFineExtend;
    if (itest  >= kMaxSigned)
    {
      *MaxX  = kMaxSigned;
    } else {
      *MaxX += kFineExtend;
    }

    itest = (int32_t)(*MaxY) + kFineExtend;
    if (itest  >= kMaxSigned)
    {
      *MaxY  = kMaxSigned;
    } else {
      *MaxY += kFineExtend;
    }

      return;
}
static int DoFineLevel(struct lg_master *pLgMaster, int16_t inpX, int16_t inpY,
		       int16_t *foundX, int16_t *foundY, int16_t *outMinX,
		       int16_t *outMinY, int16_t *outMaxX, int16_t *outMaxY)
{
    int16_t MinX, MinY, MaxX, MaxY; 
    int    result;
    int32_t stepSize, negStep, nSteps;
    int16_t eolXPos=0, eolYPos=0, eolXNeg=0, eolYNeg=0;
    int16_t currentX, currentY, centerX, centerY;
    int16_t firstX, firstY, lastX, lastY;
    int16_t Xm, Ym;
        
    nSteps = 140;
    // Seed with starting input XY pair
    centerX = inpX;
    centerY = inpY;
    MinX   = inpX;
    MaxX   = inpX;
    MinY   = inpY;
    MaxY   = inpY;

    stepSize = kFineSearchStep;
    negStep  = -kFineSearchStep;
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return kStopWasDone;
	
    // Start with NegX/NegY pair
    // starting Y, NegX
    // also try to find dot size

    limitCalc(centerX, centerY, &eolXNeg, &eolXPos, &eolYNeg, &eolYPos, stepSize * (nSteps/2));
    currentX = eolXNeg;
    currentY = centerY;
    result =  findFirstLast(pLgMaster, &firstX, &firstY, &lastX, &lastY, &Xm, &Ym, currentX,
			    currentY, stepSize, 0, nSteps);
    if (result == kStopWasDone)
         return(result);


    // Set up for next round, look for min/max of current XY pair
    SetFineLevelResults(firstX, firstY, lastX, lastY, &MinX, &MinY, &MaxX, &MaxY);
    // Starting X, NegY
    currentX = centerX;
    currentY = eolYNeg;
    result =  findFirstLast(pLgMaster, &firstX, &firstY, &lastX, &lastY, &Xm, &Ym, currentX, currentY,
			    0, stepSize, nSteps);
    if (result == kStopWasDone)
      return(result);
    // Set up for next round, look for min/max of current XY pair
    SetFineLevelResults(firstX, firstY, lastX, lastY, &MinX, &MinY, &MaxX, &MaxY);
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return kStopWasDone;

    //  End of original XY pair, on to next set.
    centerX = Xm;
    centerY = Ym;
    limitCalc(centerX, centerY, &eolXNeg, &eolXPos, &eolYNeg, &eolYPos, stepSize * (nSteps/2));
    // ctrX, NegY
    currentX = centerX;
    currentY = eolYNeg;
    result =  findFirstLast(pLgMaster, &firstX, &firstY, &lastX, &lastY,
			      &Xm, &Ym, currentX, currentY, 0, stepSize,
			      nSteps);
    if (result == kStopWasDone)
      return(result);
    // Set up for next round, look for min/max of current XY pair
    SetFineLevelResults(firstX, firstY, lastX, lastY, &MinX, &MinY, &MaxX, &MaxY);
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return kStopWasDone;

    centerX = Xm;
    centerY = Ym;
    limitCalc(centerX, centerY, &eolXNeg, &eolXPos, &eolYNeg, &eolYPos, stepSize * (nSteps/2));
    // PosX, Ctr Y
    currentX = eolXPos;
    currentY = centerY;
    result =  findFirstLast(pLgMaster, &firstX, &firstY, &lastX, &lastY, &Xm,
			    &Ym, currentX, currentY, negStep, 0, nSteps);
    if (result == kStopWasDone)
      return(result);
    // Set up for next round, look for min/max of current XY pair
    SetFineLevelResults(firstX, firstY, lastX, lastY, &MinX, &MinY, &MaxX, &MaxY);
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return kStopWasDone;

    // Use new center XY pair
    centerX = Xm;
    centerY = Ym;
    limitCalc(centerX, centerY, &eolXNeg, &eolXPos, &eolYNeg, &eolYPos, stepSize * (nSteps/2));
    // NegX, CtrY
    currentX = eolXNeg;
    currentY = centerY;
    result =  findFirstLast(pLgMaster, &firstX, &firstY, &lastX, &lastY, &Xm,
			    &Ym, currentX, currentY, stepSize, 0, nSteps);
    if (result == kStopWasDone)
      return(result);
    // Set up for next round, look for min/max of current XY pair
    SetFineLevelResults(firstX, firstY, lastX, lastY, &MinX, &MinY, &MaxX, &MaxY);
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return kStopWasDone;

    // Use new center XY pair
    centerX = Xm;
    centerY = Ym;
    limitCalc(centerX, centerY, &eolXNeg, &eolXPos, &eolYNeg, &eolYPos, stepSize * nSteps / 2);
    // PosY, CtrX
    currentX = centerX;
    currentY = eolYPos;
    result =  findFirstLast(pLgMaster, &firstX, &firstY, &lastX, &lastY, &Xm,
			    &Ym, currentX, currentY, 0, negStep, nSteps);
    if (result == kStopWasDone)
      return(result);
    // Last set of min/max calculations
    SetFineLevelResults(firstX, firstY, lastX, lastY, &MinX, &MinY, &MaxX, &MaxY);
    centerX = Xm;
    centerY = Ym;
    *outMinX = MinX;
    *outMinY = MinY;
    *outMaxX = MaxX;
    *outMaxY = MaxY;
    *foundX = centerX;
    *foundY = centerY;
    syslog(LOG_NOTICE,"DoFineLevel: Found x=%x,y=%x,minX=%x,maxX=%x,minY=%x,maxY=%x",
	   *foundX,*foundY,*outMinX,*outMaxX,*outMinY,*outMaxY);
    return(0);
}

static int findFirstLast(struct lg_master *pLgMaster, int16_t *firstX, int16_t *firstY,
			 int16_t *lastX, int16_t *lastY, int16_t *foundX,
			 int16_t *foundY, int16_t currentX, int16_t currentY,
			 int16_t xStep, int16_t yStep, uint16_t nSteps)
{
	struct lg_xydata  xydata;
	struct lg_xydelta xydelta;
	int32_t           sumX=0, sumY=0;
	int32_t           count;
	int               i, rc;
        int16_t           tmpX, tmpY;
        int               firstFlag = 1;

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
        pLgMaster->gSearchType = FINESEARCH;
	rc = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
			   (struct lg_xydelta *)&xydelta, nSteps, gLout,1);
	if (rc)
	  return(rc);
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;

        // test for background level near start and finish

	sumX = 0;
	sumY = 0;
	count = 0;
        for (i=0; i < nSteps; i++)
	  {
	    tmpX = currentX + (i * xStep);
	    tmpY = currentY + (i * yStep);
	    if (gLout[i] < pLgMaster->gSenseThreshold && gLout[i] > 1 )
	      {
                if ( firstFlag == 1 )
                  {
		    *firstX = tmpX;
		    *firstY = tmpY;
                  }
		*lastX = tmpX;
		*lastY = tmpY;
		sumX += tmpX;
		sumY += tmpY;
		count++;
	      }
	  }
        if (count == 0)
	  return(kFineNotFound);
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;

          // reject searches with signal at end or beginning
        if ( gLout[1] < pLgMaster->gSenseThreshold ||
             gLout[nSteps-2] < pLgMaster->gSenseThreshold ) {
          return(kFineNotFound);
        }

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
      *eolXNeg = centerX - (nSteps);
      *eolXPos = centerX + (nSteps);
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
      *eolYNeg = centerY - (nSteps);
      *eolYPos = centerY + (nSteps);
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
static void AdjustOneXYSet(int16_t tmpX, int16_t tmpY, int16_t *eolX, int16_t *eolY, int16_t delX, int16_t delY, uint16_t sample)
{
  if ((tmpX - (sample * delX)) <= kMinSigned)
    *eolX = kMinSigned;
  else
    *eolX = tmpX - (sample * delX);
  if ((tmpY - (sample * delY)) <= kMinSigned)
    *eolY = kMinSigned;
  else
    *eolY = tmpY - (sample * delY);
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

static int SearchWithTwoSetsOutY( struct lg_master *pLgMaster
                                , int sweep
                                , int16_t inpX1
                                , int16_t inpY1
                                , int16_t delX1
                                , int16_t delY1
                                , int16_t inpX2
                                , int16_t inpY2
                                , int16_t delX2
                                , int16_t delY2
                                , uint16_t *inpBuf1
                                , uint16_t *inpBuf2
                                , int steps
                                , int32_t *countX
                                , int32_t *countY
                                , int32_t *sumX
                                , int32_t *sumY
                                ) 
{
    struct lg_xydata  xydata;
    struct lg_xydelta xydelta;
    int               result;
    uint32_t          hit_count=0;
    int16_t           outY1, outY2;
    uint16_t          i;

    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    memset((char *)&xydelta, 0, sizeof(struct lg_xydata));

    xydata.xdata =  inpX1;
    xydata.ydata =  inpY1;
    xydelta.xdata = delX1;
    xydelta.ydata = delY1;
    pLgMaster->gSearchType = SUPERSEARCH;
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
    pLgMaster->gSearchType = SUPERSEARCH;
    result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
		   (struct lg_xydelta *)&xydelta, steps, inpBuf2,0);
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return(kStopWasDone);
    if (result == kStopWasDone)
      return(result);
    for (i=0; i < steps; i++)
      {
	outY1 = inpY1 + (i * delY1);
	outY2 = inpY2 + (i * delY2);
	gSaveMatch[gLoutIndex] = 0;
	gSaveSweep[gLoutIndex] = 0;
	if (inpBuf2[i] < pLgMaster->gSenseThreshold)
	  {
	    gSaveMatch[gLoutIndex] = 11;
	    gSaveSweep[gLoutIndex] = sweep;
	    gXsuperSave[gSuperIndex] = inpX2;
	    gYsuperSave[gSuperIndex] = outY2;
	    *sumX += inpX2;
	    *sumY += outY2;
	    hit_count += 1;
	    gSuperIndex++;
	  }
	if (inpBuf1[i] < pLgMaster->gSenseThreshold)
	  {
	    gSaveMatch[gLoutIndex] = 12;
	    gSaveSweep[gLoutIndex] = sweep;
	    gXsuperSave[gSuperIndex] = inpX1;
	    gYsuperSave[gSuperIndex] = outY1;
	    *sumX += inpX1;
	    *sumY += outY1;
	    hit_count += 1;
	    gSuperIndex++;
  	}
	if ((inpBuf2[i] < pLgMaster->gSenseThreshold) && (inpBuf1[i] < pLgMaster->gSenseThreshold))
	  {
	    gSaveMatch[gLoutIndex] = 1;
	    gSaveSweep[gLoutIndex] = sweep;
	  }
	gSaveLout1[gLoutIndex] = inpBuf2[i];
	gSaveLout2[gLoutIndex] = inpBuf1[i];
	gSaveAvgX[gLoutIndex]  = (inpX1 + inpX2) / 2;
	gSaveAvgY[gLoutIndex]  = (outY1 + outY2) / 2;
	gLoutIndex++;
      }
    *countX += hit_count;
    *countY += hit_count;
    return(0);
}
static int SearchWithTwoSetsOutX( struct lg_master *pLgMaster
                                , int sweep
                                , int16_t inpX1
                                , int16_t inpY1
                                , int16_t delX1
                                , int16_t delY1
                                , int16_t inpX2
                                , int16_t inpY2
                                , int16_t delX2
                                , int16_t delY2
                                , uint16_t *inpBuf1
                                , uint16_t *inpBuf2
                                , int steps
                                , int32_t *countX
                                , int32_t *countY
                                , int32_t *sumX
                                , int32_t *sumY
                                ) 
{
    struct lg_xydata  xydata;
    struct lg_xydelta xydelta;
    uint32_t          hit_count=0;
    int               result;
    int16_t           outX1, outX2;
    uint16_t          i;

    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    memset((char *)&xydelta, 0, sizeof(struct lg_xydata));
  
    xydata.xdata =  inpX1;
    xydata.ydata =  inpY1;
    xydelta.xdata = delX1;
    xydelta.ydata = delY1;
    pLgMaster->gSearchType = SUPERSEARCH;
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
    pLgMaster->gSearchType = SUPERSEARCH;
    result = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
		   (struct lg_xydelta *)&xydelta, steps, inpBuf2,0);
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return(kStopWasDone);
    if (result == kStopWasDone)
      return(result);
    for (i=0; i < steps; i++)
      {
	outX1 = inpX1 + (i * delX1);
	outX2 = inpX2 + (i * delX2);
	gSaveMatch[gLoutIndex] = 0;
	gSaveSweep[gLoutIndex] = 0;
    	if (inpBuf2[i] < pLgMaster->gSenseThreshold)
	      {
        	gSaveMatch[gLoutIndex] = 11;
	        gSaveSweep[gLoutIndex] = sweep;
	        gXsuperSave[gSuperIndex] = outX2;
	        gYsuperSave[gSuperIndex] = inpY2;
		*sumX += outX2;
		*sumY += inpY2;
		hit_count += 1;
	        gSuperIndex++;
	     }
	if (inpBuf1[i] < pLgMaster->gSenseThreshold)
	     {
		gSaveMatch[gLoutIndex] = 12;
		gSaveSweep[gLoutIndex] = sweep;
		gXsuperSave[gSuperIndex] = outX1;
		gYsuperSave[gSuperIndex] = inpY1;
		*sumX += outX1;
		*sumY += inpY1;
		hit_count += 1;
		gSuperIndex++;
	     }
        if ((inpBuf2[i] < pLgMaster->gSenseThreshold) && (inpBuf1[i] < pLgMaster->gSenseThreshold))
             {
		gSaveMatch[gLoutIndex] = 1;
		gSaveSweep[gLoutIndex] = sweep;
	     }
	gSaveLout1[gLoutIndex] = inpBuf2[i];
        gSaveLout2[gLoutIndex] = inpBuf1[i];
        gSaveAvgX[gLoutIndex]  = (inpY1 + inpY2) / 2;
        gSaveAvgY[gLoutIndex]  = (outX1 + outX2) / 2;
        gLoutIndex++;
      }
    *countX += hit_count;
    *countY += hit_count;
    return(0);
}
static int SearchSingleSetOutXY( struct lg_master *pLgMaster
                               , int16_t inpX
                               , int16_t inpY
                               , int16_t delX
                               , int16_t delY
                               , uint16_t *inpBuf
                               , int steps
                               , int32_t *countX
                               , int32_t *countY
                               , int32_t *sumX
                               , int32_t *sumY
                               ) 
{	      
    struct lg_xydata  xydata;
    struct lg_xydelta xydelta;
    uint32_t          hit_count=0;
    int               result;
    int16_t           outX, outY;
    uint16_t          i;

    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    memset((char *)&xydelta, 0, sizeof(struct lg_xydata));
  
    xydata.xdata =  inpX;
    xydata.ydata =  inpY;
    xydelta.xdata = delX;
    xydelta.ydata = delY;
    pLgMaster->gSearchType = SUPERSEARCH;
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
	if (inpBuf[i] < pLgMaster->gSenseThreshold)
	  {
	    *sumX += outX;
	    *sumY += outY;
	    hit_count++;
	  }
      }
    *countX += hit_count;
    *countY += hit_count;
    return(0);
}
int SuperSearch(struct lg_master *pLgMaster, int16_t *foundX, int16_t *foundY,
	int16_t *MinX, int16_t *MinY, int16_t *MaxX, int16_t *MaxY)
{
    int               result, sweep;
    int32_t           sumX, sumY;
    int32_t           countX, countY;
    int32_t           dummyCount, dummySum;
    int32_t           Dstep, XSpan, YSpan;
    int16_t           avgX, avgY;
    int16_t           currentX, currentY, newX, newY;
    int16_t           Xmid, Ymid, Xlow, Ylow, Xhigh, Yhigh;
    int16_t           delNeg, delPos;
    uint16_t          nXsteps, nYsteps;

    // Initialize buffers for new sense
    gLoutIndex  = 0;
    gSuperIndex = 0;
    memset((char *)gXsuperSave, 0, SENSE_BUF_SIZE);
    memset((char *)gYsuperSave, 0, SENSE_BUF_SIZE);
    memset((char *)gSaveAvgX, 0, SENSE_BUF_SIZE);
    memset((char *)gSaveAvgY, 0, SENSE_BUF_SIZE);

    // extend the edges of search  

    XSpan = *MaxX - *MinX;
    YSpan = *MaxY - *MinY;
#if 1
    nXsteps = (uint16_t)((double)XSpan / (double)gSuperFineSearchStep);
    nYsteps = (uint16_t)((double)YSpan / (double)gSuperFineSearchStep);
    delNeg = -gSuperFineSearchStep;
    delPos = gSuperFineSearchStep;
#else
    // Sensing hardware needs a minimum number of points to get sensing data
    nXsteps = nYsteps = 50;
    delNeg = -gSuperFineSearchStep;
    delPos = gSuperFineSearchStep;
#endif
    Xmid  = (*MaxX + *MinX)/2;
    Ymid  = (*MaxY + *MinY)/2;
    Xlow  = *MinX - ((*MaxY - *MinY)/2);
    Xhigh = *MaxX + ((*MaxY - *MinY)/2);
    Ylow  = *MinY - ((*MaxX - *MinX)/2);
    Yhigh = *MaxY + ((*MaxX - *MinX)/2);

    sweep = 1;
    sumX = 0;
    sumY = 0;
    countX = 0;
    countY = 0;
    Dstep = delPos;
    dummySum = 0; dummyCount = 0;
    for (currentX = *MinX; currentX < *MaxX; currentX +=  Dstep )
      {
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
	// Sweep 1---Scanset is (currentX, currentY, delNeg) and (currentX, MinY, delPos)
	//   since Y is moving fast, use this to determine X position
        result = SearchWithTwoSetsOutY(pLgMaster
                              , sweep
                              , currentX
                              , *MaxY
                              , 0
                              , delNeg
                              , currentX
                              , *MinY
                              , 0
                              , delPos
                              , gLout2
                              , gLout1
                              , nYsteps
                              , &countX
                              , &dummyCount
                              , &sumX
                              , &dummySum
                              );
	if (result)
	  return(result);
	if (countX >= 1)
	    syslog(LOG_NOTICE,"SuperSearch:  Sweep %d, hit countX %d, #samples %d,delPos=%x",sweep,countX,nYsteps,delPos);
      }
    sweep = 2;
    Dstep = delPos;
    dummySum = 0; dummyCount = 0;
    for (currentY = *MinY; currentY < *MaxY; currentY +=  Dstep)
      {
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
          return kStopWasDone;
          
        // Sweep 2---Scanset is (MaxX, currentY, delNeg) and (MinX, currentY, delPos)
        //   since X is moving fast, use this to determine Y position
        result = SearchWithTwoSetsOutX( pLgMaster
                                  , sweep
                                  , *MaxX
                                  , currentY
                                  , delNeg
                                  , 0
                                  , *MinX
                                  , currentY
                                  , delPos
                                  , 0
                                  , gLout2
                                  , gLout1
                                  , nXsteps
                                  , &dummyCount
                                  , &countY
                                  , &dummySum
                                  , &sumY
                                  );
        if (result)
          return(result);
        if (countX >= 1 && countY >= 1)
          syslog(LOG_NOTICE,"SuperSearch:  Sweep %d hit count %d %d, #samples %d,delPos=%x",sweep,countX,countY,nYsteps,delPos);
      }
    if ((countX < 3) && (gSuperIndex == 0))
      {
        syslog(LOG_NOTICE,"SuperSearch: END: Not enough hits, Sweep %d, hit countX %d, #samples %d,delNeg=%x,delPos=%x",
            sweep, countX ,nYsteps,delNeg,delPos);
        return(kSuperFineTooFew);
      }
    if ((countY < 3) && (gSuperIndex == 0))
      {
        syslog(LOG_NOTICE,"SuperSearch: END: Not enough hits, Sweep %d, hit countY %d, #samples %d,delNeg=%x,delPos=%x",
           sweep, countY ,nYsteps,delNeg,delPos);
        return(kSuperFineTooFew);
      }
    if (pLgMaster->gMultipleSweeps >= 3)
      {
	sweep = 3;
	Dstep = delPos;
	for (currentX = *MaxX; currentX > *MinX; currentX -=  Dstep)
	  {
    	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);
	    // Sweep 3---Scanset is (currentX, MaxY, delNeg) and (currentX, MinY, delPos)
            result = SearchWithTwoSetsOutY(pLgMaster
                                  , sweep
                                  , currentX
                                  , *MaxY
                                  , 0
                                  , delNeg
                                  , currentX
                                  , *MinY
                                  , 0
                                  , delPos
                                  , gLout2
                                  , gLout1
                                  , nYsteps
                                  , &countX
                                  , &countY
                                  , &sumX
                                  , &sumY
                                  );
            if (result)
              return(result);
            if (countX >= 1 && countY >= 1)
              syslog(LOG_NOTICE,"SuperSearch:  Sweep %d, hit count %d %d, #samples %d,delPos=%x",sweep,countX,countY,nYsteps,delPos);
	  }
      }
    if (pLgMaster->gMultipleSweeps >= 4)
      {
	sweep = 4;
	Dstep = delPos;
	for (currentY = *MaxY; currentY > *MinY; currentY -=  Dstep)
	  {
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);
	    // Sweep 4---Scanset is (MaxX, currentY, delNeg) and (MinX, currentY, delPos)
	    result = SearchWithTwoSetsOutX(pLgMaster
                                  , sweep
                                  , *MaxX
                                  , currentY
                                  , delNeg
                                  , 0
                                  , *MinX
                                  , currentY
                                  , delPos
                                  , 0
                                  , gLout2
                                  , gLout1
                                  , nXsteps
                                  , &countX
                                  , &countY
                                  , &sumX
                                  , &sumY
                                  );
	    if (result)
	      return(result);
	    if (countX >= 1 && countY >= 1)
	      syslog(LOG_NOTICE,"SuperSearch:  Sweep %d, count %d %d, #samples %d,delPos=%x",sweep,countX,countY,nYsteps,delPos);
	  }
      }
    if (pLgMaster->gMultipleSweeps >= 5)
      {
	sweep = 5;
	//  start of diagonals
	Dstep = delPos;
	for (currentX=Xlow,currentY=Ymid; ((currentX<Xmid) && (currentY<Yhigh)); (currentX+=Dstep), (currentY+=Dstep))
	  {
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return kStopWasDone;
	    // Sweep 5---Scanset is (dbl, dblY, delPos, delNeg)
	    result = SearchSingleSetOutXY(pLgMaster
                                 , currentX
                                 , currentY
                                 , delPos
                                 , delNeg
                                 , gLout1
                                 , nYsteps
                                 , &countX
                                 , &countY
                                 , &sumX
                                 , &sumY
                                 );
	    if (result)
	      return(result);

	    // Sweep 5---Scanset is (dblX+(delPos * steps), (dblY+(delPos*steps)), delNeg, delPos)
	    newX = currentX + (Dstep * nYsteps);
	    newY = currentY - (Dstep * nYsteps);
	    result = SearchSingleSetOutXY( pLgMaster
                                 , newX
                                 , newY
                                 , delNeg
                                 , delPos
                                 , gLout1
                                 , nYsteps
                                 , &countX
                                 , &countY
                                 , &sumX
                                 , &sumY
                                 );
	    if (result)
	      return(result);
	    if (countX >= 1 && countY >= 1)
	      syslog(LOG_NOTICE,"SuperSearch:  Sweep %d, hit count %d %d, #samples %d,delPos=%x",sweep,countX,countY,nYsteps,delPos);
	  }
      }
    if (pLgMaster->gMultipleSweeps >= 6)
      {
        sweep = 6;
        for (currentX=Xhigh,currentY=Ymid; ((currentX>Xmid) && (currentY<Yhigh)); currentX-=Dstep,currentY-=Dstep)
	  {
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);

	    // Sweep 6---Scanset is (currentX, currentY, delNeg, delPos)
	    result = SearchSingleSetOutXY (pLgMaster
                               , currentX
                               , currentY
                               , delNeg
                               , delPos
                               , gLout1
                               , nYsteps
                               , &countX
                               , &countY
                               , &sumX
                               , &sumY
                               );
	    if (result)
	      return(result);
	    // Sweep 6---Scanset is (currentX-(delPos * steps), (currentY+(delPos*steps)), delPos, delNeg)
	    newX = currentX - (Dstep * nYsteps);
	    newY = currentY + (Dstep * nYsteps);
	    result = SearchSingleSetOutXY( pLgMaster
                               , newX
                               , newY
                               , delPos
                               , delNeg
                               , gLout1
                               , nYsteps
                               , &countX
                               , &countY
                               , &sumX
                               , &sumY
                               );
	    if (result)
	      return(result);
	    if (countX >= 1 && countY >= 1)
	        syslog(LOG_NOTICE,"SuperSearch:  Sweep %d, hit count %d %d, #samples %d,delPos=%x",sweep,countX,countY,nYsteps,delPos);
          }
      }
    if (pLgMaster->gMultipleSweeps >= 7)
      {
        sweep = 7;
        for (currentX = Xmid, currentY = Yhigh; ((currentX < Xhigh) && (currentY > Ymid)); currentX += Dstep, currentY -= Dstep)
	  {
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);
	    // Sweep 7---Scanset is (dbl, dblY, delNeg, delNeg)
	    result = SearchSingleSetOutXY( pLgMaster
                               , currentX
                               , currentY
                               , delNeg
                               , delNeg
                               , gLout1
                               , nYsteps
                               , &countX
                               , &countY
                               , &sumX
                               , &sumY
                               );
	    if (result)
	      return(result);
	    // Sweep 7---Scanset is (currentX-(delPos * steps), (currentY-(delPos*steps)), delPos, delPos)
	    newX = currentX - (Dstep * nYsteps);
	    newY = currentY - (Dstep * nYsteps);
	    result = SearchSingleSetOutXY( pLgMaster
                               , newX
                               , newY
                               , delPos
                               , delPos
                               , gLout1
                               , nYsteps
                               , &countX
                               , &countY
                               , &sumX
                               , &sumY
                               );
	    if (result)
	      return(result);
	    if (countX >= 1 && countY >= 1)
	        syslog(LOG_NOTICE,"SuperSearch:  Sweep %d, hit count %d %d, #samples %d,delPos=%x",sweep,countX,countY,nYsteps,delPos);
          }
      }
    if (pLgMaster->gMultipleSweeps >= 8)
      {
        sweep = 8;
        for (currentX = Xmid, currentY = Ylow; ((currentX >Xlow) && (currentY<Ymid)); currentX -= Dstep, currentY += Dstep)
	  {
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);

	    // Sweep 8---Scanset is (currentX, currentY, delPos, delPos)
	    result = SearchSingleSetOutXY( pLgMaster
                               , currentX
                               , currentY
                               , delPos
                               , delPos
                               , gLout1
                               , nYsteps
                               , &countX
                               , &countY
                               , &sumX
                               , &sumY
                               );
	    if (result)
	      return(result);
	    // Sweep 8---Scanset is (currentX+(delPos * steps), (currentY+(delPos*steps)), delNeg, delNeg)
	    newX =  currentX + (Dstep * nYsteps);
	    newY =  currentY + (Dstep * nYsteps);
	    result = SearchSingleSetOutXY( pLgMaster
                               , newX
                               , newY
                               , delNeg
                               , delNeg
                               , gLout1
                               , nYsteps
                               , &countX
                               , &countY
                               , &sumX
                               , &sumY
                               );
	    if (result)
	      return(result);
	    if (countX >= 1)
	      syslog(LOG_NOTICE,"SuperSearch:  Sweep %d, hit countX %d %d, #samples %d,delPos=%x",sweep,countX,countY,nYsteps,delPos);
          }
      }

    if ((countX < 3) && (gSuperIndex == 0))
      {
        syslog(LOG_NOTICE,"SuperSearch: END: Not enough hits, Sweep %d, hit countX %d, #samples %d,delNeg=%x,delPos=%x",
           sweep, countX ,nYsteps,delNeg,delPos);
        return(kSuperFineTooFew);
      }
    if ((countY < 3) && (gSuperIndex == 0))
      {
        syslog(LOG_NOTICE,"SuperSearch: END: Not enough hits, Sweep %d, hit countY %d, #samples %d,delNeg=%x,delPos=%x",
           sweep, countY ,nYsteps,delNeg,delPos);
        return(kSuperFineTooFew);
      }

    if (countX >= 3 && countY >= 3)
      {
        avgX = (int16_t)(sumX / countX);
        avgY = (int16_t)(sumY / countY);
        *foundX = avgX;
        *foundY = avgY;
        return(0);
      }
#if 0
    else if (gSuperIndex > 0)
      {
        sumX = 0;
        sumY = 0;
        for (i = 0; i < gSuperIndex; i++)
	  {
	    sumX += gXsuperSave[i];
	    sumY += gYsuperSave[i];
	  }
        avgX = (int16_t)(sumX / gSuperIndex);
        avgY = (int16_t)(sumY / gSuperIndex);
        *foundX = avgX;
        *foundY = avgY;
      }
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
