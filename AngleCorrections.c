#include <stdint.h>
//static char rcsid[] = "$Id: AngleCorrections.c,v 1.11 2001/01/03 17:46:35 ags-sw Exp pickle $";

#include <float.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <ctype.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include <unistd.h>
#include "BoardComm.h"
#include "AppCommon.h"
#include "AppErrors.h"
#include "AngleCorrections.h"
#include "LaserInterface.h"
#include "3DTransform.h"
#include "AutoCert.h"
#include "Files.h"

#if GENERATINGCFM
#define setround(x)
#define rint(x) floor(x)
#else
#endif

#include <stdlib.h>
#include <string.h>

enum
{
	kNone,
	kX,
	kY
};

#define kSIX	0
#define kFOUR	0
#define	kMaxNumberOfCALIBAngles	4000

#define	kGridPointsX	61
#define	kGridPointsY	61
#define	kGridOriginX	-30.0
#define	kGridStepX		1.0
#define	kGridOriginY	-30.0
#define	kGridStepY		1.0

static	unsigned char		gHalfWayThru = false;

static	double	AbsCos ( double x1, double y1,
						double x2, double y2 );

static	void	OldRemoveCorrection
					( double *x, double *y );
					
static	void	OldApplyCorrection
					( double *x, double *y );

static	char **		gFileBuffer = (char **)NULL;
static	int32_t		gFileLength = 0L;

static	double	**gXTheoryHdl = (double **)NULL;
static	double	**gYTheoryHdl = (double **)NULL;

static	double	**gXActualHdl = (double **)NULL;
static	double	**gYActualHdl = (double **)NULL;

static	short		gNumberOfPoints = 0;

static	double	(*gXTheoryCoeffs)[kGridPointsY][6] =
						(double (*)[kGridPointsY][6])NULL;
static	double	(*gYTheoryCoeffs)[kGridPointsY][6] =
						(double (*)[kGridPointsY][6])NULL;

static	double	(*gXActualCoeffs)[kGridPointsY][6] =
						(double (*)[kGridPointsY][6])NULL;
static	double	(*gYActualCoeffs)[kGridPointsY][6] =
						(double (*)[kGridPointsY][6])NULL;

static	short		*gSortedPoints = (short *)NULL;

static	double	gCurrentX;
static	double gCurrentY;

static	double	*gCurrentXTable;
static	double *gCurrentYTable;

						
static	unsigned char		BuildTwoTables ( double *x, double *y,
						double *zX, double *zY,
						double coeffsX[][kGridPointsY][6],
						double coeffsY[][kGridPointsY][6] );

static	int			DistanceFromCurrentPoint
						( const void *elem1, const void *elem2 );
						
static	unsigned char		GetPlaneCoefficients ( double *z,
						double *coefficients,
						double *x, double *y,
						short *indices, short type );
						
#if	kFOUR
static	unsigned char		GetBetterPlaneCoefficients ( double *z,
						double *coefficients,
						double *x, double *y,
						short *indices, short type );
#endif

#if kSIX
static	unsigned char		GetParabolicCoefficients ( double *z,
						double *coefficients,
						double *x, double *y,
						short *indices, short type );
#endif

static void DoCorrection(struct lg_master *pLgMaster, double *x, double *y,
			 double	xCoeffs[][kGridPointsY][6],
			 double	yCoeffs[][kGridPointsY][6]);
						
static	void		CleanupInit ( void );

static	unsigned char		AllocateAngleBuffers ( void );
static	unsigned char		AllocateCorrectionTables ( void );
static	unsigned char		ProcessFile (struct lg_master *pLgMaster);
						
enum
{
	kInitializingAngluarCorrectionsMsg = 1,
	kCannotAllocateAngleBuffer,
	kCalibrationFileNameNotFoundMsg,
	kCalibrationFileNotFound,
	kCalibrationFileCannotBeOpened,
	kCalibrationFileCannotBeSized,
	kCannotFitFileInMemory,
	kCannotReadCalibrationFile,
	kCalibrationFileCannotBeClosed,
	kLineIsTooLong,
	kDeltaMirrorCorrupted,
	kBadLine,
	kTooManyLines,
	kNoDeltaMirror,
	kProblemsSettingUpTheory,
	kProblemsSettingUpActual,
	kCannotAllocateTableBuffer,
	kCannotAllocateSortBuffer,
	kBuildingCorrectionTable,
	kBuildingCorruptionTable,
	kToleranceCorrupted,
	kSerialNumberCorrupted,
	kNoTolerance,
	kNoSerialNumber,
	kAutoCertFile,
	kCmdPeriodToStop
};

enum
{
	kCalibrationFileName = 1
};


void ApplyCorrection (struct lg_master *pLgMaster, double *x, double *y )
{
	if ( !pLgMaster->gCALIBFileOK ) OldApplyCorrection ( x, y );
	else DoCorrection(pLgMaster, x, y, gXActualCoeffs, gYActualCoeffs );
}

void RemoveCorrection(struct lg_master *pLgMaster, double *x, double *y )
{
	if (!pLgMaster->gCALIBFileOK)
	  OldRemoveCorrection ( x, y );
	else DoCorrection(pLgMaster, x, y, gXTheoryCoeffs, gYTheoryCoeffs );
}

static void DoCorrection(struct lg_master *pLgMaster, double *x, double *y,
	double	xCoeffs[][kGridPointsY][6],
	double	yCoeffs[][kGridPointsY][6] )
{
	double temp, indexLD, pointsLD;
	double weightRight, weightLeft, weightUp, weightDown;
	double wDwL, wDwR, wUwL, wUwR;
	short xIndex, yIndex, xIndex1, yIndex1; 

	if (!pLgMaster->gCALIBFileOK)
	  return;

	/* setround ( DOWNWARD ); */
#ifdef ANGDEBUG
fprintf( stderr, "DoCorr187 xy  %lf %lf\n", *x, *y );
#endif
	
	if ( *x < kGridOriginX )
	{
		xIndex = 0;
		temp = kGridOriginX - *x;
		weightRight = temp / ( temp + temp + kGridStepX );
		weightLeft = 1.0 - weightRight;
	}
	else
	{

		temp = ( *x - kGridOriginX ) / kGridStepX;
		indexLD = (double)floor ( 0.5 + (double)temp );
		pointsLD = (double)( kGridPointsX - 1 );
		if ( indexLD >= pointsLD )
		{
			xIndex = kGridPointsX - 2;
			temp = *x - kGridOriginX - kGridStepX * pointsLD;
			weightLeft = temp / ( temp + temp + kGridStepX );
			weightRight = 1.0 - weightLeft;
		}
		else
		{
			xIndex = (short)indexLD;
			weightRight = temp - indexLD;
			weightLeft = 1.0 - weightRight;
		}
	} 
	
	if ( *y < kGridOriginY )
	{
		yIndex = 0;
		temp = kGridOriginY - *y;
		weightUp = temp / ( temp + temp + kGridStepY );
		weightDown = 1.0 - weightUp;
	}
	else
	{

		temp = ( *y - kGridOriginY ) / kGridStepY;
		indexLD = (double)floor( 0.5 + (double)temp );
		pointsLD = (double)( kGridPointsY - 1 );
		if ( indexLD >= pointsLD )
		{
			yIndex = kGridPointsY - 2;
			temp = *y - kGridOriginY - kGridStepY * pointsLD;
			weightDown = temp / ( temp + temp + kGridStepY );
			weightUp = 1.0 - weightDown;
		}
		else
		{
			yIndex = (short)indexLD;
			weightUp = temp - indexLD;
			weightDown = 1.0 - weightUp;
		}
	} 

	/* setround ( TONEAREST ); */
	
	wDwL = weightDown * weightLeft;
	wDwR = weightDown * weightRight;
	wUwL = weightUp * weightLeft;
	wUwR = weightUp * weightRight;
	xIndex1 = xIndex + 1;
	yIndex1 = yIndex + 1;

#ifdef ANGDEBUG
fprintf( stderr, "DoCorr256 wDwL %lf\n", wDwL );
fprintf( stderr, "DoCorr256 wDwR %lf\n", wDwR );
fprintf( stderr, "DoCorr256 wUwL %lf\n", wUwL );
fprintf( stderr, "DoCorr256 wUwR %lf\n", wUwR );
fprintf( stderr, "DoCorr258 xyIndex1  %d %d\n", xIndex1, yIndex1 );
fprintf( stderr, "xCoeffs %d %d 0 %lf\n", xIndex,  yIndex , xCoeffs[xIndex ][yIndex ][0] );
fprintf( stderr, "xCoeffs %d %d 0 %lf\n", xIndex1, yIndex , xCoeffs[xIndex1][yIndex ][0] );
fprintf( stderr, "xCoeffs %d %d 0 %lf\n", xIndex,  yIndex1, xCoeffs[xIndex ][yIndex1][0] );
fprintf( stderr, "xCoeffs %d %d 0 %lf\n", xIndex1, yIndex1, xCoeffs[xIndex1][yIndex1][0] );
fprintf( stderr, "xCoeffs %d %d 1 %lf\n", xIndex,  yIndex , xCoeffs[xIndex ][yIndex ][1] );
fprintf( stderr, "xCoeffs %d %d 1 %lf\n", xIndex1, yIndex , xCoeffs[xIndex1][yIndex ][1] );
fprintf( stderr, "xCoeffs %d %d 1 %lf\n", xIndex,  yIndex1, xCoeffs[xIndex ][yIndex1][1] );
fprintf( stderr, "xCoeffs %d %d 1 %lf\n", xIndex1, yIndex1, xCoeffs[xIndex1][yIndex1][1] );
fprintf( stderr, "xCoeffs %d %d 2 %lf\n", xIndex,  yIndex , xCoeffs[xIndex ][yIndex ][2] );
fprintf( stderr, "xCoeffs %d %d 2 %lf\n", xIndex1, yIndex , xCoeffs[xIndex1][yIndex ][2] );
fprintf( stderr, "xCoeffs %d %d 2 %lf\n", xIndex,  yIndex1, xCoeffs[xIndex ][yIndex1][2] );
fprintf( stderr, "xCoeffs %d %d 2 %lf\n", xIndex1, yIndex1, xCoeffs[xIndex1][yIndex1][2] );
fprintf( stderr, "yCoeffs %d %d 0 %lf\n", xIndex,  yIndex , yCoeffs[xIndex ][yIndex ][0] );
fprintf( stderr, "yCoeffs %d %d 0 %lf\n", xIndex1, yIndex , yCoeffs[xIndex1][yIndex ][0] );
fprintf( stderr, "yCoeffs %d %d 0 %lf\n", xIndex,  yIndex1, yCoeffs[xIndex ][yIndex1][0] );
fprintf( stderr, "yCoeffs %d %d 0 %lf\n", xIndex1, yIndex1, yCoeffs[xIndex1][yIndex1][0] );
fprintf( stderr, "yCoeffs %d %d 1 %lf\n", xIndex,  yIndex , yCoeffs[xIndex ][yIndex ][1] );
fprintf( stderr, "yCoeffs %d %d 1 %lf\n", xIndex1, yIndex , yCoeffs[xIndex1][yIndex ][1] );
fprintf( stderr, "yCoeffs %d %d 1 %lf\n", xIndex,  yIndex1, yCoeffs[xIndex ][yIndex1][1] );
fprintf( stderr, "yCoeffs %d %d 1 %lf\n", xIndex1, yIndex1, yCoeffs[xIndex1][yIndex1][1] );
fprintf( stderr, "yCoeffs %d %d 2 %lf\n", xIndex,  yIndex , yCoeffs[xIndex ][yIndex ][2] );
fprintf( stderr, "yCoeffs %d %d 2 %lf\n", xIndex1, yIndex , yCoeffs[xIndex1][yIndex ][2] );
fprintf( stderr, "yCoeffs %d %d 2 %lf\n", xIndex,  yIndex1, yCoeffs[xIndex ][yIndex1][2] );
fprintf( stderr, "yCoeffs %d %d 2 %lf\n", xIndex1, yIndex1, yCoeffs[xIndex1][yIndex1][2] );
#endif
	
	temp =
		wDwL * xCoeffs[xIndex ][yIndex ][0] +
		wDwR * xCoeffs[xIndex1][yIndex ][0] +
		wUwL * xCoeffs[xIndex ][yIndex1][0] +
		wUwR * xCoeffs[xIndex1][yIndex1][0]
		+ *x * (
		wDwL * xCoeffs[xIndex ][yIndex ][1] +
		wDwR * xCoeffs[xIndex1][yIndex ][1] +
		wUwL * xCoeffs[xIndex ][yIndex1][1] +
		wUwR * xCoeffs[xIndex1][yIndex1][1] )
		+ *y * (
		wDwL * xCoeffs[xIndex ][yIndex ][2] +
		wDwR * xCoeffs[xIndex1][yIndex ][2] +
		wUwL * xCoeffs[xIndex ][yIndex1][2] +
		wUwR * xCoeffs[xIndex1][yIndex1][2] )
#if	kSIX
		+ *x * *x * (
		wDwL * xCoeffs[xIndex ][yIndex ][3] +
		wDwR * xCoeffs[xIndex1][yIndex ][3] +
		wUwL * xCoeffs[xIndex ][yIndex1][3] +
		wUwR * xCoeffs[xIndex1][yIndex1][3] )
		+ *x * *y * (
		wDwL * xCoeffs[xIndex ][yIndex ][4] +
		wDwR * xCoeffs[xIndex1][yIndex ][4] +
		wUwL * xCoeffs[xIndex ][yIndex1][4] +
		wUwR * xCoeffs[xIndex1][yIndex1][4] )
		+ *y * *y * (
		wDwL * xCoeffs[xIndex ][yIndex ][5] +
		wDwR * xCoeffs[xIndex1][yIndex ][5] +
		wUwL * xCoeffs[xIndex ][yIndex1][5] +
		wUwR * xCoeffs[xIndex1][yIndex1][5] )
#endif
		;
	
	*y =
		wDwL * yCoeffs[xIndex ][yIndex ][0] +
		wDwR * yCoeffs[xIndex1][yIndex ][0] +
		wUwL * yCoeffs[xIndex ][yIndex1][0] +
		wUwR * yCoeffs[xIndex1][yIndex1][0]
		+ *x * (
		wDwL * yCoeffs[xIndex ][yIndex ][1] +
		wDwR * yCoeffs[xIndex1][yIndex ][1] +
		wUwL * yCoeffs[xIndex ][yIndex1][1] +
		wUwR * yCoeffs[xIndex1][yIndex1][1] )
		+ *y * (
		wDwL * yCoeffs[xIndex ][yIndex ][2] +
		wDwR * yCoeffs[xIndex1][yIndex ][2] +
		wUwL * yCoeffs[xIndex ][yIndex1][2] +
		wUwR * yCoeffs[xIndex1][yIndex1][2] )
#if	kSIX
		+ *x * *x * (
		wDwL * yCoeffs[xIndex ][yIndex ][3] +
		wDwR * yCoeffs[xIndex1][yIndex ][3] +
		wUwL * yCoeffs[xIndex ][yIndex1][3] +
		wUwR * yCoeffs[xIndex1][yIndex1][3] )
		+ *x * *y * (
		wDwL * yCoeffs[xIndex ][yIndex ][4] +
		wDwR * yCoeffs[xIndex1][yIndex ][4] +
		wUwL * yCoeffs[xIndex ][yIndex1][4] +
		wUwR * yCoeffs[xIndex1][yIndex1][4] )
		+ *y * *y * (
		wDwL * yCoeffs[xIndex ][yIndex ][5] +
		wDwR * yCoeffs[xIndex1][yIndex ][5] +
		wUwL * yCoeffs[xIndex ][yIndex1][5] +
		wUwR * yCoeffs[xIndex1][yIndex1][5] )
#endif
		;
	
	*x = temp;	

#if defined( ANGDEBUG )
fprintf( stderr, "DoCorr326 xy  %lf %lf\n", *x, *y );
#endif
	
	return;
}


void OldApplyCorrection ( double *x, double *y )
{
  // Next two lines fix gcc complaints
  *x = *x;
  *y = *y;
  return;
}
					
void OldRemoveCorrection ( double *x, double *y )
{
  // Next two lines fix gcc complaints
  *x = *x;
  *y = *y;
  return;
}


void CloseAngleCorrections ( void )
{
	if ( gXTheoryCoeffs ) free( (void *)gXTheoryCoeffs );
	gXTheoryCoeffs = (double (*)[kGridPointsY][6])NULL;
	if ( gYTheoryCoeffs ) free( (void *)gYTheoryCoeffs );
	gYTheoryCoeffs = (double (*)[kGridPointsY][6])NULL;
	if ( gXActualCoeffs ) free( (void *)gXActualCoeffs );
	gXActualCoeffs = (double (*)[kGridPointsY][6])NULL;
	if ( gYActualCoeffs ) free( (void *)gYActualCoeffs );
	gYActualCoeffs = (double (*)[kGridPointsY][6])NULL;
}

#if kSIX
#include "NumRec.c"
#endif
unsigned char InitAngleCorrections (struct lg_master *pLgMaster)
{
	int filenum;
	uint16_t i;
        int32_t j;
        int length;
	unsigned char fTableBuilt;
        char * gTempBuff;
        char * ptr;

	gFileLength = 0L;
	gFileBuffer = (char **)NULL;
	gXTheoryHdl = (double **)NULL;
	gYTheoryHdl = (double **)NULL;
	gXActualHdl = (double **)NULL;
	gYActualHdl = (double **)NULL;
	gXTheoryCoeffs = (double (*)[kGridPointsY][6])NULL;
	gYTheoryCoeffs = (double (*)[kGridPointsY][6])NULL;
	gXActualCoeffs = (double (*)[kGridPointsY][6])NULL;
	gYActualCoeffs = (double (*)[kGridPointsY][6])NULL;
	gSortedPoints = (short *)NULL;
	
        
	filenum = open( "/etc/ags/conf/calib", O_RDONLY );
	if ( filenum <= 0 ) { 
             printf( "calibration file missing\n" );
             exit(1);
        }
        gTempBuff = (char *)malloc( (size_t)CALIB_SIZE );
        length = read( filenum, gTempBuff, CALIB_SIZE );
        printf( "calibration file size %d\n", length );
        close( filenum );
        
        j = 0L;
        ptr = gTempBuff;
             /* change all CRs to LFs and upper to lower case */
        while( (*ptr) && (j < (int32_t)CALIB_SIZE) ) {
          if ( *ptr == '\r' ) *ptr = '\n';
          if ( isupper( *ptr ) ) {
             *ptr = tolower( *ptr );
          }
          ptr++; j++;
        }
        gFileLength = j;

        gFileBuffer = (char **)malloc( sizeof(void *) + (size_t)gFileLength );
        if( gFileBuffer ) {
                *(void **)gFileBuffer = (void *)gFileBuffer + sizeof (void *);
        } else {
		gFileBuffer = (char **)NULL;
		return false;
	}
	
        memmove( *gFileBuffer, gTempBuff, (size_t)gFileLength );

	if ( !AllocateAngleBuffers (  ) ) return false;
	
	
	if ( !ProcessFile (pLgMaster) ) return false;

        fprintf( stderr, "gDeltaMirror %lf\n", gDeltaMirror );
        fprintf( stderr, "gMirrorThickness %lf\n", gMirrorThickness );
        fprintf( stderr, "gHalfMirror %lf\n",  gHalfMirror );
	
	if ( gFileBuffer ) free((void *)gFileBuffer );
	gFileBuffer = (char **)NULL;
	
	if ( !AllocateCorrectionTables (  ) ) return false;
	
	gSortedPoints = (short *)
		malloc ( gNumberOfPoints * sizeof ( short ) );
	if ( ( gSortedPoints == (short *)NULL )  )
	{
		gSortedPoints = (short *)NULL;
		return false;
	}
	
	i = gNumberOfPoints;
	while ( i-- ) gSortedPoints[i] = i;
	
	gHalfWayThru = false;
	
/*
**	theXTheoryHdlState = HGetState ( (char **)gXTheoryHdl );
**	theYTheoryHdlState = HGetState ( (char **)gYTheoryHdl );
**	theXActualHdlState = HGetState ( (char **)gXActualHdl );
**	theYActualHdlState = HGetState ( (char **)gYActualHdl );
**	HLock ( (char **)gXTheoryHdl );
**	HLock ( (char **)gYTheoryHdl );
**	HLock ( (char **)gXActualHdl );
**	HLock ( (char **)gYActualHdl );
*/
	fTableBuilt = BuildTwoTables ( *gXTheoryHdl, *gYTheoryHdl,
		*gXActualHdl, *gYActualHdl, gXActualCoeffs, gYActualCoeffs );
/*
**	HSetState ( (char **)gXTheoryHdl, theXTheoryHdlState );
**	HSetState ( (char **)gYTheoryHdl, theYTheoryHdlState );
**	HSetState ( (char **)gXActualHdl, theXActualHdlState );
**	HSetState ( (char **)gYActualHdl, theYActualHdlState );
*/

	if ( !fTableBuilt )
	{
		return false;
	}
	

	gHalfWayThru = true;
	
/*
**	HLock ( (char **)gXTheoryHdl );
**	HLock ( (char **)gYTheoryHdl );
**	HLock ( (char **)gXActualHdl );
**	HLock ( (char **)gYActualHdl );
*/
	fTableBuilt = BuildTwoTables ( *gXActualHdl, *gYActualHdl,
		*gXTheoryHdl, *gYTheoryHdl, gXTheoryCoeffs, gYTheoryCoeffs );
/*
**	HSetState ( (char **)gXTheoryHdl, theXTheoryHdlState );
**	HSetState ( (char **)gYTheoryHdl, theYTheoryHdlState );
**	HSetState ( (char **)gXActualHdl, theXActualHdlState );
**	HSetState ( (char **)gYActualHdl, theYActualHdlState );
**	EndProgressBar (  );
*/
	
	if ( gSortedPoints ) free( (void *)gSortedPoints );
	gSortedPoints = (short *)NULL;
		
	if ( !fTableBuilt )
	{
		return false;
	}
	
	CleanupInit (  );
	pLgMaster->gCALIBFileOK = true;
	fprintf( stderr, "gCALIBFileOK = true\n" );
	return true;
}

unsigned char ProcessFile (struct lg_master *pLgMaster)
{
	char * currentFilePtr;
	char * newFilePtr;
	unsigned char fWasPound, fWasDeltaMirror;
	unsigned char fWasTolerance, fWasSerialNumber;
	short lineNumber, numberOfDollars;
        int32_t  lineLength;
	char theLinePtr[1024];
	double tX, tY, tZ, aX, aY, dX, dY;
        double dtemp1;
	double currentTolerance, currentDeltaMirror;
	int32_t processedLength, currentSerialNumber;
	
	gNumberOfPoints = 0;
	currentFilePtr = *gFileBuffer;
	fWasPound = false;
	fWasDeltaMirror = false;
	lineNumber = 0;
	numberOfDollars = 0;
	processedLength = 0L;
	
	while ( gFileLength )
	{
		lineNumber++;

		lineLength = 0L;
		currentFilePtr = &(*gFileBuffer)[processedLength];
		newFilePtr = currentFilePtr;

                while ( ( lineLength < gFileLength ) &&
                        ( *newFilePtr != 0x0A )      &&
                        ( *newFilePtr != 0x0D ) ) {
                  lineLength++;
                  newFilePtr++;
                }
		if ( ( lineLength != gFileLength ) &&
			((*newFilePtr == 0x0A) || (*newFilePtr == 0x0D)) )
		{
			lineLength++;
			newFilePtr++;
		}

		if ( lineLength > 1023 )
		{
			return false;
		}
		
		gFileLength -= lineLength;
		processedLength += lineLength;
		if ( lineLength == 0 ) continue;

		
		memmove( (void *)theLinePtr, (void *)currentFilePtr,
                         (size_t)lineLength );
		currentFilePtr = newFilePtr;
		
		if (  ( theLinePtr[0] == 0x0D ) || ( theLinePtr[0] == 0x0A ) )
			continue;

		if (  ( theLinePtr[lineLength - 1] ==  0x0D )
		   || ( theLinePtr[lineLength - 1] ==  0x0A ) )
			theLinePtr[lineLength - 1] = '\0';
				
		if ( memcmp ( theLinePtr, "$AutoCert", 9 ) == 0 )
		{
			return false;
		}

		if ( theLinePtr[0] == '$' )
		{
			numberOfDollars++;
			switch ( numberOfDollars )
			{
				case 8:
					if ( sscanf (theLinePtr
                                                    , "$%lf"
                                                    , &dtemp1
                                                    ) == 1 )
					{
					    gMirrorThickness =       dtemp1;
                                            gHalfMirror      = 0.5 * dtemp1;
					}
#ifdef ANGDEBUG
fprintf( stderr, "gMirrorThickness %lf\n", gMirrorThickness );
#endif
                                        break;
				case 7:
					currentDeltaMirror = gDeltaMirror;
					if ( sscanf ( theLinePtr, "$%lf ",
						&gDeltaMirror ) != 1 )
					{
					    gDeltaMirror = currentDeltaMirror;
					    return false;
					}
					fWasDeltaMirror = true;
#ifdef ANGDEBUG
fprintf( stderr, "gDeltaMirror %lf\n", gDeltaMirror );
#endif
					break;
				case 6:
					currentTolerance = pLgMaster->gTolerance;
					if ( sscanf ( theLinePtr, "$%lf ",
						&pLgMaster->gTolerance ) != 1 )
					{
						pLgMaster->gTolerance = currentTolerance;
						return false;
#ifdef ANGDEBUG
fprintf( stderr, "gTolerance %lf\n", pLgMaster->gTolerance );
#endif
					}
					fWasTolerance = true;
					break;
				case 5:
					currentSerialNumber = pLgMaster->gProjectorSerialNumber;
					if ( sscanf ( theLinePtr, "$%ld ",
						&pLgMaster->gProjectorSerialNumber ) != 1 )
					{
						pLgMaster->gProjectorSerialNumber = currentSerialNumber;
#ifdef ANGDEBUG
fprintf( stderr, "serial number %d\n", currentSerialNumber );
#endif
						return false;
					}
					fWasSerialNumber = true;
					break;
				default:
					break;
			}
			continue;
		}
		
		fWasPound = false;
		if ( theLinePtr[0] == '#' ) fWasPound = true;
		if ( fWasPound ) continue;

		if ( sscanf ( theLinePtr,
				" %lf %lf %lf %lf %lf %lf %lf ", &tX, &tY, &tZ,
				&aX, &aY, &dX, &dY  ) != 7 )
		{
			return false;
		}
#ifdef ANGDEBUG
fprintf( stderr, "calibdata %lf %lf %lf %lf %lf %lf %lf\n", tX, tY, tZ,
                                aX, aY, dX, dY);
#endif

		if ( gNumberOfPoints >= kMaxNumberOfCALIBAngles )
		{
			return false;
		}
		
		if ( !fWasDeltaMirror )
		{
			return false;
		}
		

#ifdef ANGDEBUG
fprintf( stderr, "gNumberOfPoints %d\n", gNumberOfPoints);
#endif
		GeometricAnglesFrom3D ( tX, tY, tZ,
			&(*gXTheoryHdl)[gNumberOfPoints],
			&(*gYTheoryHdl)[gNumberOfPoints] );
			
		ConvertGeometricAnglesToMirror (
			&(*gXTheoryHdl)[gNumberOfPoints],
			&(*gYTheoryHdl)[gNumberOfPoints] );
		
		GeometricAnglesFrom3D ( aX, aY, tZ,
			&(*gXActualHdl)[gNumberOfPoints],
			&(*gYActualHdl)[gNumberOfPoints] );
			
		ConvertGeometricAnglesToMirror (
			&(*gXActualHdl)[gNumberOfPoints],
			&(*gYActualHdl)[gNumberOfPoints] );

		gNumberOfPoints++;
	}
	
	if ( !fWasTolerance )
	{
		return false;
	}
	
	if ( !fWasSerialNumber )
	{
		return false;
	}
	
fprintf( stderr, "Points %d    ", gNumberOfPoints );
fprintf( stderr, "Mirror %f   ", gDeltaMirror );
fprintf( stderr, "SerialNumber %ld\n", pLgMaster->gProjectorSerialNumber );
	return true;
}

unsigned char AllocateAngleBuffers ( void )
{
		
	gXTheoryHdl = malloc( sizeof (void *) +
         (size_t) ( kMaxNumberOfCALIBAngles * sizeof ( double ) ) );
        if( gXTheoryHdl ) {
                *(void **)gXTheoryHdl = (void *)gXTheoryHdl + sizeof (void *);
        } else {
		gXTheoryHdl = (double **)NULL;
		return false;
	}

	gYTheoryHdl = malloc( sizeof (void *) +
         (size_t) ( kMaxNumberOfCALIBAngles * sizeof ( double ) ) );
        if( gYTheoryHdl ) {
                *(void **)gYTheoryHdl = (void *)gYTheoryHdl + sizeof (void *);
        } else {
		gYTheoryHdl = (double **)NULL;
		return false;
	}

	gXActualHdl = malloc( sizeof (void *) +
         (size_t) ( kMaxNumberOfCALIBAngles * sizeof ( double ) ) );
        if( gXActualHdl ) {
                *(void **)gXActualHdl = (void *)gXActualHdl + sizeof (void *);
        } else {
		gXActualHdl = (double **)NULL;
		return false;
	}

	gYActualHdl = malloc( sizeof (void *) +
         (size_t) ( kMaxNumberOfCALIBAngles * sizeof ( double ) ) );
        if( gYActualHdl ) {
                *(void **)gYActualHdl = (void *)gYActualHdl + sizeof (void *);
        } else {
		gYActualHdl = (double **)NULL;
		return false;
	}

	return true;
}


unsigned char AllocateCorrectionTables ( void )
{
	
	gXTheoryCoeffs = (double (*)[kGridPointsY][6])malloc
		( kGridPointsX * kGridPointsY * 6 * sizeof ( double ) );
	if ( gXTheoryCoeffs == (double (*)[kGridPointsY][6])NULL ) {
		gXTheoryCoeffs = (double (*)[kGridPointsY][6])NULL;
		return false;
	}
	
	gYTheoryCoeffs = (double (*)[kGridPointsY][6])malloc
		( kGridPointsX * kGridPointsY * 6 * sizeof ( double ) );
	if ( gYTheoryCoeffs == (double (*)[kGridPointsY][6])NULL ) {
		gYTheoryCoeffs = (double (*)[kGridPointsY][6])NULL;
		return false;
	}
	
	gXActualCoeffs = (double (*)[kGridPointsY][6])malloc
		( kGridPointsX * kGridPointsY * 6 * sizeof ( double ) );
	if ( gXActualCoeffs == (double (*)[kGridPointsY][6])NULL ) {
		gXActualCoeffs = (double (*)[kGridPointsY][6])NULL;
		return false;
	}
	
	gYActualCoeffs = (double (*)[kGridPointsY][6])malloc
		( kGridPointsX * kGridPointsY * 6 * sizeof ( double ) );
	if ( gYActualCoeffs == (double (*)[kGridPointsY][6])NULL )
	{
		gYActualCoeffs = (double (*)[kGridPointsY][6])NULL;
		return false;
	}
	
	return true;
}

void CleanupInit ( void )
{
	if ( gXTheoryHdl ) free ( (void *)gXTheoryHdl );
	gXTheoryHdl = (double **)NULL;
	if ( gYTheoryHdl ) free ( (void *)gYTheoryHdl );
	gYTheoryHdl = (double **)NULL;
	if ( gXActualHdl ) free ( (void *)gXActualHdl );
	gXTheoryHdl = (double **)NULL;
	if ( gYActualHdl ) free ( (void *)gYActualHdl );
	gYActualHdl = (double **)NULL;
	if ( gFileBuffer ) free ( (void *)gFileBuffer );
	gFileBuffer = (char **)NULL;
	if ( gSortedPoints ) free ( (void *)gSortedPoints );
	gSortedPoints = (short *)NULL;
}


unsigned char BuildTwoTables ( double *x, double *y,
	double *zX, double *zY,
	double coeffsX[][kGridPointsY][6],
	double coeffsY[][kGridPointsY][6] )
{
	short index[6], nX, nY, i, j, k;
	short indexCacheX[6], indexCacheY[6], n;
	unsigned char fPlaneFoundX, fPlaneFoundY;
	double coeffsXcache[6], coeffsYcache[6];

	
	i = 6;
	while ( i-- )
	{
		indexCacheX[i] = gNumberOfPoints;
		indexCacheY[i] = gNumberOfPoints;
	}
	
	gCurrentXTable = x;
	gCurrentYTable = y;
	
	for ( nX = 0; nX < kGridPointsX; nX++ )
	{
		for ( nY = 0; nY < kGridPointsY; nY++ )
		{
			gCurrentX = kGridOriginX + nX * kGridStepX;
			gCurrentY = kGridOriginY + nY * kGridStepY;
			qsort ( (void *)gSortedPoints, (size_t)gNumberOfPoints,
				sizeof ( short ), &DistanceFromCurrentPoint );

			fPlaneFoundX = false;
			fPlaneFoundY = false;
#if		kSIX
			for ( l = 3; l < ( gNumberOfPoints - 2 ); l++ )
			{
				for ( k = 2; k < l; k++ )
				{
					for ( j = 1; j < k; j++ )
					{
						for ( i = 0; i < j; i++ )
						{
							for ( q = l + 2; q < gNumberOfPoints; q++ )
							{
								for ( p = l + 1; p < q; p++ )
								{
#elif	kFOUR
			for ( l = 3; l < gNumberOfPoints; l++ )
			{
				for ( k = 2; k < l; k++ )
				{
					
					for ( j = 1; j < k; j++ )
					{
						
						for ( i = 0; i < j; i++ )
						{
#else
			for ( k = 2; k < gNumberOfPoints; k++ )
			{
				
				for ( j = 1; j < k; j++ )
				{
					
					for ( i = 0; i < j; i++ )
					{
#endif
						index[0] = gSortedPoints[i];
						index[1] = gSortedPoints[j];
						index[2] = gSortedPoints[k];
#if		kSIX
						index[3] = gSortedPoints[l];
						index[4] = gSortedPoints[p];
						index[5] = gSortedPoints[q];
#elif	kFOUR
						index[3] = gSortedPoints[l];
#endif
						if ( !fPlaneFoundX )
						{
							if ( ( index[0] == indexCacheX[0] )
								&& ( index[1] == indexCacheX[1] )
								&& ( index[2] == indexCacheX[2] )
#if		kSIX
								&& ( index[3] == indexCacheX[3] )
								&& ( index[4] == indexCacheX[4] )
								&& ( index[5] == indexCacheX[5] )
#elif	kFOUR
								&& ( index[3] == indexCacheX[3] )
#endif
								)
							{
								n = 6;
								while ( n-- ) {
                                                                    coeffsX[nX][nY][n] = coeffsXcache[n];
#ifdef ANGDEBUG
fprintf(stderr, "coeffsX %lf %d %d %d\n",coeffsX[nX][nY][n], nX, nY, n );
#endif
                                                                }
								fPlaneFoundX = true;
							}
							else
							{
								fPlaneFoundX =
#if		kSIX
									GetParabolicCoefficients
#elif	kFOUR
									GetBetterPlaneCoefficients
#else
									GetPlaneCoefficients
#endif
										( zX, coeffsX[nX][nY],
										x, y, index, kX );
								if ( fPlaneFoundX )
								{
									n = 6;
									while ( n-- )
									{
										indexCacheX[n] = index[n];
										coeffsXcache[n] = 
											coeffsX[nX][nY][n];
#ifdef ANGDEBUG
fprintf(stderr, "coeffsX %lf %d %d %d\n",coeffsX[nX][nY][n], nX, nY, n );
#endif
									}
								}
							}
						}
						if ( !fPlaneFoundY )
						{
							if ( ( index[0] == indexCacheY[0] )
								&& ( index[1] == indexCacheY[1] )
								&& ( index[2] == indexCacheY[2] )
#if		kSIX
								&& ( index[3] == indexCacheY[3] )
								&& ( index[4] == indexCacheY[4] )
								&& ( index[5] == indexCacheY[5] )
#elif	kFOUR
								&& ( index[3] == indexCacheY[3] )
#endif
								)
							{
								n = 6;
								while ( n-- ) {
                                                                   coeffsY[nX][nY][n] = coeffsYcache[n];
#ifdef ANGDEBUG
fprintf(stderr, "coeffsY %lf %d %d %d\n",coeffsY[nX][nY][n], nX, nY, n );
#endif
                                                                }
								fPlaneFoundY = true;
							}
							else
							{
								fPlaneFoundY =
#if		kSIX
									GetParabolicCoefficients
#elif	kFOUR
									GetBetterPlaneCoefficients
#else
									GetPlaneCoefficients
#endif
										( zY, coeffsY[nX][nY],
										x, y, index, kY );
								if ( fPlaneFoundY )
								{
									n = 6;
									while ( n-- )
									{
										indexCacheY[n] = index[n];
										coeffsYcache[n] = 
											coeffsY[nX][nY][n];
#ifdef ANGDEBUG
fprintf(stderr, "coeffsY %lf %d %d %d\n",coeffsY[nX][nY][n], nX, nY, n );
#endif
									}
								}
							}
						}
#if		kSIX
									if ( fPlaneFoundX && fPlaneFoundY )
										break;
								}
								if ( fPlaneFoundX && fPlaneFoundY )
									break;
							}
							if ( fPlaneFoundX && fPlaneFoundY )
								break;
						}
						if ( fPlaneFoundX && fPlaneFoundY ) break;
					}
					if ( fPlaneFoundX && fPlaneFoundY ) break;
				}
				if ( fPlaneFoundX && fPlaneFoundY ) break;
			}
#elif	kFOUR
							if ( fPlaneFoundX && fPlaneFoundY ) break;
						}
						if ( fPlaneFoundX && fPlaneFoundY ) break;
					}
					if ( fPlaneFoundX && fPlaneFoundY ) break;
				}
				if ( fPlaneFoundX && fPlaneFoundY ) break;
			}
#else
						if ( fPlaneFoundX && fPlaneFoundY ) break;
					}
					if ( fPlaneFoundX && fPlaneFoundY ) break;
				}
				if ( fPlaneFoundX && fPlaneFoundY ) break;
			}
#endif
			if ( fPlaneFoundX && fPlaneFoundY ) continue;
			else return false;
		}
	}
	return true;
}

int DistanceFromCurrentPoint ( const void *elem1, const void *elem2 )
{
	register double dist, temp;
	temp = gCurrentXTable[*(const short *)elem1] - gCurrentX;
	dist = temp * temp;
	temp = gCurrentYTable[*(const short *)elem1] - gCurrentY;
	dist += temp * temp;
	temp = gCurrentXTable[*(const short *)elem2] - gCurrentX;
	dist -= temp * temp;
	temp = gCurrentYTable[*(const short *)elem2] - gCurrentY;
	dist -= temp * temp;
	if ( dist < 0.0 ) return -1;
	if ( dist > 0.0 ) return 1;
	return 0;
}
	
#define kDeterminantMin		0.000001	
#define kMaxCos				0.850	

#if	kFOUR
unsigned char GetBetterPlaneCoefficients ( double *z,
		double *coefficients, double *x, double *y,
		short *indices, short type )
{
	double m[3][3], mi[3][3], zv[3], tempCoeffs[3];
	short i, nOfPlanes;
	short *indexPtr;

	// FIXME--PAH--fix compile error: Get rid of parameter??
	type = type;
	
	indexPtr = indices;
	nOfPlanes = 0;
	i = 3;
	while ( i-- ) coefficients[i] = 0.0;
	for ( i = 0; i < 3; i++, indexPtr++ )
	{
		m[i][0] = 1.0;
		m[i][1] = x[*indexPtr];
		m[i][2] = y[*indexPtr];
		zv[i] = z[*indexPtr];
	}
	if ( ( fabs ( Determinant3D ( m ) ) > kDeterminantMin ) &&
		( AbsCos ( ( m[1][1] - m[0][1] ), ( m[1][2] - m[0][2] ), 
		( m[2][1] - m[0][1] ), ( m[2][2] - m[0][2] ) ) < kMaxCos ) )
	{
		Invert3DMatrix ( m, mi );
		Multiply3DMatrixByVector ( mi, zv, tempCoeffs );
		i = 3;
		while ( i-- ) coefficients[i] += tempCoeffs[i];
		nOfPlanes++;
	}
	m[2][1] = x[3];
	m[2][2] = y[3];
	zv[2] = z[3];
	if ( ( fabs ( Determinant3D ( m ) ) > kDeterminantMin ) &&
		( AbsCos ( ( m[1][1] - m[0][1] ), ( m[1][2] - m[0][2] ), 
		( m[2][1] - m[0][1] ), ( m[2][2] - m[0][2] ) ) < kMaxCos ) )
	{
		Invert3DMatrix ( m, mi );
		Multiply3DMatrixByVector ( mi, zv, tempCoeffs );
		i = 3;
		while ( i-- ) coefficients[i] += tempCoeffs[i];
		nOfPlanes++;
	}
	m[1][1] = x[2];
	m[1][2] = y[2];
	zv[1] = z[2];
	if ( ( fabs ( Determinant3D ( m ) ) > kDeterminantMin ) &&
		( AbsCos ( ( m[1][1] - m[0][1] ), ( m[1][2] - m[0][2] ), 
		( m[2][1] - m[0][1] ), ( m[2][2] - m[0][2] ) ) < kMaxCos ) )
	{
		Invert3DMatrix ( m, mi );
		Multiply3DMatrixByVector ( mi, zv, tempCoeffs );
		i = 3;
		while ( i-- ) coefficients[i] += tempCoeffs[i];
		nOfPlanes++;
	}
	if ( nOfPlanes == 0 ) return false;
	i = 3;
	while ( i-- ) coefficients[i] /= (double)nOfPlanes;
	return true;
}
#endif

unsigned char GetPlaneCoefficients ( double *z,
		double *coefficients, double *x, double *y,
		short *indices, short type )
{
	double m[3][3], mi[3][3], zv[3], temp;
	short i;

	// FIXME--PAH--fix compile error: Get rid of parameter??
	type = type;
	
	for ( i = 0; i < 3; i++, indices++ )
	{
		m[i][0] = 1.0;
		m[i][1] = x[*indices];
		m[i][2] = y[*indices];
		zv[i] = z[*indices];
	}
	if ( fabs ( Determinant3D ( m ) ) <= kDeterminantMin ) return false;
	temp = AbsCos ( ( m[1][1] - m[0][1] ), ( m[1][2] - m[0][2] ), 
		( m[2][1] - m[0][1] ), ( m[2][2] - m[0][2] ) );
	if ( temp >= kMaxCos ) return false;
	Invert3DMatrix ( m, mi );
	Multiply3DMatrixByVector ( mi, zv, coefficients );
	return true;
}

double AbsCos ( double x1, double y1,
	double x2, double y2 )
{
	return ( fabs ( x1 * x2 + y1 * y2 ) /
		sqrt ( ( x1 * x1 + y1 * y1 ) * ( x2 * x2 + y2 * y2 ) ) );
}


#if kSIX
unsigned char GetParabolicCoefficients ( double *z,
		double *coefficients, double *x, double *y,
		short *indices, short type )
{
	double m[6][6], zv[6];
	short i, indx[6];
	i = 0;
	while ( i < 6 )
	{
		m[i][0] = 1.0;
		m[i][1] = x[*indices];
		m[i][2] = y[*indices];
		m[i][3] = x[*indices] * x[*indices];
		m[i][4] = x[*indices] * y[*indices];
		m[i][5] = y[*indices] * y[*indices];
		switch ( type )
		{
			case kX:
				zv[i] = z[*indices] - x[*indices];
				break;
			case kY:
				zv[i] = z[*indices] - y[*indices];
				break;
			default:
				zv[i] = z[*indices];
				break;
		}
		i++;
		indices++;
	}
	if ( LUDCMP ( m, indx ) == 0.0 ) return false;
	if ( fabs (
		m[0][0] * m[1][1] * m[2][2] * m[3][3] * m[4][4] * m[5][5]
		) <= kDeterminantMin ) return false;
	LUBKSB ( m, indx, zv );
	memmove( (void *)coefficients, (void *)zv, sizeof ( zv ) );
	switch ( type )
	{
		case kX:
			coefficients[1] += 1.0;
			break;
		case kY:
			coefficients[2] += 1.0;
			break;
		default:
			break;
	}
	return true;
}
#endif
