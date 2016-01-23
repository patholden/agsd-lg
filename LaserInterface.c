#include <stdint.h>
//static char rcsid[] = "$Id: LaserInterface.c,v 1.17 1999/07/29 20:55:55 ags-sw Exp $";

#include <stdio.h>
#include <string.h>
#include <float.h>
#include <math.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "AppCommon.h"
#include "AppErrors.h"
#include "AppStrListIDs.h"
#include "LaserInterface.h"
#include "3DTransform.h"
#include "AngleCorrections.h"


int gMaxPiledPts;

static	double	gAPTunitsPerInch	= 1.0;

static	double	gXMirrorAngularRange = 30.0;	
static	double	gYMirrorAngularRange = 30.0;	

static	double	gXExternalCoefficient = +1.0;	
static	double	gYExternalCoefficient = -1.0;
static	double	gXExternalCenterAngle = 0.0;
static	double	gYExternalCenterAngle = 0.0;
/* These four numbers determine the wiring of the DACs */
static	double	gXGeometricCoefficient;	
static	double	gYGeometricCoefficient;
static	double	gXGeometricCenterAngle;
static	double	gYGeometricCenterAngle;

#define	kDefaultDeltaMirror .50

double gDeltaMirror = -1.0;
double gMirrorThickness = 0.050;
double gHalfMirror      = 0.025;

enum
{
	kInitializingLaserInterfaceMsg = 1,
	kMirrorDistanceNotFoundMsg,
	kMirrorDistanceInvalidMsg
};

		double		gQuarterPi;

static	double  gSqrtOfTwo;
static	double	MirrorFactor ( double x );
static	double	MirrorOffset ( double x );
								
static	void				ConvertMirrorToExternalAngles
						( double xIn, double yIn,
						double *xOut, double *yOut );

static	void				TwoPointsOnLine
						( double angleX, double angleY,
						double transformArray[12],
						double point1[3],
						double point2[3] );


double		gBinaryCenter;
double		gBinarySpanEachDirection;

uint32_t		gSlowBabies;
uint32_t		gFastBabies;
uint32_t		gSlowExponent;
uint32_t		gFastExponent;

uint32_t		gCoarseBabies;
uint32_t		gFineBabies;
uint32_t		gSuperFineBabies;

uint32_t 		gMinNeg;
uint32_t		gMaxPos;


#define kBabyStep ( 1U << gNumberOfDummyBits )

uint32_t SlowDownStep ( void )
{
	return ( gSlowBabies * kBabyStep );
}

uint32_t FastStep ( void )
{
	return ( gFastBabies * kBabyStep );
}

uint32_t CoarseSearchStep ( void )
{
	return ( gCoarseBabies * kBabyStep );
}

uint32_t FineSearchStep ( void )
{
	return ( gFineBabies * kBabyStep );
}

uint32_t SuperFineSearchStep ( void )
{
	return ( gSuperFineBabies * kBabyStep );
}


void XYFromGeometricAnglesAndZ
	( double xa, double ya, double z,
	double *x, double *y )
{
        double mf, mo;
        double t1, t2, t3, t4, t5;

        mf = MirrorFactor ( ya );
	*y = z * tan ( ya ) + gHalfMirror * mf;
	if ( fabs ( z ) > LDBL_MIN ) {
                mo = MirrorOffset ( ya );
                mf = MirrorFactor ( xa );
                t1 = tan( xa );
                t2 = 1.L / fabs ( cos ( ya ) );
                t3 = fabs ( ( gDeltaMirror - gHalfMirror * mo ) / z );
                t4 = t2 + t3;
                t5 = gHalfMirror * mf;
		// *x = z * tan ( xa ) *
                //     ( 
                //               ( 1.L / fabs ( cos ( ya ) )
                //        + fabs ( ( gDeltaMirror - gHalfMirror * mo ) / z ) )
                //     )
                //        + gHalfMirror * mf;
                *x = z * t1 * t4 + t5;
#if defined(ZDEBUG)
fprintf( stderr
       , "LI143 %lf %lf %lf %lf %lf %lf %lf %lf %lf\n"
       , *x
       , z
       , mf
       , mo
       , t1
       , t2
       , t3
       , t4
       , t5 
       );
#endif
	} else {
                mf = MirrorFactor ( xa );
		if ( z > 0.0 )
			*x = tan ( xa ) * fabs ( gDeltaMirror ) +
				gHalfMirror * mf;
		else
			*x = -tan ( xa ) * fabs ( gDeltaMirror ) +
				gHalfMirror * mf;
	}
}

uint32_t CenterBetweenLines (
	double angleX1, double angleY1, double transformArray1[12],
	double angleX2, double angleY2, double transformArray2[12],
	double centerPoint[3], double *distance )
{
	double point11[3], point12[3], point21[3], point22[3];
#if 0
	// FIXME---PAH---a1 & a2 not used?
	double a1[3], a2[3], b1[3], b2[3], bNorm, d[3];
#else
	double b1[3], b2[3], bNorm, d[3];
#endif	
	double B, D, G1, G2, k1, k2;
	
	TwoPointsOnLine ( angleX1, angleY1, transformArray1,
		point11, point12 );
	
	TwoPointsOnLine ( angleX2, angleY2, transformArray2,
		point21, point22 );
	
#if 0
	a1[0] = .50 * ( point12[0] + point11[0] );
	a1[1] = .50 * ( point12[1] + point11[1] );
	a1[2] = .50 * ( point12[2] + point11[2] );
#endif
	
	b1[0] = point12[0] - point11[0];
	b1[1] = point12[1] - point11[1];
	b1[2] = point12[2] - point11[2];
	
	bNorm = b1[0] * b1[0] + b1[1] * b1[1] + b1[2] * b1[2];
	bNorm = sqrt ( bNorm );
	if ( bNorm < LDBL_MIN ) return digiMathErr;
	b1[0] /= bNorm;
	b1[1] /= bNorm;
	b1[2] /= bNorm;
	
#if 0
	a2[0] = .50 * ( point22[0] + point21[0] );
	a2[1] = .50 * ( point22[1] + point21[1] );
	a2[2] = .50 * ( point22[2] + point21[2] );
#endif	
	b2[0] = point22[0] - point21[0];
	b2[1] = point22[1] - point21[1];
	b2[2] = point22[2] - point21[2];
	
	bNorm = b2[0] * b2[0] + b2[1] * b2[1] + b2[2] * b2[2];
	bNorm = sqrt ( bNorm );
	if ( bNorm < LDBL_MIN ) return digiMathErr;
	b2[0] /= bNorm;
	b2[1] /= bNorm;
	b2[2] /= bNorm;
	
	d[0] = .50 * ( point12[0] + point11[0] - point22[0] - point21[0] );
	d[1] = .50 * ( point12[1] + point11[1] - point22[1] - point21[1] );
	d[2] = .50 * ( point12[2] + point11[2] - point22[2] - point21[2] );
	
	/* line1: a1 + k1 * b1; line2: a2 - k2 * b2 */
	/* a1 - a2 = d */
	/* minimize the length of: d + k1 * b1 + k2 * b2 */
	
	B = b1[0] * b2[0] + b1[1] * b2[1] + b1[2] * b2[2];
	D = 1.L - B * B;
	if ( fabs ( D ) < LDBL_MIN ) return digiParLinesErr;
	
	G1 = d[0] * b1[0] + d[1] * b1[1] + d[2] * b1[2];
	G2 = d[0] * b2[0] + d[1] * b2[1] + d[2] * b2[2];
	
	k1 = ( G2 * B - G1 ) / D;
	k2 = ( G1 * B - G2 ) / D;
	
	d[0] += k1 * b1[0] + k2 * b2[0];
	d[1] += k1 * b1[1] + k2 * b2[1];
	d[2] += k1 * b1[2] + k2 * b2[2];
	
	*distance = sqrt ( d[0] * d[0] + d[1] * d[1] + d[2] * d[2] );
	
	centerPoint[0] = .50 * (
		.50 * ( point12[0] + point11[0] + point22[0] + point21[0] ) +
			k1 * b1[0] - k2 * b2[0] );
	centerPoint[1] = .50 * (
		.50 * ( point12[1] + point11[1] + point22[1] + point21[1] ) +
			k1 * b1[1] - k2 * b2[1] );
	centerPoint[2] = .50 * (
		.50 * ( point12[2] + point11[2] + point22[2] + point21[2] ) +
			k1 * b1[2] - k2 * b2[2] );
	
	return 0U;
}

void TwoPointsOnLine ( double angleX, double angleY,
	double transformArray[12],
	double point1[3], double point2[3] )
{
	transform theTransform, invTransform;
	double xa, ya, point[3];
	
	ConvertExternalAnglesToMirror ( angleX, angleY, &xa, &ya );
	ConvertMirrorToGeometricAngles ( &xa, &ya );
	
	ArrayIntoTransform ( transformArray, &theTransform );
	InvertTransform ( &theTransform, &invTransform );
	
	if ( gDeltaMirror == 0.0)
	{
		point[2] = -1.0;
		XYFromGeometricAnglesAndZ
			( xa, ya, point[2], &point[0], &point[1] );
		TransformPoint ( &invTransform, point, point1 );
		point[2] = -5.0;
		XYFromGeometricAnglesAndZ
			( xa, ya, point[2], &point[0], &point[1] );
		TransformPoint ( &invTransform, point, point2 );
	}
	else
	{
		point[2] = -10.0 * gDeltaMirror;
		XYFromGeometricAnglesAndZ
			( xa, ya, point[2], &point[0], &point[1] );
		TransformPoint ( &invTransform, point, point1 );
		point[2] = -20.0 * gDeltaMirror;
		XYFromGeometricAnglesAndZ
			( xa, ya, point[2], &point[0], &point[1] );
		TransformPoint ( &invTransform, point, point2 );
	}
}


uint32_t ZeroOnLine ( double angleX, double angleY,
	double transformArray[12], double *x, double *y )
{
	double point1[3], point2[3], dz;
	
	TwoPointsOnLine ( angleX, angleY, transformArray, point1, point2 );
	
	dz = point1[2] - point2[2];
	
	if ( fabs ( dz ) > LDBL_MIN )
	{
		*x = ( point1[2] * point2[0] - point2[2] * point1[0] ) / dz;
		*y = ( point1[2] * point2[1] - point2[2] * point1[1] ) / dz;
		return 0U;
	}
	else
	{
		if ( ( fabs ( point1[2] ) + fabs ( point2[2] ) ) > LDBL_MIN )
			return kNoIntersection;
		else
		{
			*x = ( point2[0] + point1[0] ) * 0.50;
			*y = ( point2[1] + point1[1] ) * 0.50;
			return 0U;
		}
	}
} 


void ConvertBinaryToMirrorAngles 
		( uint32_t xIn, uint32_t yIn,
		double *xOut, double *yOut )
{
	ConvertToNumber ( &xIn, &yIn );
	*xOut = gXMirrorAngularRange *
		( (double)( (int32_t)xIn >> gNumberOfDummyBits ) -
		gBinaryCenter ) / gBinarySpanEachDirection;
	*yOut = gYMirrorAngularRange *
		( (double)( (int32_t)yIn >> gNumberOfDummyBits ) -
		gBinaryCenter ) / gBinarySpanEachDirection;
}


uint32_t ConvertMirrorAnglesToBinary(double xIn, double yIn,
				     uint32_t *xOut, uint32_t *yOut)
{
  uint32_t theResult = 0;
#if PATDEBUG
  fprintf(stderr,"\nCNVRTMA2BIN: x %d, y %d", *xOut, *yOut);
#endif
  if ( xIn > gXMirrorAngularRange )
    {
      *xOut = (uint32_t)
	(rint((gBinaryCenter + gBinarySpanEachDirection)));
      theResult = kXTooLarge;
#if PATDEBUG
      fprintf(stderr, "\nCNVRTMA2BIN: x %d TOOLARGE", *xOut);
#endif
    }
  else
    {
      if (xIn < -gXMirrorAngularRange)
	{
	  *xOut = (uint32_t)
	    (rint((gBinaryCenter - gBinarySpanEachDirection)));
	  theResult = kXTooSmall;
#if PATDEBUG
	  fprintf(stderr, "\nxCNVRTMA2BIN: x %d TOOSMALL", *xOut);
#endif
	}
      else
	{
	  *xOut = (uint32_t)
	    (rint((gBinaryCenter
		   + ((xIn * gBinarySpanEachDirection) / gXMirrorAngularRange))));
#if PATDEBUG
	  fprintf(stderr, "\nCNVRTMA2BIN: x %d CORRECT", *xOut);
#endif
	}
    }

  if (yIn > gYMirrorAngularRange)
    {
      *yOut = (uint32_t)
	(rint((gBinaryCenter + gBinarySpanEachDirection)));
      theResult += kYTooLarge;
#if PATDEBUG
      fprintf(stderr, "\nCNVRTMA2BIN: y %d TOOLARGE", *yOut);
#endif
    }
  else
    {
      if (yIn < -gYMirrorAngularRange)
	{
	  *yOut = (uint32_t)
	    (rint((gBinaryCenter - gBinarySpanEachDirection)));
	  theResult += kYTooSmall;
#if PATDEBUG
	  fprintf(stderr, "\nCNVRTMA2BIN: y %d TOOSMALL", *yOut);
#endif
	}
      else
	{
	  *yOut = (uint32_t)
	    (rint((gBinaryCenter
		   + ((yIn * gBinarySpanEachDirection) / gYMirrorAngularRange))));
	}
#if PATDEBUG
      fprintf(stderr, "\nCNVRTMA2BIN: y %d CORRECT", *yOut);
#endif
    }
#if PATDEBUG
  fprintf(stderr, "\nCNVRTMA2BIN: END xOut %d, yOut %d", *xOut, *yOut);
#endif
  return theResult;
}
/*********************************************************/

void ConvertToNumber ( uint32_t *x, uint32_t *y )
{
  *x = (*x & gMaxUnsigned);
  *y = (*y & gMaxUnsigned);
}

uint32_t ConvertExternalAnglesToBinary(struct lg_master *pLgMaster,
				       double xIn, double yIn,
				       uint32_t *xOut, uint32_t *yOut)
{
	double xMirror, yMirror;
	ConvertExternalAnglesToMirror ( xIn, yIn, &xMirror, &yMirror );
	ApplyCorrection(pLgMaster, &xMirror, &yMirror);
	return ConvertMirrorAnglesToBinary ( xMirror, yMirror, xOut, yOut );
}

void ConvertExternalAnglesToMirror ( double xIn, double yIn,
		double *xOut, double *yOut )
{
	*xOut = (xIn - gXExternalCenterAngle) *	gXExternalCoefficient;
	*yOut = (yIn - gYExternalCenterAngle) *	gYExternalCoefficient;
}

void ConvertGeometricAnglesToMirror ( double *x, double *y )
{
	*x = ( *x - gXGeometricCenterAngle ) * gXGeometricCoefficient;
	*y = ( *y - gYGeometricCenterAngle ) * gYGeometricCoefficient;
}

void ConvertMirrorToGeometricAngles ( double *x, double *y )
{
	*x = *x / gXGeometricCoefficient + gXGeometricCenterAngle;
	*y = *y / gYGeometricCoefficient + gYGeometricCenterAngle;
}


uint32_t ConvertGeometricAnglesToBinary(struct lg_master *pLgMaster,
					double xIn, double yIn,
					uint32_t *xOut, uint32_t *yOut)
{
	ConvertGeometricAnglesToMirror ( &xIn, &yIn );
	ApplyCorrection(pLgMaster, &xIn, &yIn );
	return ConvertMirrorAnglesToBinary ( xIn, yIn, xOut, yOut );
}


void ConvertBinaryToGeometricAngles(struct lg_master *pLgMaster,
				    uint32_t xIn, uint32_t yIn,
				    double *xOut, double *yOut)
{
	ConvertBinaryToMirrorAngles ( xIn, yIn, xOut, yOut );
	RemoveCorrection(pLgMaster, xOut, yOut);
	ConvertMirrorToGeometricAngles ( xOut, yOut );
}

void Convert3DToExternalAngles ( double x, double y, double z,
	double *xOut, double *yOut )
{
	double xMirror, yMirror;
	GeometricAnglesFrom3D ( x, y, z, &xMirror, &yMirror );
	ConvertGeometricAnglesToMirror ( &xMirror, &yMirror );
	ConvertMirrorToExternalAngles ( xMirror, yMirror, xOut, yOut );
}

void ConvertMirrorToExternalAngles ( double xIn, double yIn,
	double *xOut, double *yOut )
{
	*xOut = (double)( xIn / gXExternalCoefficient +
		gXExternalCenterAngle );
	*yOut = (double)( yIn / gYExternalCoefficient +
		gYExternalCenterAngle );
}


void ConvertBinaryToExternalAngles(struct lg_master *pLgMaster,
				   uint32_t xIn, uint32_t yIn,
				   double *xOut, double *yOut)
{
	double xMirror, yMirror;
	ConvertBinaryToMirrorAngles ( xIn, yIn, &xMirror, &yMirror );
	RemoveCorrection(pLgMaster, &xMirror, &yMirror );
	ConvertMirrorToExternalAngles ( xMirror, yMirror, xOut, yOut );
}

double MirrorFactor ( double x )
{
	return ( 1.0 / fabs ( cos ( x * .50 - gQuarterPi ) ) - gSqrtOfTwo );
}

double MirrorOffset ( double x )
{
	return ( 1.0 / fabs ( cos ( x * .50 - gQuarterPi ) ) );
}

#define kIterations 6

void GeometricAnglesFrom3D
	( double x, double y, double z,
	double *xa, double *ya )
{
	double temp;
	short i;
        double mf, mo;

	if ( fabs ( z ) > LDBL_MIN )
	{
		*ya = atan ( y / z );
		if ( z < 0.0 ) *ya += M_PI;
		i = kIterations;
#if defined( ZTDEBUG )
fprintf( stderr, "GAF3D-538  ya %lf y %lf z %lf\n", *ya, y, z );
#endif
		while ( i-- )
		{
                        mf = MirrorFactor ( *ya );
			*ya = atan ( ( y - gHalfMirror * mf ) / z );
			if ( z < 0.0 ) *ya += M_PI;
#if defined( ZTDEBUG )
fprintf( stderr, "GAF3D-546  ya %lf mf %lf \n", *ya, mf );
#endif
		}
                mo = MirrorOffset ( *ya );
#if defined( ZTDEBUG )
fprintf( stderr, "GAF3D-551  mo %lf\n", mo );
#endif
		temp = 1.0 / fabs ( cos ( *ya ) ) +
			fabs ( ( gDeltaMirror - gHalfMirror * mo ) / z );
		*xa = atan ( ( x / z ) / temp );
		if ( z < 0.0 ) *xa += M_PI;
#if defined( ZTDEBUG )
fprintf( stderr, "GAF3D-551  xa %lf  x %lf\n", *xa, x );
#endif
		i = kIterations;
		while ( i-- )
		{
                        mf = MirrorFactor ( *xa );
			*xa = atan ( ( ( x - gHalfMirror * mf ) / z ) / temp );
			if ( z < 0.0 ) *xa += M_PI;
#if defined( ZTDEBUG )
fprintf( stderr, "GAF3D-546  xa %lf mf %lf \n", *xa, mf );
#endif
		}
/*
 *		Old stuff:
 *		temp = y / z;
 *		*ya = atan ( temp );
 *		*xa = atan ( ( x / z ) /
 *			 ( sqrt ( 1.0 + temp * temp ) +
 *			 fabs ( gDeltaMirror / z ) ) );
 *		if ( z < 0 )
 *		{
 *			*xa += M_PI;
 *			*ya += M_PI;
 *		}
 */
	}
	else
	{
                mo = MirrorOffset ( 0.0 );
		temp = fabs ( gDeltaMirror - gHalfMirror * mo );
		*xa = atan ( x / temp );
		if ( z < 0 ) *xa += M_PI;
		i = kIterations;
		while ( i-- )
		{
                        mf = MirrorFactor ( *xa );
			*xa = atan ( ( x - gHalfMirror * mf ) / temp );
			if ( z < 0 ) *xa += M_PI;
		}
		if ( fabs ( y ) > LDBL_MIN )
		{
			if ( y < 0 ) *ya = -0.50 * M_PI;
			else *ya = 0.50 * M_PI;
		}
		else
			*ya = 0.0;
	}
	return;	
}


void CloseLaserInterface ( void )
{
	CloseAngleCorrections (  );
}


void FromInchtoAPT ( double *number )
{
/*         *** we are never in local mode
**	if ( !gLocalMode )
*/
       *number *= gAPTunitsPerInch;
	return;
}


void FromAPTtoInch ( double *number )
{
/*         *** we are never in local mode
**	if ( !gLocalMode )
*/
        *number /= gAPTunitsPerInch;
	return;
}



short PilePoints
	( unsigned char fChangeBeam, unsigned char fSlowDown, double cosInOut )
{
	short pilePoints;

	/*
	return 1;
	*/
	/* kludge to avoid point piling of any sort */

	if ( fChangeBeam || ( cosInOut <= 0.0 ) )
		return gMaxPiledPts;
	else
	{
		if ( fSlowDown )
		{
			pilePoints = gMaxPiledPts -
				(short)( ( (double)gMaxPiledPts - 1 ) * cosInOut );
			if ( pilePoints < 2 ) return 2;
			if ( pilePoints < ( gMaxPiledPts >> 1 ) )
				return ( gMaxPiledPts >> 1 );
			return pilePoints;
		}
		else return 1;
	}
}

unsigned char InitLaserInterface (struct lg_master *pLgMaster)
{
	unsigned char theResult;

	gXGeometricCoefficient = ( - 180.0 / M_PI );	
	gYGeometricCoefficient = ( + 180.0 / M_PI );
	gXGeometricCenterAngle = M_PI;
	gYGeometricCenterAngle = M_PI;
	gQuarterPi = M_PI * .250;
	gSqrtOfTwo = sqrt ( 2.L );
	
        gAPTunitsPerInch = 1.0;


	gBinaryCenter = -.50;
	gBinarySpanEachDirection = (double)( 1U << 17 ) - .50;

	gMaxPiledPts = 9;
	gCoarseBabies = 1U << 3;
	gFineBabies = 1U << 2;
	gSuperFineBabies = 1U;
	gMaxPos = 0x7FFFC000U;
	gMinNeg = 0x80000000U;
	gYGeometricCoefficient = -gYGeometricCoefficient;
	
	gDeltaMirror = -1.0;
	
	theResult = (unsigned char)InitAngleCorrections (pLgMaster);
	
	if ( gDeltaMirror < 0.0 ) {
          gDeltaMirror = .50;
        }

	return theResult;
}
