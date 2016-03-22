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
#include <syslog.h>
#include "BoardComm.h"
#include "AppCommon.h"
#include "AppErrors.h"
#include "AppStrListIDs.h"
#include "LaserInterface.h"
#include "3DTransform.h"
#include "AngleCorrections.h"


int gMaxPiledPts;

static	double	gAPTunitsPerInch	= 1.0;
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
//#define kBinaryCenter	   -.50
#define kBinaryCenter	   0.50
#define kXMirrorAngularRange  30.0	
#define kYMirrorAngularRange  30.0	
double gDeltaMirror = -1.0;
double gMirrorThickness = 0.050;
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

static double		gBinarySpanEachDirection;

uint32_t		gSlowBabies;
uint32_t		gFastBabies;
uint32_t		gSlowExponent;
uint32_t		gFastExponent;

uint32_t		gCoarseBabies;
uint32_t		gFineBabies;
uint32_t		gSuperFineBabies;


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


void XYFromGeometricAnglesAndZ(struct lg_master *pLgMaster, double xa, double ya,
			       double z,double *x, double *y)
{
        double mf, mo;
        double t1, t2, t3, t4, t5;

        mf = MirrorFactor ( ya );
	*y = z * tan ( ya ) + pLgMaster->gHalfMirror * mf;
	if ( fabs ( z ) > LDBL_MIN ) {
                mo = MirrorOffset(ya);
                mf = MirrorFactor(xa);
                t1 = tan(xa);
                t2 = 1.L / fabs(cos(ya));
                t3 = fabs ((gDeltaMirror - (pLgMaster->gHalfMirror * mo)) / z);
                t4 = t2 + t3;
                t5 = pLgMaster->gHalfMirror * mf;
                *x = z * t1 * t4 + t5;
	} else {
                mf = MirrorFactor(xa);
		if ( z > 0.0 )
		  *x = (tan(xa) * fabs(gDeltaMirror)) +
		    (pLgMaster->gHalfMirror * mf);
		else
		  *x = (-tan(xa) * fabs(gDeltaMirror)) +
		    (pLgMaster->gHalfMirror * mf);
	}
}
void ConvertBinaryToMirrorAngles(int16_t xIn, int16_t yIn, double *xOut, double *yOut)
{
    if (xIn < 0)
      *xOut = (double)(xIn * kXMirrorAngularRange / gBinarySpanEachDirection) - kBinaryCenter;
    else
      *xOut = (double)(xIn * kXMirrorAngularRange / gBinarySpanEachDirection) + kBinaryCenter;
    if (yIn < 0)
      *yOut = (double)(yIn * kYMirrorAngularRange / gBinarySpanEachDirection) - kBinaryCenter;
    else
      *yOut = (double)(yIn * kYMirrorAngularRange / gBinarySpanEachDirection) - kBinaryCenter;
#ifdef AGS_DEBUG
    syslog(LOG_DEBUG,"CNVRTBINTOMIRANGL: x=%f, y=%f,span%f",*xOut, *yOut,gBinarySpanEachDirection);
#endif
    return;
}
uint32_t ConvertMirrorAnglesToBinary(double xIn, double yIn,
				     int16_t *xOut, int16_t *yOut)
{
  uint32_t theResult = 0;
  
  if ( xIn >kXMirrorAngularRange )
    {
      *xOut = (int16_t)gBinarySpanEachDirection;
      theResult = kXTooLarge;
    }
  else
    {
      if (xIn < -kXMirrorAngularRange)
	{
	  *xOut = (int16_t)-gBinarySpanEachDirection;
	  theResult = kXTooSmall;
	}
      else
	{
	  (xIn > 0) ? (xIn += kBinaryCenter) : (xIn -= kBinaryCenter);
	  *xOut = (int16_t)(xIn * gBinarySpanEachDirection / kXMirrorAngularRange);
	}
    }

  if (yIn > kYMirrorAngularRange)
    {
      *yOut = (int16_t)gBinarySpanEachDirection;
      theResult += kYTooLarge;
    }
  else
    {
      if (yIn < -kYMirrorAngularRange)
	{
	  *yOut = (int16_t)-gBinarySpanEachDirection;
	  theResult += kYTooSmall;
	}
      else
	{
	  (yIn > 0) ? (yIn += kBinaryCenter) : (yIn -= kBinaryCenter);
	  *yOut = (int16_t)(yIn * gBinarySpanEachDirection / kYMirrorAngularRange);
	}
    }
#ifdef AGS_DEBUG
  syslog(LOG_DEBUG,"MIRANGL2BIN: x=%x,y=%x,span=%f",*xOut,*yOut,gBinarySpanEachDirection);
#endif
  return(theResult);
}
/*********************************************************/

uint32_t ConvertExternalAnglesToBinary(struct lg_master *pLgMaster,
				       double xIn, double yIn,
				       int16_t *xOut, int16_t *yOut)
{
    double xMirror=0, yMirror=0;
    
    ConvertExternalAnglesToMirror ( xIn, yIn, &xMirror, &yMirror );
    ApplyCorrection(pLgMaster, &xMirror, &yMirror);
    return(ConvertMirrorAnglesToBinary(xMirror, yMirror, xOut, yOut));
}
void ConvertExternalAnglesToMirror ( double xIn, double yIn,
		double *xOut, double *yOut )
{
    *xOut = (xIn - gXExternalCenterAngle) *	gXExternalCoefficient;
    *yOut = (yIn - gYExternalCenterAngle) *	gYExternalCoefficient;
    return;
}
void ConvertGeometricAnglesToMirror ( double *x, double *y )
{
    *x = ( *x - gXGeometricCenterAngle ) * gXGeometricCoefficient;
    *y = ( *y - gYGeometricCenterAngle ) * gYGeometricCoefficient;
    return;
}
void ConvertMirrorToGeometricAngles ( double *x, double *y )
{
    *x = *x / gXGeometricCoefficient + gXGeometricCenterAngle;
    *y = *y / gYGeometricCoefficient + gYGeometricCenterAngle;
    return;
}
uint32_t ConvertGeometricAnglesToBinary(struct lg_master *pLgMaster,
					double xIn, double yIn,
					int16_t *xOut, int16_t *yOut)
{
    ConvertGeometricAnglesToMirror ( &xIn, &yIn );
    ApplyCorrection(pLgMaster, &xIn, &yIn );
    return(ConvertMirrorAnglesToBinary(xIn, yIn, xOut, yOut));
}
void ConvertBinaryToGeometricAngles(struct lg_master *pLgMaster,
				    int16_t xIn, int16_t yIn,
				    double *xOut, double *yOut)
{
    ConvertBinaryToMirrorAngles ( xIn, yIn, xOut, yOut );
    RemoveCorrection(pLgMaster, xOut, yOut);
    ConvertMirrorToGeometricAngles ( xOut, yOut );
    return;
}
void Convert3DToExternalAngles(struct lg_master *pLgMaster, double x, double y, double z,
			       double *xOut, double *yOut)
{
    double xMirror, yMirror;
    GeometricAnglesFrom3D(pLgMaster, x, y, z, &xMirror, &yMirror );
    ConvertGeometricAnglesToMirror ( &xMirror, &yMirror );
    ConvertMirrorToExternalAngles ( xMirror, yMirror, xOut, yOut );
    return;
}
void ConvertMirrorToExternalAngles ( double xIn, double yIn,
	double *xOut, double *yOut )
{
    *xOut = (double)( xIn /(gXExternalCoefficient +
			    gXExternalCenterAngle));
    *yOut = (double)( yIn /(gYExternalCoefficient +
			    gYExternalCenterAngle));
    return;
}
void ConvertBinaryToExternalAngles(struct lg_master *pLgMaster,
				   int16_t xIn, int16_t yIn,
				   double *xOut, double *yOut)
{
    double xMirror, yMirror;
    ConvertBinaryToMirrorAngles(xIn, yIn, &xMirror, &yMirror);
    RemoveCorrection(pLgMaster, &xMirror, &yMirror);
    ConvertMirrorToExternalAngles(xMirror, yMirror, xOut, yOut);
    return;
}
double MirrorFactor ( double x )
{
    return(1.0 / fabs ( cos ( x * .50 - gQuarterPi ) ) - gSqrtOfTwo );
}
double MirrorOffset ( double x )
{
    return( 1.0 / fabs ( cos ( x * .50 - gQuarterPi ) ) );
}
#define kIterations 6
void GeometricAnglesFrom3D(struct lg_master *pLgMaster, double x, double y,
			   double z, double *xa, double *ya)
{
	double temp;
	short i;
        double mf, mo;

	if (fabs(z) > LDBL_MIN)
	{
		*ya = atan(y / z);
		if ( z < 0.0 ) *ya += M_PI;
		i = kIterations;
		while ( i-- )
		{
                        mf = MirrorFactor ( *ya );
			*ya = atan ( ( y - (pLgMaster->gHalfMirror * mf)) / z );
			if ( z < 0.0 ) *ya += M_PI;
		}
                mo = MirrorOffset ( *ya );
		temp = 1.0 / fabs ( cos ( *ya ) ) +
		  fabs ( ( gDeltaMirror - (pLgMaster->gHalfMirror * mo)) / z );
		*xa = atan ( ( x / z ) / temp );
		if ( z < 0.0 ) *xa += M_PI;
		i = kIterations;
		while ( i-- )
		{
                        mf = MirrorFactor ( *xa );
			*xa = atan ( ( ( x - (pLgMaster->gHalfMirror * mf)) / z ) / temp );
			if ( z < 0.0 ) *xa += M_PI;
		}
	}
	else
	{
                mo = MirrorOffset ( 0.0 );
		temp = fabs ( gDeltaMirror - (pLgMaster->gHalfMirror * mo));
		*xa = atan ( x / temp );
		if ( z < 0 ) *xa += M_PI;
		i = kIterations;
		while ( i-- )
		{
                        mf = MirrorFactor ( *xa );
			*xa = atan ( ( x - (pLgMaster->gHalfMirror * mf)) / temp );
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
	gBinarySpanEachDirection = (double)(1 << 15) - .50;
	gMaxPiledPts = 9;
	gCoarseBabies = 1U << 3;
	gFineBabies = 1U << 2;
	gSuperFineBabies = 1U;
	gYGeometricCoefficient = -gYGeometricCoefficient;
	gDeltaMirror = -1.0;
	theResult = (unsigned char)InitAngleCorrections (pLgMaster);
	if ( gDeltaMirror < 0.0 ) {
          gDeltaMirror = .50;
        }

	return theResult;
}
