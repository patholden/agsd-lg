#include <stdint.h>
/*   $Id: LaserInterface.h,v 1.11 1999/07/29 19:05:48 ags-sw Exp $  */

#ifndef __unix__
#pragma once
#endif

#ifndef LASERINTERFACE_H
#define LASERINTERFACE_H

#include "3DTransform.h"

/******************************
 Defines
******************************/
#define gNumberOfDummyBits 14

enum
{
	digiMathErr = 1,
	digiParLinesErr
};

extern	double		gDeltaMirror;
extern	double		gMirrorThickness;
extern	double		gHalfMirror;

extern	double		gQuarterPi;

extern double                     gBinaryCenter;
extern double                     gBinarySpanEachDirection;

extern uint32_t           gSlowBabies;
extern uint32_t           gFastBabies;
extern uint32_t           gSlowExponent;
extern uint32_t           gFastExponent;

extern uint32_t           gCoarseBabies;
extern uint32_t           gFineBabies;
extern uint32_t           gSuperFineBabies;

extern	double	gBeamLinearRangeX;	
extern  double	gBeamLinearRangeY;	

extern  int gMaxPiledPts;

// Function Prototypes
short PilePoints(unsigned char fChangeBeam, unsigned char fSlowDown,
		 double cosInOut );
void ConvertToNumber(int32_t *x, int32_t *y);
uint32_t CenterBetweenLines(double angleX1, double angleY1,
			    double transformArray1[12],
			    double angleX2, double angleY2,
			    double transformArray2[12],
			    double centerPoint[3],
			    double *distance );
uint32_t ConvertGeometricAnglesToBinary(struct lg_master *pLgMaster,
					double xIn,double yIn,
					int32_t *xOut,	int32_t *yOut );
uint32_t ConvertMirrorAnglesToBinary( double xIn, double yIn,
				int32_t *xOut, int32_t *yOut );
void ConvertMirrorToGeometricAngles(double *x, double *y);
void ConvertBinaryToMirrorAngles(int32_t xIn, int32_t yIn,
				 double *xOut, double *yOut );
void XYFromGeometricAnglesAndZ(double xa, double ya, double z,
			       double *x, double *y);
uint32_t ConvertExternalAnglesToBinary(struct lg_master *pLgMaster,
				       double xIn, double yIn,
				       int32_t *xOut, int32_t *yOut);
void ConvertBinaryToGeometricAngles(struct lg_master *pLgMaster,
				    int32_t xIn, int32_t yIn,
				    double *xOut, double *yOut );
void ConvertBinaryToExternalAngles(struct lg_master *pLgMaster,
				   int32_t xIn, int32_t yIn,
				   double *xOut, double *yOut);
uint32_t ZeroOnLine(double angleX, double angleY, double transformArray[12],
		    double *x, double *y );
void CloseLaserInterface(void);
unsigned char InitLaserInterface(struct lg_master *pLgMaster);
void ConvertExternalAnglesToMirror(double xIn, double yIn, double *xOut, double *yOut);
void GeometricAnglesFrom3D(double x, double y, double z, double *xa, double *ya);
void ConvertGeometricAnglesToMirror(double *x, double *y);
void Convert3DToExternalAngles(double x, double y, double z, double *xOut, double *yOut);
void FromAPTtoInch(double *number);
void FromInchtoAPT(double *number);
uint32_t SlowDownStep(void);
uint32_t FastStep(void);
uint32_t CoarseSearchStep(void);
uint32_t FineSearchStep(void);
uint32_t SuperFineSearchStep(void);
#endif
