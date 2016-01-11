#include <stdint.h>
/*   $Id: LaserInterface.h,v 1.11 1999/07/29 19:05:48 ags-sw Exp $  */

#ifndef __unix__
#pragma once
#endif

#ifndef LASERINTERFACE_H
#define LASERINTERFACE_H

#include "3DTransform.h"

enum
{
	digiMathErr = 1,
	digiParLinesErr
};

extern	short			PilePoints
			( unsigned char fChangeBeam, unsigned char fSlowDown,
			double cosInOut );

extern	void			SetProperBeam
					( uint32_t *x, uint32_t *y );
							
extern	void			ConvertToNumber
					( uint32_t *x, uint32_t *y );
							
extern	void			SetBeamSearch
					( uint32_t *x, uint32_t *y );
							
extern	void			DummifyNumber
					( uint32_t *x, uint32_t *y );
							
extern	void			UndummifyNumber
					( uint32_t *x, uint32_t *y );


extern	uint32_t	CenterBetweenLines (
					double angleX1, double angleY1,
					double transformArray1[12],
					double angleX2, double angleY2,
					double transformArray2[12],
					double centerPoint[3],
					double *distance );
								
extern	uint32_t	ConvertGeometricAnglesToBinary (
							double xIn,
							double yIn,
							uint32_t *xOut,
							uint32_t *yOut );

extern	uint32_t	ConvertMirrorAnglesToBinary
				( double xIn, double yIn,
				uint32_t *xOut, uint32_t *yOut );
					
extern	void			ConvertMirrorToGeometricAngles
					( double *x, double *y );
					
extern	void			ConvertBinaryToMirrorAngles 
					( uint32_t xIn, uint32_t yIn,
					double *xOut, double *yOut );

extern	void			XYFromGeometricAnglesAndZ
					( double xa, double ya,
					double z,
					double *x, double *y );

uint32_t ConvertExternalAnglesToBinary(double xIn, double yIn, uint32_t *xOut, uint32_t *yOut);
extern	void			ConvertBinaryToGeometricAngles
					( uint32_t xIn, uint32_t yIn,
					double *xOut, double *yOut );

extern	void			ConvertBinaryToExternalAngles
					( uint32_t xIn, uint32_t yIn,
					double *xOut, double *yOut );

extern	uint32_t	ZeroOnLine ( double angleX, double angleY,
					double transformArray[12],
					double *x, double *y );

extern	double		gDeltaMirror;
extern	double		gMirrorThickness;
extern	double		gHalfMirror;

extern	double		gQuarterPi;

// Function Prototypes
void CloseLaserInterface(void);
unsigned char InitLaserInterface(struct lg_master *pLgMaster);
void ConvertExternalAnglesToMirror(double xIn, double yIn, double *xOut, double *yOut);
void GeometricAnglesFrom3D(double x, double y, double z, double *xa, double *ya);
void ConvertGeometricAnglesToMirror(double *x, double *y);
void Convert3DToExternalAngles(double x, double y, double z, double *xOut, double *yOut);
							
extern	void			FromAPTtoInch ( double *number );
							
extern	void			FromInchtoAPT ( double *number );

extern	void			SetHighBeam
					( uint32_t *x, uint32_t *y );

extern	void			SetLowBeam
					( uint32_t *x, uint32_t *y );

extern	uint32_t	SlowDownStep ( void );

extern	uint32_t	FastStep ( void );

extern	uint32_t	CoarseSearchStep ( void );

extern	uint32_t	FineSearchStep ( void );

extern	uint32_t	SuperFineSearchStep ( void );

extern uint32_t           gBlankBit;
extern uint32_t           gBrightBit;
extern uint32_t           gSearchOffBit;
extern uint32_t           gSearchOnBit;
extern uint32_t           gSampleOffBit;
extern uint32_t           gMaxUnsigned;
extern double                     gBinaryCenter;
extern double                     gBinarySpanEachDirection;
extern unsigned short          gNumberOfDummyBits;

extern uint32_t           gSlowBabies;
extern uint32_t           gFastBabies;
extern uint32_t           gSlowExponent;
extern uint32_t           gFastExponent;

extern uint32_t           gCoarseBabies;
extern uint32_t           gFineBabies;
extern uint32_t           gSuperFineBabies;

extern	uint32_t	gMinNeg;
extern	uint32_t	gMaxPos;

extern	double	gBeamLinearRangeX;	
extern  double	gBeamLinearRangeY;	

extern  int gMaxPiledPts;

#endif
