#include <stdint.h>
/*   $Id: SensorRegistration.h,v 1.7 2007/04/02 08:43:56 pickle Exp pickle $  */

#ifndef __unix__
#pragma once
#endif

#ifndef SENSORREGISTRATION_H
#define SENSORREGISTRATION_H

#define  MAXTRANSNUM  11000


extern	unsigned char	FindTransformMatrix ( short numberOfPoints, 
					double deltaMirror, double tolerance,
					double *foundAngles, double *theTransform );

extern	void	SaveFullRegCoordinates ( short numberOfPoints,
					double *theOriginalCoordinates );
					
extern	void	DontFindTransform ( unsigned char doOrDont );

extern short GnOfTrans;
extern int   gSaved;
extern double gX[kNumberOfFlexPoints];
extern double gY[kNumberOfFlexPoints];
extern double gZ[kNumberOfFlexPoints];
extern double gXfoundAngle[kNumberOfFlexPoints];
extern double gYfoundAngle[kNumberOfFlexPoints];
extern double chiX[kNumberOfFlexPoints];
extern double chiY[kNumberOfFlexPoints];
extern double chiZ[kNumberOfFlexPoints];
extern double chiXfoundAngle[kNumberOfFlexPoints];
extern double chiYfoundAngle[kNumberOfFlexPoints];

extern int32_t gColinear[kNumberOfFlexPoints];
extern int32_t gCoplanar[kNumberOfFlexPoints];
extern int32_t gCoplanarCount;


extern int  foundTarget[kNumberOfFlexPoints];

extern int  gNPoints;

extern double gDiffX[MAXTRANSNUM];
extern double gDiffY[MAXTRANSNUM];

extern int savePoint[MAXTRANSNUM];

extern double gWorstTolReg;
extern double gWorstTolAll;
extern double gBestTolAll;

#endif
