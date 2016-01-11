#include <stdint.h>
/*   $Id: LaserPattern.h,v 1.5 1999/07/29 19:03:59 ags-sw Exp $  */

#ifndef __unix__
#pragma once
#endif

extern	void			CloseLaserPattern ( void );

int InitLaserPattern(void);

extern	uint32_t	PendPenDown ( void );

extern	uint32_t	UnpendPenDown ( void );

extern	uint32_t	PutGoTo2D (struct lg_master *pLgMaster, double x, double y );

extern	uint32_t	PutGoTo3D (struct lg_master *pLgMaster, double x, double y, double z);
					
extern	uint32_t	FinishPattern (struct lg_master *pLgMaster);

extern	uint32_t	SetPenDown ( void );

extern	uint32_t	SetPenUp ( void );

extern	uint32_t	SetDefaultZ ( double z );

void SetUpLaserPattern(struct lg_master *pLgMaster, double *transform);
void ChangeTransform(double *transform);

uint32_t Transform3DPointToBinary(double x, double y, double z,
				  uint32_t *xAngle, uint32_t *yAngle);
uint32_t PointToBinary(double *point, uint32_t *xAngle,	uint32_t *yAngle);
extern double gMaxCos;

extern  double gCurveMin;
extern  double gCurveMax;

extern  double gLongToShort;

extern uint32_t           gTransmittedLengthSum;

