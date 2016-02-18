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

uint32_t SetPenDown ( void );
uint32_t SetPenUp ( void );
uint32_t SetDefaultZ ( double z );
struct lg_xydata *SetUpLaserPattern(struct lg_master *pLgMaster, double *transform);
void ChangeTransform(double *transform);
uint32_t Transform3DPointToBinary(struct lg_master *pLgMaster, double x, double y, double z,
				  int32_t *xAngle, int32_t *yAngle);
uint32_t PointToBinary(struct lg_master *pLgMaster, double *point, int32_t *xAngle, int32_t *yAngle);
extern double gMaxCos;

extern  double gCurveMin;
extern  double gCurveMax;

extern  double gLongToShort;

extern uint32_t           gTransmittedLengthSum;

