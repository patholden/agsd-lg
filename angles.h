#include <stdint.h>
/*
 * $Id: angles.h,v 1.2 1999/07/29 18:25:40 ags-sw Exp $
 */

extern int TransformtoRPY (transform *m,
                           double *roll,
                           double *pitch, 
		           double *yaw,
                           double *x_trans,
                           double *y_trans, 
		           double *z_trans);
	
extern int TransformfromRPY (double roll,
                             double pitch,
                             double yaw,
		             double x_trans, 
                             double y_trans, 
                             double z_trans,
		             transform *tr);
