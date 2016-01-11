#include <stdint.h>
/*   $Id: AngleCorrections.h,v 1.4 1999/07/29 19:56:56 ags-sw Exp $  */

#ifndef __unix__
#pragma once
#endif

extern	void		RemoveCorrection
						( double *x, double *y );
					
extern	void		ApplyCorrection
						( double *x, double *y );


extern	unsigned char InitAngleCorrections (struct lg_master *pLgMaster);
					
extern	void		CloseAngleCorrections ( void );

extern	unsigned char		gCALIBFileOK;
