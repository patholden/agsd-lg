#include <stdint.h>
/*
 * $Id: amoeba.h,v 1.1 1999/03/19 18:39:25 ags-sw Exp $
 */

extern void amoeba(double **p, double *y, int ndim, double ftol,
            double (*funk)(double []), int *nfunk);
