#include <stdint.h>
#include <math.h>
#include <stdio.h>

#define MINANGSEP  0.1

#include "shoelace4.h"

double
shoelace4( double *x, double *y )
{
   int j,i;
   double a, dx, dy, mag;
   // first make sure that no two points are too close

    for ( i=0; i<3; i++ ) {
      for ( j=i+1; j<4; j++ ) {
        dx = x[i] - x[j];
        dy = y[i] - y[j];
        mag = sqrt( dx*dx + dy*dy );
#ifdef XDEBUG
printf( "mag %lf\n", mag );
#endif
        if ( mag < MINANGSEP ) {
#ifdef YDEBUG
           printf("too close (%lf, %lf) (%lf, %lf)\n", x[i], y[i], x[j], y[j]);
#endif
           return( 0 );
        }
      }
    }

    a = 0.5 * (  x[0]*y[1] 
               + x[1]*y[2]
               + x[2]*y[3]
               + x[3]*y[0]
               - x[1]*y[0]
               - x[2]*y[1]
               - x[3]*y[2]
               - x[0]*y[3]
               );

    if ( a < 0 ) { a = -a; }

   return( a );
}
