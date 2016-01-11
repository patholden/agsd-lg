#include <stdint.h>

#include <stdio.h>
#include <stdlib.h>

#include "allace4.h"
#include "permutes.h"
#include "L3DTransform.h"
#include "area_order.h"
#include "heap.h"

void
area_order( int  numin
          , doubleInputPoint *inPt
          , int *num_out
          , permutes *Combos
          , int32_t * reversedex
          , int * found
          )
{
    double x[4], y[4];
    int count, i, j, k, l;
    double * area_array;
    int32_t   * area_index;
    double area;
    int ncombo;

    area_array = (double *)calloc( 200000, sizeof(double) );
    area_index = (int32_t *)calloc( 200000, sizeof(int32_t) );

    count = 1;
    for ( i=0; i < numin; i++ ) {
      if ( found[i] == 0 ) continue;
      for ( j=i; j < numin; j++ ) {
        if ( found[j] == 0 ) continue;
        for ( k=j; k < numin; k++ ) {
          if ( found[k] == 0 ) continue;
          for ( l=k; l < numin; l++ ) {
            if ( found[l] == 0 ) continue;
            if ( i!=j && i!=k && i!=l && j!=k && j!=l && k!=l ) {
              Combos[count].index1 = i;
              Combos[count].index2 = j;
              Combos[count].index3 = k;
              Combos[count].index4 = l;
              x[0] =  inPt[i].xRad; y[0] =  inPt[i].yRad;
              x[1] =  inPt[j].xRad; y[1] =  inPt[j].yRad;
              x[2] =  inPt[k].xRad; y[2] =  inPt[k].yRad;
              x[3] =  inPt[l].xRad; y[3] =  inPt[l].yRad;
              area = allace4( x, y );
              Combos[count].area = area;
              area_array[count] = area;
              area_index[count] = count;
              count++;
            }
          }
        }
      }
    }
    ncombo = count - 1;

    heap( ncombo, area_array, area_index );

    for ( i=1; i <= ncombo; i++ ) {
         reversedex[1+ncombo-i] = area_index[i];
    }
    *num_out = ncombo;

#ifdef YDEBUG
    for ( i=1; i <= ncombo; i++ ) {
       printf( "%5d %12.4lf %12.4lf %2d %2d %2d %2d\n"
             , i
             , area_array[reversedex[i]]
             , Combos[reversedex[i]].area
             , Combos[reversedex[i]].index1
             , Combos[reversedex[i]].index2
             , Combos[reversedex[i]].index3
             , Combos[reversedex[i]].index4
             );
    }
#endif

}
