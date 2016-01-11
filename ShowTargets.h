#include <stdint.h>
/*
 *  $Id$
 */

#ifndef SHOWTARGETS_H
#define SHOWTARGETS_H  1

extern void DoShowTargets(struct lg_master *pLgMaster, char * Parameters, uint32_t respondToWhom );

extern
void
Draw0( int32_t xmin
     , int32_t xmax
     , int32_t ymin
     , int32_t ymax
     , char * tmpPtr
     , int * pIndex
     )
;

extern
void
Draw1( int32_t x0
     , int32_t ymin
     , int32_t ymax
     , char * tmpPtr
     , int * pIndex
     )
;

extern
void
Draw2( int32_t xmin
     , int32_t xmax
     , int32_t ymin
     , int32_t y0
     , int32_t ymax
     , char * tmpPtr
     , int * pIndex
     )
;

extern
void
Draw3( int32_t xmin
     , int32_t xmax
     , int32_t ymin
     , int32_t y0
     , int32_t ymax
     , char * tmpPtr
     , int * pIndex
     )
;

extern
void
Draw4( int32_t xmin
     , int32_t xmax
     , int32_t ymin
     , int32_t y0
     , int32_t ymax
     , char * tmpPtr
     , int * pIndex
     )
;

extern
void
Draw5( int32_t xmin
     , int32_t xmax
     , int32_t ymin
     , int32_t y0
     , int32_t ymax
     , char * tmpPtr
     , int * pIndex
     )
;

extern
void
Draw6( int32_t xmin
     , int32_t xmax
     , int32_t ymin
     , int32_t y0
     , int32_t ymax
     , char * tmpPtr
     , int * pIndex
     )
;

extern
void
Draw7( int32_t xmin
     , int32_t xmax
     , int32_t ymin
     , int32_t ymax
     , char * tmpPtr
     , int * pIndex
     )
;

extern
void
Draw8( int32_t xmin
     , int32_t xmax
     , int32_t ymin
     , int32_t y0
     , int32_t ymax
     , char * tmpPtr
     , int * pIndex
     )
;

extern
void
Draw9( int32_t xmin
     , int32_t xmax
     , int32_t ymin
     , int32_t y0
     , int32_t ymax
     , char * tmpPtr
     , int * pIndex
     )
;

extern
void
draw_line( int32_t x0
         , int32_t y0
         , int32_t x1
         , int32_t y1
         , char * tmpPtr
         , int * pIndex
         )
;

#if 0
extern
void
show_move( unsigned int x0
         , unsigned int y0
         , unsigned int x1
         , unsigned int y1
         , char * tmpPtr
         , int * pIndex
         )
;
#endif
extern
void
draw_dark( int32_t x0
         , int32_t y0
         , int32_t x1
         , int32_t y1
         , char * tmpPtr
         , int * pIndex
         )
;

extern
void
off_pause( int32_t x1
         , int32_t y1
         , char * tmpPtr
         , int * pIndex
         )
;

extern
void
limitXY( uint32_t currentX
       , uint32_t currentY
       , int32_t * eolXNeg
       , int32_t * eolXPos
       , int32_t * eolYNeg
       , int32_t * eolYPos
       )
;
    

extern
void
limit2C( uint32_t currentX
       , uint32_t currentY
       , int32_t * eolXNeg
       , int32_t * eolX001
       , int32_t * eolX002
       , int32_t * eolXPos
       , int32_t * eolYNeg
       , int32_t * eolYPos
       )
;

#endif
