#ifndef LEDTRNSFORM_H
#define LEDTRNSFORM_H
/*   $Id: L3DTransform.h,v 1.1 1999/08/17 20:17:57 ags-sw Exp $   */

typedef struct { double oldLoc[3], xRad, yRad; }
        doubleInputPoint;

typedef struct { double rotMatrix[3][3], transVector[3]; }
        doubleTransform;

extern  unsigned char   FindBestTransform
                        ( doubleInputPoint iPt[4]
                        , doubleTransform *tr
                        , double deltaXHeight
                        , double tolerance
                        , double * bestCosine
                        );

extern double minL3Distance;

extern unsigned char gBestTargetArray[128];
extern int32_t gBestTargetNumber;

#endif // LEDTRNSFORM_H
