#include <stdint.h>
/*   $Id: SensorSearch.h,v 1.12 1999/07/29 18:36:01 ags-sw Exp $  */

#ifndef SENSORSEARCH_H
#define SENSORSEARCH_H

#define kStopWasDone            -1
#define kCoarseNotFound         -2
#define kFineNotFound           -3
#define kXtooSmallToSearch      -4
#define kXtooLargeToSearch      -5
#define kYtooSmallToSearch      -6
#define kYtooLargeToSearch      -7
#define kSuperFineNotFound      -8

#define kNumberDrift   12

int SearchForASensor(struct lg_master *pLgMaster, int32_t startX, int32_t startY,
		     int32_t *foundX, int32_t *foundY );

int DoSearch(int32_t startX, int32_t startY, int32_t *foundX, int32_t *foundY);

int QuickCheckASensor(struct lg_master *pLgMaster, int32_t centerX, int32_t centerY);

int QuickCheckOne(struct lg_master *pLgMaster, int32_t centerX, int32_t centerY,
		  int32_t *foundX, int32_t *foundY);
int DoRedundandSuperFineSearch(int32_t *foundX, int32_t *foundY);
int DoSuperFineSearch(int32_t *foundX, int32_t *foundY, int numberOfSuperCrosses);
int DoFineSearch(int32_t *foundX, int32_t *foundY);

extern int32_t   gQCerrors[6];

extern double xPosAvg;
extern double xNegAvg;
extern double yPosAvg;
extern double yNegAvg;

extern int32_t * xPosSave;
extern int32_t * xNegSave;
extern int32_t * yPosSave;
extern int32_t * yNegSave;

extern int32_t * xPosEdge;
extern int32_t * xNegEdge;
extern int32_t * yPosEdge;
extern int32_t * yNegEdge;

extern int32_t * xPosSuper;
extern int32_t * xNegSuper;
extern int32_t * yPosSuper;
extern int32_t * yNegSuper;

extern int32_t * xPosESuper;
extern int32_t * xNegESuper;
extern int32_t * yPosESuper;
extern int32_t * yNegESuper;

extern int32_t * xPosPosition;
extern int32_t * yPosPosition;
extern int32_t * xNegPosition;
extern int32_t * yNegPosition;

extern double * xResArr;
extern double * yResArr;

extern double g_aX;
extern double g_bX;
extern double g_aY;
extern double g_bY;

extern double gFixX[ kNumberDrift ];
extern double gFixY[ kNumberDrift ];
extern double gDriftX[ kNumberDrift ];
extern double gDriftY[ kNumberDrift ];

void InitDrift(int32_t *Xarr, int32_t *Yarr);
void FindDrift(int32_t cX, int32_t cY, int32_t fX, int32_t fY);
void CorrectDrift(int32_t cX, int32_t cY, int32_t *fX, int32_t *fY );
void SetSuperFineFactor ( uint32_t n ) ;

extern int gTargetDrift;

extern uint32_t gCoarseSearchStep;
extern int           gCoarseFactor;
extern int           gCoarse2Factor;
extern int  gNumberOfSpirals;
extern int  gSpiralFactor;
extern int  gHatchFactor;
extern int  gSuperFineCount;
extern int  gSuperFineSkip;
extern uint32_t gSuperFineFactor;
extern int32_t gDwell;

extern void InitSensorSearch ( void );

void CloseSensorSearch ( void );

extern int  gCentroid;

extern int  gNoFine;

extern int  gMaxQuickSearches;

extern int  LongOrShortThrowSearch;

extern int  gNumberOfSensorSearchAttempts;

extern int32_t   gLoutCount;
extern int32_t   gLoutSize;
extern char * gLoutBase;
extern char * gLoutPtr;

extern int gMultipleSweeps;

extern int32_t   gDELLEV;

#endif
