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

extern	int SearchForASensor(struct lg_master *pLgMaster,
			     uint32_t startX, uint32_t startY,
			     uint32_t *foundX, uint32_t *foundY );

extern	int	DoSearch (
				uint32_t startX, uint32_t startY,
				uint32_t *foundX, uint32_t *foundY );

extern int QuickCheckASensor(struct lg_master *pLgMaster, uint32_t centerX, uint32_t centerY);

extern int QuickCheckOne(struct lg_master *pLgMaster,
			 uint32_t centerX, 
			 uint32_t centerY,
			 uint32_t *foundX, 
			 uint32_t *foundY);
extern int    DoRedundandSuperFineSearch
        ( uint32_t *foundX, uint32_t *foundY );


extern int     DoSuperFineSearch
                           ( uint32_t *foundX, uint32_t *foundY,
                                int numberOfSuperCrosses );

extern int     DoFineSearch
                        ( uint32_t *foundX, uint32_t *foundY );

extern int32_t   gQCerrors[6];

extern double xPosAvg;
extern double xNegAvg;
extern double yPosAvg;
extern double yNegAvg;

extern uint32_t * xPosSave;
extern uint32_t * xNegSave;
extern uint32_t * yPosSave;
extern uint32_t * yNegSave;

extern uint32_t * xPosEdge;
extern uint32_t * xNegEdge;
extern uint32_t * yPosEdge;
extern uint32_t * yNegEdge;

extern uint32_t * xPosSuper;
extern uint32_t * xNegSuper;
extern uint32_t * yPosSuper;
extern uint32_t * yNegSuper;

extern uint32_t * xPosESuper;
extern uint32_t * xNegESuper;
extern uint32_t * yPosESuper;
extern uint32_t * yNegESuper;

extern uint32_t * xPosPosition;
extern uint32_t * yPosPosition;
extern uint32_t * xNegPosition;
extern uint32_t * yNegPosition;

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

extern void InitDrift( uint32_t *Xarr, uint32_t *Yarr);

extern void FindDrift( uint32_t cX,
              uint32_t cY,
              uint32_t fX,
              uint32_t fY );

extern void CorrectDrift( uint32_t cX,
                   uint32_t cY,
                   uint32_t *fX,
                   uint32_t *fY );

extern void SetSuperFineFactor ( uint32_t n ) ;

extern int gTargetDrift;

extern uint32_t gCoarseSearchStep;
extern int           gCoarseFactor;
extern uint32_t gCoarse2SearchStep;
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
