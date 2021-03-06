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
#define kSuperFineTooFew        -9
#define kNumberDrift   12
#define kMaxQuickSearches       3
#define SENSE_BUF_SIZE          sizeof(int16_t) * 500000

int QuickCheckASensor(struct lg_master *pLgMaster, int16_t centerX, int16_t centerY);
int DoSearch(int16_t startX, int16_t startY, int16_t *foundX, int16_t *foundY);

int QuickCheckASensor(struct lg_master *pLgMaster, int16_t centerX, int16_t centerY);

int QuickCheckOne(struct lg_master *pLgMaster, int16_t centerX, int16_t centerY,
		  int16_t *foundX, int16_t *foundY);
int DoRedundandSuperFineSearch(int16_t *foundX, int16_t *foundY);
int DoSuperFineSearch(int16_t *foundX, int16_t *foundY, int numberOfSuperCrosses);
int DoFineSearch(int16_t *foundX, int16_t *foundY);
void InitDrift(int16_t *Xarr, int16_t *Yarr);
void FindDrift(int16_t cX, int16_t cY, int16_t fX, int16_t fY);
void CorrectDrift(int16_t cX, int16_t cY, int16_t *fX, int16_t *fY );
void SetSuperFineFactor(uint32_t n);
void InitSensorSearch(void);
void CloseSensorSearch(void);
void SensorInitLog(void);
int SearchForASensor (struct lg_master *pLgMaster, int16_t startX, int16_t startY,
		      int16_t *foundX, int16_t *foundY);
void ClearSensorBuffers(void);
uint32_t getSuperFineFactor(void);

extern int gTargetDrift;

extern uint32_t gCoarseSearchStep;
extern int  gSuperFineCount;
extern int  gSuperFineSkip;
extern int  gNoFine;
extern int  gMaxQuickSearches;
extern int  gNumberOfSensorSearchAttempts;

extern int32_t   gLoutCount;
extern int32_t   gLoutSize;
extern char * gLoutBase;
#endif
