#ifndef BOARDCOMM_H
#define BOARDCOMM_H
/*   $Id: BoardComm.h,v 1.20 2001/01/03 17:48:53 ags-sw Exp pickle $  */

#ifndef __unix__
#pragma once
#endif

#define kMaxUnsigned  0xFFFFC000U
#define STEPDELAY   500
#define kSensorBit                      0x40    /*  edge trigger */
#define kCDRHbit                        0x08    /*  high when plugged in */
#define kINTERLOCKbit                   0x02    /*  high scanners fault */
#define kBaseSlot24bit    0x00000000U
#define kOffsetSlot24bit  0x00100000U
#define kBaseSlot32bit    0xF0000000U
#define kOffsetSlot32bit  0x01000000U
#define kDefaultFirstSlot  9
#define kDefaultSlotCount  6
#define  kMaxMissedInQuickCheck      3
#define  kMaxMissesOnGoodSensor      2
#define  kStopGoResponseTimeout      300L
#define  kLEDon      0x00
#define  kLEDoff     0x10
#define  kSEARCHoff      0x00
#define  kSEARCHon       0x40

extern  uint32_t  gRespondToWhom;

extern  int gSearchFlag;
extern  int gStopFlag;
struct displayData
{
  uint32_t	*sensorAngles;
  uint16_t	numberOfSensorSets;
};

// The following header is fixed for incoming commands from
// host PC
struct cmd_inhdr {
  unsigned char theCommand;
  unsigned char specialByte;
  uint16_t      seqNo;
};
struct k_header {
  union {
    struct {
      unsigned char cmd;
      unsigned char flags;
      uint16_t      seq_num;
    };
    struct {
      unsigned char status;
      unsigned char fill;
      uint16_t      errtype;
    };
    struct {
      unsigned char status1;
      unsigned char errtype1;
      uint16_t      errtype2;
    };
  };
} __attribute__ ((packed));
struct hobbs_ctrs {
  time_t    hobbs_counter;
  time_t    xscanner_counter;
  time_t    yscanner_counter;
  time_t    laser_counter;
};
struct lg_master {
  struct sockaddr_in webhost_addr;
  struct hobbs_ctrs hobbs;
  struct k_header gOutOfRange;
  char            webhost[128];
  double          gArgTol;
  double          ping_count;
  double          gTolerance;
  double          dmax;
  unsigned char   *gInputBuffer;
  unsigned char   *gDataChunksBuffer;
  unsigned char   *gSensorBuffer;
  unsigned char   *gAFInputBuffer;
  unsigned char   *gRawBuffer;
  unsigned char   *gParametersBuffer;
  unsigned char   *theResponseBuffer;
  int             af_serial;
  int             socketfd;
  int             datafd;
  int             fd_laser;
  int             serial_ether_flag;
  unsigned long   gProjectorSerialNumber;
  uint16_t        seqNo;
  unsigned char   newCommand;
  unsigned char   gHeaderSpecialByte;
  uint32_t        gHEX;
  uint32_t        enet_retry_count;
  uint32_t        gotA1;
  uint32_t        gDataChunksLength;
  uint32_t        patternLength;
  uint32_t        optic_status;
  uint32_t        gPeriod;
  uint32_t        gDisplayFlag;
  uint32_t        gSrchStpPeriod;
  int32_t         gQCcount;
};

void	RandomSegment ( void );
int32_t InitQCtimer( void );
int32_t GetQCtimer( void );
void SetROIsearch( void );
void ClearROIsearch( void );
void PostCommand(struct lg_master *pLgMaster, uint32_t theCommand, char * data,
		 uint32_t respondToWhom);
int IfStopThenStopAndNeg1Else0 (struct lg_master *pLgMaster);
int doWriteDevCmdNoData(struct lg_master *pLgMaster, uint32_t command);
int doWriteDevCmd32(struct lg_master *pLgMaster, uint32_t command, uint32_t write_val);
int doWriteDevDelta(struct lg_master *pLgMaster, uint32_t xval, uint32_t yval);
int doWriteDevPoints(struct lg_master *pLgMaster, uint32_t xval, uint32_t yval);
void StopPulse(struct lg_master *pLgMaster);
int CDRHflag(struct lg_master *pLgMaster);
int SetQCcounter(struct lg_master *pLgMaster, int count);
int initQCcounter(struct lg_master *pLgMaster);
int stopQCcounter(struct lg_master *pLgMaster);
int resetQCcounter(struct lg_master *pLgMaster);
int SearchBeamOff(struct lg_master *pLgMaster);
int SearchBeamOn(struct lg_master *pLgMaster);
int InitBoard(struct lg_master *pLgMaster);
void ReleaseBoard (struct lg_master *pLgMaster);
void SlowDownAndStop(struct lg_master *pLgMaster);
void JustDoDisplay(struct lg_master *pLgMaster, char * wr_ptr, int patternLength );
void ClearLinkLED(struct lg_master *pLgMaster);
void FlashLed(struct lg_master *pLgMaster, int numFlash);         
void SaveBeamPosition( char * data );
int setROIlength(struct lg_master *pLgMaster, int32_t half_pattern);
int ROIoff(struct lg_master *pLgMaster);
int doLGSTOP(struct lg_master *pLgMaster);
int doROIOff(struct lg_master *pLgMaster);
int doDevDisplay(struct lg_master *pLgMaster);
int doStopPulse(struct lg_master *pLgMaster);
int doStartPulse(struct lg_master *pLgMaster);
int doSetROI(struct lg_master *pLgMaster, uint32_t write_val);
int doSlowDownTimer(struct lg_master *pLgMaster);
int doLoadWriteNum(struct lg_master *pLgMaster, uint32_t write_count);
int doLoadReadNum(struct lg_master *pLgMaster, uint32_t read_count);
int doSetPulseOff(struct lg_master *pLgMaster, uint32_t pulse_off);
int doSetPulseOn(struct lg_master *pLgMaster, uint32_t pulse_on);
int doLoadThreshold(struct lg_master *pLgMaster, uint32_t threshold);
int doSetClock(struct lg_master *pLgMaster, uint32_t clock_rate);
int doSetXOffset(struct lg_master *pLgMaster, uint32_t xoff);
int doSetYOffset(struct lg_master *pLgMaster, uint32_t yoff);
void ZeroLGoffset(struct lg_master *pLgMaster);
void SetHWtrigger(struct lg_master *pLgMaster);
void LedBoot(struct lg_master *pLgMaster);
void GoToPulse(struct lg_master *pLgMaster,uint32_t *x, uint32_t *y, int32_t pulseoffvalue, int32_t pulseonvalue);
void GoToRaw(struct lg_master *pLgMaster, uint32_t *x, uint32_t *y);
int move_dark(struct lg_master *pLgMaster, int32_t new_xpos, int32_t new_ypos);
void ResumeDisplay(struct lg_master *pLgMaster);
void OffHWflag(struct lg_master *pLgMaster);
void OnHWflag(struct lg_master *pLgMaster);
int32_t GetQCflag(struct lg_master *pLgMaster);
int DoLevelSearch(struct lg_master *pLgMaster,
		  uint32_t x0,
		  uint32_t y0,
		  uint32_t dx,
		  uint32_t dy,
		  uint32_t n,
		  uint32_t *out);
int DoLineSearch(struct lg_master *pLgMaster,
		 uint32_t x0,
		 uint32_t y0,
		 uint32_t dx,
		 uint32_t dy,
		 uint32_t n,
		 unsigned char *out);
void PostCmdDispNoResp(struct lg_master *pLgMaster, struct displayData *p_dispdata, uint32_t respondToWhom);
void PostCmdDisplay(struct lg_master *pLgMaster, struct displayData *p_dispdata, uint32_t respondToWhom);

#endif // BOARDCOMM_H
