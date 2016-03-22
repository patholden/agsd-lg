#ifndef BOARDCOMM_H
#define BOARDCOMM_H
/*   $Id: BoardComm.h,v 1.20 2001/01/03 17:48:53 ags-sw Exp pickle $  */

#define DONTRESPOND  1      // command will NOT send response to PC host
#define SENDRESPONSE  2     // command will send response to PC host
#define  MAX_NUM_THREADS 1
#define STEPDELAY   500
#define NUM_HOBBS_COUNTERS  4
#define PARSE_HOBBS_HOBBS 1
#define PARSE_HOBBS_XSCAN 2
#define PARSE_HOBBS_YSCAN 3
#define PARSE_HOBBS_LASER 4
#define kMaxUnsigned      0xFFFF
#define kMinSigned        0x0000
#define kMaxSigned        0xFFFF

extern  uint32_t  gRespondToWhom;
struct displayData
{
  struct lg_xydata  *pattern;
  int32_t           *sensorAngles;
  uint32_t          numberOfSensorSets;
};

// The following header is fixed for incoming commands from
// host PC
struct cmd_inhdr {
  unsigned char theCommand;
  unsigned char specialByte;
  uint16_t      seqNo;
};
// k_header is used for responding to caller with basic status
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
struct version_info {
  char      *pVersions;
  uint32_t   version_size;
  uint32_t   isVersInit;
};

struct lg_master {
  struct sockaddr_in webhost_addr;
  struct hobbs_ctrs hobbs;
  struct version_info   vers_data;
  struct k_header gOutOfRange;
  struct lg_xydata gSaveXY;
  struct lg_xydata gCheckXY;
  char            webhost[128];
  unsigned char   gBestTargetArray[128];
  double          gArgTol;
  double          ping_count;
  double          gTolerance;
  double          dmax;
  double          gHalfMirror;
  unsigned char   *gInputBuffer;
  char            *gDataChunksBuffer;
  char            *gSensorBuffer;
  char            *gAFInputBuffer;
  unsigned char   *gRawBuffer;
  unsigned char   *gParametersBuffer;
  unsigned char   *theResponseBuffer;
  int16_t         *gScan;
  uint16_t        *coarsedata;
  uint16_t        *gLsort;
  int             af_serial;
  int             socketfd;
  int             datafd;
  int             fd_laser;
  int             serial_ether_flag;
  unsigned long   gProjectorSerialNumber;
  uint32_t        gHEX;
  uint32_t        enet_retry_count;
  uint32_t        gotA1;
  uint32_t        gDataChunksLength;
  uint32_t        gBuiltPattern;
  uint32_t        gPeriod;
  uint32_t        gDisplayFlag;
  uint32_t        gSrchStpPeriod;
  uint32_t        gTransmitLengthSum;
  uint32_t        gBestTargetNumber;
  uint32_t        gPlysToDisplay;
  uint32_t        gPlysReceived;
  int32_t         gQCcount;
  int32_t         gCoarse2SearchStep;
  int16_t         gXcheck;
  int16_t         gYcheck;
  uint16_t        seqNo;
  uint8_t         gCALIBFileOK;
  uint8_t         newCommand;
  uint8_t         gHeaderSpecialByte;
  uint8_t         RFUpad;
  uint8_t         gAbortDisplay;
};

typedef struct ags_bkgd_thread_struct
{
  int              thread_instance;  // (debug)field to distinguish between threads.
  struct lg_master *pLgMaster;      // pointer to master data structure
  int              time_to_update;  // determines when to do backup, set by timer from Main loop
} AGS_BKGD_THREAD;

double ArrayToDouble(double inp_data);
void RandomSegment(void);
int32_t InitQCtimer( void );
int32_t GetQCtimer( void );
void SetROIsearch( void );
void ClearROIsearch(void);
int CheckStopFlag(void);
void PostCommand(struct lg_master *pLgMaster, uint32_t theCommand, char * data,
		 uint32_t respondToWhom);
int IfStopThenStopAndNeg1Else0 (struct lg_master *pLgMaster);
void SetHighBeam(struct lg_xydata *pDevXYData);
void SetLowBeam(struct lg_xydata *pDevXYData);
void SetDarkBeam(struct lg_xydata *pDevXYData);
int doWriteDevCmdNoData(struct lg_master *pLgMaster, uint32_t command);
int doWriteDevCmd32(struct lg_master *pLgMaster, uint32_t command, uint32_t write_val);
int doWriteDevDelta(struct lg_master *pLgMaster, struct lg_xydelta *pDelta);
int doWriteDevPoints(struct lg_master *pLgMaster, struct lg_xydata *pXYData);
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
void JustDoDisplay(struct lg_master *pLgMaster, char * wr_ptr, int patternLength );
void FlashLed(struct lg_master *pLgMaster, int numFlash);         
void SlowDownAndStop(struct lg_master *pLgMaster);
int setROIlength(struct lg_master *pLgMaster, int32_t half_pattern);
int ROIoff(struct lg_master *pLgMaster);
int doLGSTOP(struct lg_master *pLgMaster);
int doROIOff(struct lg_master *pLgMaster);
int doDevDisplay(struct lg_master *pLgMaster);
int doStopPulse(struct lg_master *pLgMaster);
int doStartPulse(struct lg_master *pLgMaster);
int doSetROI(struct lg_master *pLgMaster, uint32_t write_val);
int doSetReadyLED(struct lg_master *pLgMaster);
int doClearReadyLED(struct lg_master *pLgMaster);
int doSetSearchBeam(struct lg_master *pLgMaster);
int doClearSearchBeam(struct lg_master *pLgMaster);
void doClearLinkLED(struct lg_master *pLgMaster);
void doSetLinkLED(struct lg_master *pLgMaster);
int doSetShutterENB(struct lg_master *pLgMaster);
int doLoadWriteNum(struct lg_master *pLgMaster, uint32_t write_count);
int doLoadReadNum(struct lg_master *pLgMaster, uint32_t read_count);
int doSetPulseOff(struct lg_master *pLgMaster, uint32_t pulse_off);
int doSetPulseOn(struct lg_master *pLgMaster, uint32_t pulse_on);
int doSetClock(struct lg_master *pLgMaster, uint32_t clock_rate);
int doSetXOffset(struct lg_master *pLgMaster, uint32_t xoff);
int doSetYOffset(struct lg_master *pLgMaster, uint32_t yoff);
void ZeroLGoffset(struct lg_master *pLgMaster);
void GoToRaw(struct lg_master *pLgMaster, struct lg_xydata *pRawData);
void GoToPulse(struct lg_master *pLgMaster, struct lg_xydata *pPulseData,
	       int32_t pulseoffvalue, int32_t pulseonvalue);
int move_dark(struct lg_master *pLgMaster, struct lg_xydata *pNewData);
void ResumeDisplay(struct lg_master *pLgMaster);
int32_t GetQCflag(struct lg_master *pLgMaster);
int DoLevelSearch(struct lg_master *pLgMaster, struct lg_xydata *pSrchData,
		  struct lg_xydelta *pDeltaData, int16_t n, int16_t *c_out);
int DoLineSearch(struct lg_master *pLgMaster, struct lg_xydata *pSrchData,
		 struct lg_xydelta *pDeltaData, int16_t n, unsigned char *c_out);
void PostCmdDisplay(struct lg_master *pLgMaster, struct displayData *p_dispdata, int32_t do_response, uint32_t respondToWhom);
void PostCmdEtherAngle(struct lg_master *pLgMaster, struct lg_xydata *pAngleData);
void PostCmdGoAngle(struct lg_master *pLgMaster, struct lg_xydata *pAngleData, uint32_t respondToWhom);

#endif // BOARDCOMM_H
