#include <stdint.h>
/*     specific changes for the laserguide system  */

#define   MAX_LG_BUFFER    262144  /* maximum size of lg_data[] */
#define   MAX_DIODE_BUFFER  65536  /* maximum number of diode readings */

/*  IO addresses */
#define   LG_BASE            0x380
#define   LG_IO_CNTRL        0x380
#define   LG_IO_X2           0x382
#define   LG_IO_X3           0x383
#define   LG_IO_Y2           0x386
#define   LG_IO_Y3           0x387
#define   LG_IO_SERVO        0x380
#define   LG_IO_OPTIC        0x388
#define   TFPORTRL           0x390          // Read TF IO port address
#define   TFPORTRH           0x392          // Read upper byte of TF IO ;
#define   TFPORTW            0x394          // Write TF IO port address 
#define   LASER_REGION (TFPORTW-LG_BASE)


// WRITE Command defines
typedef enum{
  CMDW_BUFFER=1,
  CMDW_STOP,
  CMDW_DISPLAY,
  CMDW_RSTRTTMR,
  CMDW_HWTRIGGER,
  CMDW_DODARKSENS,
  CMDW_QUICKCHECK,
  CMDW_SETDELTA,
  CMDW_GOANGLE,
  CMDW_LOADWRTCNT,
  CMDW_LOADRDCNT,
  CMDW_SLOWTIMER,
  CMDW_FASTTIMER,
  CMDW_SETQCCOUNTER,
  CMDW_SETXOFFSET,
  CMDW_SETYOFFSET,
  CMDW_ROIOFF,
  CMDW_SETROI,
  CMDW_STARTPULSE,
  CMDW_STOPPULSE,
  CMDW_SETPULSEONVAL,
  CMDW_SETPULSEOFFVAL,
  CMDW_SETQCFLAG,
  CMDW_SETCLOCK,
  CMDW_LOADTHRESH,
  CMDW_DOSENSOR,
  CMDR_BUFFER,
  CMDR_GETQCCOUNTER,
  CMDR_GETANGLE,
  CMDR_GETQCFLAG,
}lg_cmd_enums;
#define CMD_LAST CMDR_GETQCFLAG   // NOTE:  Change this if appending new commands
struct lg_points {
   uint32_t   xpos;   
   uint32_t   ypos;
};

struct lg_delta {
   int32_t   xdel;   
   int32_t   ydel;
};

struct cmd_rw_base {
  int        cmd;
  int        length;
  union {
    struct {
      uint32_t   val32;
      uint32_t   pad1;
    };
    struct {
      uint16_t   val16;
      uint16_t   pad2;
      uint32_t   pad3;
    };
    struct lg_points xypos;
    struct lg_delta xydel;
    double     val64;
  };
};
struct cmd_rw {
  struct cmd_rw_base base;
  char   data[MAX_LG_BUFFER * sizeof(uint32_t)];
};

struct lg_info {
  /*   positions are 16/18 bits left-justified in 32-bit int32_ts         */
  /*  X and Y positions, plus a strobe bit 7 of first (least sig) byte */
  struct lg_points ptns;
  struct lg_delta  delta;
  /*   change in position for searching sensor                         */
   int32_t   num_points;   /* number of points (as position pairs!) */
   int32_t   roi_points;   /* number region-of-interest points  */
   uint16_t  threshold;  /* diode light threshold -- needed? */
   uint16_t  clock_tick;  /* length of clock tick -- 1.192 MHz */
   uint16_t  pos_offset;  /* position offset (high word) */
      /*  servo status -- input from projector  */
      /*    bit 7   X ok                        */
      /*    bit 6   Y ok                        */
      /*    bit 5   Projector Power Supplies up */
      /*    bit 4   position err comparator  (future imp)       */
      /*    bit 3   (unassigned)                                */
      /*    bit 2   scan. board ID 2                            */
      /*    bit 1   scan. board ID 1                            */
      /*         ID2    ID1                                     */
      /*          0      0      6350 scanners installed         */
      /*          0      1      6810 scanners installed         */
      /*          1      0      undef                           */
      /*          1      1      undef                           */
      /*    bit 0   0=18-bit DAC     1=16-bit DAC               */
   uint8_t   servo_status;
      /*  optics command  --  output to projector   */
      /*   bit 7   fast/slow scanner speed toggle                   */
      /*   bit 6   search power   set enables higher power          */
      /*   bit 5   video switch and flag  (video board?)            */
      /*   bit 4   comm error LED   visible on outside              */
      /*   bit 3   n/a                                              */
      /*   bit 2   ethernet link LED                                */
      /*   bit 1   n/a                                              */
      /*   bit 0   n/a                                              */
   uint8_t   optics_command;
      /*  optics status  --  input from projector   */
      /*   bit 7   shutter relay check ???                          */
      /*   bit 6   trigger 1 (edge)  goes to 1 briefly (how brief?) */
      /*   bit 5   trigger 2 (level)                                */
      /*   bit 4   diode temperature check                          */
      /*   bit 3   Center of Devices and Radiological Health        */
      /*   bit 2   diode current check                              */
      /*   bit 1   interlock check                                  */
      /*   bit 0   n/a                                              */
   uint8_t   optics_status;
      /*  board configuration  --  input from projector   */
      /*   bit 7   optics board ID 3                                */
      /*   bit 6   optics board ID 2                                */
      /*   bit 5   optics board ID 1                                */
      /*   bit 4   optics board ID 0                                */
      /*   bit 3   diode driver ID 3  0000 SDL 15mW 635nm laser diode        */
      /*   bit 2   diode driver ID 2  0001 Sanyo 10mW 635nm laser diode      */
      /*   bit 1   diode driver ID 1  0010 Casix 50mW 532nm Double Nd:YVO4   */
      /*   bit 0   diode driver ID 0    all others undefined                 */
   uint8_t   board_config;
       /* diode_light, diode_boost, and temp_error are inputs for projector */
   uint8_t   diode_light;  /* diode light monitor  0x00 to 0xFF */
   uint8_t   diode_boost;  /* diode boost current  0x00 to 0xFF */
       /*  temperature error      */
       /*  0x00  =  -10 C below set-point  */
       /*  0x80  =  perfect   (nominal setpoint should be 17 C) */
       /*  0xFF  =  +10 C above set-point  */
   uint8_t   temp_error;
   uint8_t   low_ip_octet;
   uint8_t   unassigned1;
   uint8_t   unassigned2;
   uint32_t  unassigned3;
   int32_t            qcFlag;      /* quick check flag (0 is false) */
   int32_t            qcCounter;   /* quick check counter */
   int32_t            pulseonvalue;
   int32_t            pulseoffvalue;
   uint32_t  unassigned4;
};
