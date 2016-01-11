#include <stdint.h>
/* $Id: Files.h,v 1.4 2001/01/03 17:57:21 ags-sw Exp $ */
#define  KERNEL_SIZE  (400*1024)
#define  KERNEL_OFFSET     0

#define  ELF_SIZE     (800*1024)
#define  ELF_OFFSET     (400*1024)

#define  AGS_SIZE     (800*1024)
#define  AGS_OFFSET    (1200*1024)

#define  CALIB_SIZE    (800*1024)
#define  CALIB_OFFSET  (1400*1024)

#define  INIT_SIZE      (5*1024)
#define  INIT_OFFSET   (1473*1024)

#define  INFO_SIZE      (5*1024)

#define  DATA_SIZE      (10*1024)
#define  DATA_OFFSET   (1481*1024)

#define  HOBB_SIZE      (1*1024)
#define  HOBB_OFFSET   (1483*1024)

#define  AUTO_SIZE      (2*1024)
#define  AUTO_OFFSET   (1485*1024)

extern void HandleFilePutData(struct lg_master *pLgMaster, char * parameters, uint32_t respondTo );
extern void DoFilePutStart (struct lg_master *pLgMaster, char * parameters, uint32_t respondToWhom );
extern void DoFileGetStart(struct lg_master *pLgMaster, char * parameters, uint32_t respondToWhom);
extern void DoFileGetData  (struct lg_master *pLgMaster, char * parameters, uint32_t respondToWhom );
extern void DoFilePutDone  (struct lg_master *pLgMaster, char * parameters, uint32_t respondToWhom );

