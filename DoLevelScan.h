#include <stdint.h>

#ifndef DOLEVELSCAN_H
#define DOLEVELSCAN_H

extern void DoLevelScan(struct lg_master *pLgMaster, double dX, double dY );
extern void InitLevelScan(void);

extern uint32_t * scandata;

#define LSCNTSTEP   64
#define  LSSTEPSIZE  0x80000

#endif
