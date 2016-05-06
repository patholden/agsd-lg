#include <stdint.h>
/* $Id: Init.h,v 1.3 1999/08/09 15:11:29 ags-sw Exp $ */

#ifndef INIT_H
#define INIT_H

// FIXME PAH need to add to master structure:
extern int gROIsearch;

// Function Prototypes
int FlashInit(void);
void DoReInit(struct lg_master *pLgMaster, uint32_t respondToWhom );
int LGMasterInit(struct lg_master *pLgMaster);
int ConfigDataInit(struct lg_master *pLgMaster);
void LGMasterFree(struct lg_master *pLgMaster);
#endif   // INIT_H
