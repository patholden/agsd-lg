#include <stdint.h>
#ifndef DOAUTOFOCUSCMD_H
#define DOAUTOFOCUSCMD_H

extern
void
DoAutoFocusCmd( struct lg_master *pLgMaster, unsigned char * buffer );

extern int  InitAutoFocus ( void );
extern void FreeUpAFBuffs(void);
extern void DoAutoFocusCmd(struct lg_master *pLgMaster, unsigned char *buffer);

#endif
