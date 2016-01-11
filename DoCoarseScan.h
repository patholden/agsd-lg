#include <stdint.h>
#ifndef DOCOARSESCAN_H
#define DOCOARSESCAN_H

int DoCoarseScan(struct lg_master *pLgMaster,
		 uint32_t dX, uint32_t dY,
		 uint32_t lsstep, int lscount,
		 uint32_t *xfound, uint32_t *yfound);
void InitCoarseScan ( void ) ;

#endif
