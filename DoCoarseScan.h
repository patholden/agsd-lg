#include <stdint.h>
#ifndef DOCOARSESCAN_H
#define DOCOARSESCAN_H

int DoCoarseScan(struct lg_master *pLgMaster, int32_t dX, int32_t dY,
		 uint32_t lsstep, int lscount,
		 int32_t *xfound, int32_t *yfound);
void InitCoarseScan ( void ) ;

#endif
