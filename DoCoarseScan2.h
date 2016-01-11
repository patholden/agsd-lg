#include <stdint.h>
#ifndef DOCOARSESCAN2_H
#define DOCOARSESCAN2_H

int DoCoarseScan2(struct lg_master *pLgMaster,
		  uint32_t dX, uint32_t dY,
		  uint32_t lsstep, int lscount,
		  uint32_t *xfound, uint32_t *yfound);


#endif
