#include <stdint.h>
#ifndef DOCOARSESCAN2_H
#define DOCOARSESCAN2_H

int DoCoarseScan2(struct lg_master *pLgMaster,
		  int32_t dX, int32_t dY,
		  uint32_t lsstep, int lscount,
		  int32_t *xfound, int32_t *yfound);


#endif
