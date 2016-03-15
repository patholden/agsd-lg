#include <stdint.h>
#ifndef DOCOARSESCAN_H
#define DOCOARSESCAN_H

int DoCoarseScan(struct lg_master *pLgMaster, int16_t dX, int16_t dY,
		 uint32_t lsstep, int lscount,
		 int16_t *xfound, int16_t *yfound);
#endif
