#ifndef LASERFLEX_H
#define LASERFLEX_H
void DoThresholdQuickCheck(struct lg_master *pLgMaster, char * data, uint32_t respondToWhom);
void DoFlexQuickCheck(struct lg_master *pLgMaster, struct parse_flexquickcheck_parms* data, uint32_t respondToWhom);
void DoFlexDisplayChunks(struct lg_master *pLgMaster,
			 struct parse_chunkflex_parms *parameters,
			 uint32_t respondToWhom);
void DoFlexDisplay (struct lg_master *pLgMaster,
		    uint32_t dataLength,
		    struct parse_flexdisp_parms * parameters,
		    unsigned char * patternData);
#endif // LASERFLEX_H

