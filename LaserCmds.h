/*   $Id: LaserCmds.h,v 1.8 2001/01/03 18:01:36 ags-sw Exp pickle $  */
#ifndef LASERCMDS_H
#define LASERCMDS_H

void ResetFlexPlyCounter(void);
void CloseLaserCommands (void);
void InitLaserCommands ( void );
void DoStop ( struct lg_master *pLgMaster, uint32_t respondToWhom );
void DoGoAngle(struct lg_master *pLgMaster,struct parse_goangle_parms *pInp, uint32_t respondToWhom);
void DoEtherAngle(struct lg_master *pLgMaster, struct parse_ethangle_parms *pInp, uint32_t respondToWhom);
void DarkAngle(struct lg_master *pLgMaster, double x, double y, uint32_t respondToWhom);
void DoSearchForASensor(double x, double y, uint32_t respondToWhom);
void DoFullReg(struct lg_master *pLgMaster, struct parse_dofullreg_parms* pInp,
				uint32_t respondToWhom);
void DoDisplay(struct lg_master *pLgMaster, uint32_t dataLength, char *otherParameters, char *patternData);
void DoNewDisplay(uint32_t dataLength, char *otherParameters, char *patternData,
		  uint32_t respondToWhom);
void DoDisplayNoQuickCheck(uint32_t dataLength, char *transformation,
			   char *patternData, uint32_t respondToWhom);
void DoDisplayChunksStart(struct lg_master *pLgMaster,
			  struct parse_chunkstart_parms *pInp,
			  uint32_t respondToWhom );
void DoDisplayChunks(struct lg_master *pLgMaster,
		     struct parse_chunksdo_parms *pInp,
		     uint32_t respondToWhom );

void DoDisplayKitVideo (struct lg_master *pLgMaster, uint32_t dataLength,
			unsigned char * otherParameters, char * patternData,
			uint32_t respondToWhom);
void AddDisplayChunksData(struct lg_master *pLgMaster, uint32_t dataLength,
			  uint32_t dataOffset, char *patternData, uint32_t respondToWhom);

void DoQuickCheck(struct lg_master *pLgMaster, char *angles, uint32_t respondToWhom);
void AbortDisplay(void);
void ResetPlyCounter(void);
void SetDisplaySeveral(struct lg_master *pLgMaster, uint32_t number,
		       uint32_t respondToWhom);
void DoDisplayVideoCheck(uint32_t dataLength, char *otherParameters,
                        char *patternData, uint32_t respondToWhom);
void DoSegmentDisplay(double x, double y, uint32_t respondToWhom);
void DoDisplayChunksRaw(int32_t respondToWhom);
void DimAngle (struct lg_master *pLgMaster, char * parameters );

double DoubleFromCharConv(unsigned char *theChar);
void ShortConv(unsigned char *theChar);
extern unsigned char           gAbortDisplay;
extern uint16_t                gPlysToDisplay;
extern uint32_t                gPlysReceived;
#endif
