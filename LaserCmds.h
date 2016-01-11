#include <stdint.h>
/*   $Id: LaserCmds.h,v 1.8 2001/01/03 18:01:36 ags-sw Exp pickle $  */

#ifndef __unix__
#pragma once
#endif

#ifndef LASERCMDS_H
#define LASERCMDS_H

extern	void	CloseLaserCommands (void);
extern	void	InitLaserCommands ( void );
extern	void	DoStop ( struct lg_master *pLgMaster, uint32_t respondToWhom );
extern	void	DoGoAngle(struct lg_master *pLgMaster,
			  double x, double y, uint32_t respondToWhom );
extern void DoEtherAngle (struct lg_master *pLgMaster, double x, double y, uint32_t respondToWhom );
extern void DarkAngle (struct lg_master *pLgMaster, double x, double y, uint32_t respondToWhom );
void AngleOffset(struct lg_master *pLgMaster, double x, double y, uint32_t respondToWhom );
extern	void	DoSearchForASensor
			( double x, double y, uint32_t respondToWhom );

extern	void	DoFullReg(struct lg_master *pLgMaster,
				struct parse_dofullreg_parms* pInp,
				uint32_t respondToWhom );

extern	void	DoDisplay ( struct lg_master *pLgMaster,
			    uint32_t dataLength
                          , char * otherParameters
                          , char * patternData
			  );

extern	void	DoNewDisplay ( uint32_t dataLength,
			char * otherParameters, char * patternData,
			uint32_t respondToWhom );

extern	void	DoDisplayNoQuickCheck (uint32_t dataLength,
				       char * transformation,
				       char * patternData,
				       uint32_t respondToWhom );

void DoDisplayChunksStart(struct lg_master *pLgMaster,
			  struct parse_chunkstart_parms *pInp,
			  uint32_t respondToWhom );
void DoDisplayChunks(struct lg_master *pLgMaster,
		     struct parse_chunksdo_parms *pInp,
		     uint32_t respondToWhom );

void DoDisplayKitVideo (struct lg_master *pLgMaster,
			uint32_t dataLength,
			char * otherParameters,
			char * patternData,
			uint32_t respondToWhom);
extern	void	AddDisplayChunksData (struct lg_master *pLgMaster,
				      uint32_t dataLength,
				      uint32_t dataOffset, char * patternData,
				      uint32_t respondToWhom );

extern void DoQuickCheck (struct lg_master *pLgMaster, char * angles, uint32_t respondToWhom );
#if 0
extern	void	DoQuickies ( int32_t StopOrGo,
                   char * angles, uint32_t respondToWhom );
#endif
extern	void	AbortDisplay ( void );

extern	void	ResetPlyCounter ( void );

extern	void	SetDisplaySeveral (struct lg_master *pLgMaster, uint32_t number,
					uint32_t respondToWhom );

extern	void	DoDisplayVideoCheck ( 
                        uint32_t dataLength,
			char * otherParameters,
                        char * patternData,
			uint32_t respondToWhom );

extern	void	DoSegmentDisplay
			( double x, double y, uint32_t respondToWhom );

extern	void	DoDisplayChunksRaw ( int32_t respondToWhom );

void DimAngle (struct lg_master *pLgMaster, char * parameters );

extern	double			DoubleFromCharConv ( unsigned char *theChar );
extern	uint32_t	LongFromCharConv ( unsigned char *theChar );
extern	void			DoubleConv ( unsigned char *theChar );
extern	void			LongConv ( unsigned char *theChar );
extern	void			ShortConv ( unsigned char *theChar );
extern unsigned char           gAbortDisplay;
extern uint16_t                gPlysToDisplay;
extern uint32_t                gPlysReceived;


#endif
