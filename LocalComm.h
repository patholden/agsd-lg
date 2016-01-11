#include <stdint.h>
/*   $Id: LocalComm.h,v 1.2 1996/12/25 18:38:04 ags-sw Exp $  */

#ifndef __unix__
#pragma once
#endif

extern	void	SaveResponse ( Ptr theResponse, int32_t length );
extern	void	SetLocalMode ( Boolean itsLocal );
extern	void	LocalStop ( void );
extern	void	LocalDisplayWithoutTransform ( void );
extern	void	LocalDisplayWithTransform ( void );
extern	void	LocalNewDisplayWithTransform ( void );
extern	void	LocalDisplayNoQuickCheck ( void );
extern	void	CloseLocalMode ( void );
extern	void	SetSensorHere ( short sensorNumber );
extern	void	LocalSearchForASensor ( short sensorNumber );
extern	void	InitLocalMode ( void );
extern	void	LocalSensorSearchFromHere ( void );
extern	void	LocalFullRegistration ( void );
extern	void	GetLocalSensorCoordinates ( void );
extern	void	DoAutoCert ( void );
extern	void	DoInspection ( void );
extern	void	LocalGoAngle ( double x, double y );
extern	Boolean	LocalGoAngleSuccess ( void );
extern	void	DoLocalWindowClick ( Point pt );
