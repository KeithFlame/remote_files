
/*****************************************************************************/
/*                                                                           */
/*              Copyright (c) SURGERII. All rights reserved                  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* File name: sr_alarm.h             Version: 1.0                            */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Type: class definition            Programming language: C++               */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Author: Dai Liang                 Date: 15.03.2019                        */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Description:                                                              */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
#ifndef _SR_TIMER_H_
#define _SR_TIMER_H_
#include "sr_globaldefine.h"
namespace sr
{
	typedef void(*TimerTimeout)(void*);
	typedef U32 TimerId;
	const TimerId INVALID_ALARM_IDENT = 0U;
	typedef enum
	{
		TIME_UNIT_MS,
		TIME_UNIT_TICK,
	}TimeUnitType;
	TimerId createTimer(TimeUnitType time_unit,
		U32 time_count,
		TimerTimeout handler,
		void* const param,
		Boolean bLoop);
	Boolean startTimer(const TimerId ident, U32 time_count =0U);
	Boolean stopTimer(const TimerId ident);
	Boolean deleteTimer(const TimerId ident);

}
#endif /*SR_ALARM_H*/