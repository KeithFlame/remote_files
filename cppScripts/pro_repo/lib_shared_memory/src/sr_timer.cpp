/****************************T*************************************************/
/*                                                                           */
/*              Copyright (c) SURGERII. All rights reserved                  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* File name: sr_alarm.cpp                 Version: 1.0                      */
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
/* Date     | History                                                        */
/* ---------+--------------------------------------------------------------- */
/*          |                                                                */
/*          |                                                                */
/*****************************************************************************/
#include "sr_timer.h"
#include "sr_mutex.h"
#include "sr_semaphore.h"
#include <Windows.h>
#pragma comment(lib, "winmm.lib")  
namespace sr
{
	const U16  MAX_TIMER_COUNT = 256;
	typedef struct {               /* structure of timer table entry */
		U32             activate_count;      /* activation counter. sign is MUST */
		U32             reload;                /* reload value for cyclic functions */
		TimerTimeout    cb_handle; /* timed function */
		void*           cb_param;
		TimerId         id;
		Boolean         is_started;
		Boolean         is_loop;
	} tTimerEntry;

	/*----------------------------------------------------------------------*/
	/*      MACRO Type Definition                                           */
	/*----------------------------------------------------------------------*/
	/* 1ms as Timer resolution */
	const U16 TIMER_ACCURACY = 1;
	/* timer tick every 1 ms */
	const U16 SYSTEMTIMETICK = 1;
	/*--------------------------------------------------------------------------*/
	/*  private data                                                            */
	/*--------------------------------------------------------------------------*/
	static tTimerEntry timer_table[MAX_TIMER_COUNT]; /* timer table                   */

	//static U32 start_tick = 0;
	static U32 auto_incre_id = 1;
	static CSRMutex mutex;
	static CSRSemaphore sem_cycle;
	static MMRESULT mmr_timerid = NULL;             /* timer id */
	static HANDLE tid_schedule_handle = 0;
	static CSRMutex mutex_instanse_create;

	static TimerId last_timeout_timerid = INVALID_ALARM_IDENT;

	void initTable()
	{
		for (U16 i = 0; i < MAX_TIMER_COUNT; i++)
		{
			timer_table[i].activate_count = 0;
			timer_table[i].reload = 0;
			timer_table[i].cb_handle = NULL;
			timer_table[i].id = INVALID_ALARM_IDENT;
			timer_table[i].is_started = SR_FALSE;
		}

		//start_tick = GetTickCount();
	}
	void PASCAL oneMilliSecondProc(U32 timerid, U32 msg, U32 dwuser, U32 dwl, U32 dw2)
	{
		sem_cycle.release();
	}
	Boolean startTimerTrigger()
	{
		Boolean rc = SR_FALSE;
		TIMECAPS timecaps;              /* min and max timer values */
		/*                                      ** get min and max period */
		if (timeGetDevCaps(&timecaps, sizeof(timecaps)) == TIMERR_NOERROR)
		{
			U32 accuracy;
			accuracy = min(max(timecaps.wPeriodMin, TIMER_ACCURACY), timecaps.wPeriodMax);

			timeBeginPeriod(accuracy);
			Sleep(128);/* wait for it to stabilize */

			/* create timer to release event */
			mmr_timerid = timeSetEvent(SYSTEMTIMETICK, 0, (LPTIMECALLBACK)oneMilliSecondProc, 0, TIME_PERIODIC);
			if (mmr_timerid == (MMRESULT)0)
			{
				//printf("### Error Failed to generate multimedia timer.\n");
				timeEndPeriod(1);
			}
			else
			{
				rc = SR_TRUE;
			}
		}
		return rc;
	}
	static void scheduleTimer()
	{
		U16 i;
		/*
		* check the scheduler table for installed handlers.
		* remember that the first entry in the table is not used.
		*/
		if (mutex.lock())
		{
			for (i = 1; i < MAX_TIMER_COUNT; i++)
			{
				/* there is an installed handler */
				if (timer_table[i].cb_handle != NULL && timer_table[i].is_started)
				{
					/*
					* the activation time for task activation is elapsed
					* if the next decrementation by SYSTEMTIMETICK results in
					* a wActivateCounter smaller than SYSTEMTIMETICK.
					* this means that the time is considered as elapsed
					* if the wActivateCounter is lower than SYSTEMTIMETICK.
					*/
					if (timer_table[i].activate_count < (2*SYSTEMTIMETICK))
					{
						/* checking for NULL is already done */
						last_timeout_timerid = i;
						(*timer_table[i].cb_handle)(timer_table[i].cb_param);

						if (timer_table[i].is_loop)
						{
							timer_table[i].activate_count =
								timer_table[i].activate_count -
								SYSTEMTIMETICK +
								timer_table[i].reload;
						}
						else
						{
							timer_table[i].cb_handle = NULL;
							timer_table[i].id = INVALID_ALARM_IDENT;
						}
					}
					else /* activation time is not elapsed */
					{
						/*
						* the activation counter is decremented by
						* the SYSTEMTIMETICK. often there is a rest while the
						* task is executed. the rest is taken into account
						* in the update.
						* Check for underflow of wActivateCounter !
						*/
						if (timer_table[i].activate_count > SYSTEMTIMETICK)
						{
							timer_table[i].activate_count -= SYSTEMTIMETICK;
						}
						else
						{
							timer_table[i].activate_count = 0;
						}
					}
				}
			}
			mutex.unLock();
		}
	}
	static U32 startThread(void *arg)
	{
		startTimerTrigger();
		while (true)
		{
			if (sem_cycle.take())
			{
				scheduleTimer();
			}
		}
		return 0U;
	}
	Boolean initTimer()
	{
		// can be called more than once. In this case DCLP used to make Safe-Thread.	
		if (tid_schedule_handle == 0)
		{
			if (mutex_instanse_create.lock())
			{
				if (tid_schedule_handle == 0)
				{
					initTable();
					DWORD dwThreadId = 0;
					tid_schedule_handle = CreateThread(
						NULL,
						0,
						(LPTHREAD_START_ROUTINE)startThread,
						(LPVOID)NULL,		// argument pointer 
						NULL,					// start immediately
						&dwThreadId);
				}
				mutex_instanse_create.unLock();
			}
		}
		return tid_schedule_handle != 0 ? SR_TRUE : SR_FALSE;
	}

	bool isIdentAvailable(U32 id)
	{
		if (id == INVALID_ALARM_IDENT)
		{
			return false;
		}
		for (U16 i = 1; i < MAX_TIMER_COUNT; i++)
		{
			if (id == timer_table[i].id)
			{
				return false;
			}
		}
		return true;
	}

	U32 getAvailableIdent()
	{
		auto_incre_id++;
		while (!isIdentAvailable(auto_incre_id))
		{
			auto_incre_id++;
		}
		return auto_incre_id;
	}
		

	/* BEGIN_DESCRIPTION *******************************************************/
	/* This function installs a function in the timer table.                   */
	/* It makes no sense to install acyclic timers after the timer is started. */
	/* It is possible that the timer interrupt occurs immediately after        */
	/* installing the acyclic timer - the time is much shorter then desired    */
	/* in this case.                                                           */
	/*                                                                         */
	/* INPUT: time_unit - time unit, currently only support Alarm_Time_Unit_ms */
	/*		  wTaskTime - time in time_unit                                    */
	/*        bLoop     - true cyclic, false acyclic                           */
	/*        fnTimerTask - task to be executed, AlarmServiceRoutine, ASR      */
	/* OUTPUT:TIMHANDLE - handle for timer                                     */
	/*                                                                         */
	/* END_DESCRIPTION *********************************************************/
	TimerId createTimer(TimeUnitType time_unit, U32 time, TimerTimeout handle, void* const param, Boolean loop)
	{
		TimerId rc = INVALID_ALARM_IDENT;

		if (time < SYSTEMTIMETICK) /* correct wTaskTime */
		{
			time = SYSTEMTIMETICK;
		}

		if (initTimer())
		{
			if (mutex.lock())
			{
				/* remember that the entry 0 is not used */
				for (U16 i = 1; i < MAX_TIMER_COUNT; i++)
				{
					/* checking for an empty entry in timer table */
					if ((timer_table[i].cb_handle == NULL) && (last_timeout_timerid != i))
					{
						timer_table[i].activate_count = time;
						timer_table[i].cb_handle = handle;
						timer_table[i].cb_param = param;
						timer_table[i].reload = time;
						timer_table[i].is_loop = loop;
						timer_table[i].is_started = SR_FALSE;
						rc = (timer_table[i].id = getAvailableIdent()/*auto_incre_id++*/);
						break;
					}
				}
				mutex.unLock();
			}
		}
		return rc;
	}
	/* BEGIN_DESCRIPTION *******************************************************/
	/* This function deletes a timed function previously installed with        */
	/* Tim_SetAlarm().                                                         */
	/*                                                                         */
	/* INPUT: hTime - timer function's handle                                  */
	/* OUTPUT:1 - timed function deleted                                       */
	/*        0 - handle doesn't exist                                         */
	/*                                                                         */
	/* END_DESCRIPTION *********************************************************/
	/* BEGIN_FCTCALL ***********************************************************/
	Boolean deleteTimer(const TimerId ident)
		/* END_FCTCALL *************************************************************/
	{
		Boolean rc = SR_FALSE;
		if (ident != INVALID_ALARM_IDENT)
		{
			if (mutex.lock())
			{
				for (U16 i = 1; i < MAX_TIMER_COUNT; i++)
				{
					if (timer_table[i].id == ident)
					{
						timer_table[i].cb_handle = NULL;
						timer_table[i].id = INVALID_ALARM_IDENT;
						rc = SR_TRUE;
						break;
					}
				}
				mutex.unLock();
			}
		}
		return rc;
	}
	Boolean changTimerStatus(const TimerId ident, Boolean start, U32 time_count = 0)
	{
		Boolean rc = SR_FALSE;
		if (ident != INVALID_ALARM_IDENT)
		{
			if (mutex.lock())
			{
				for (U16 i = 1; i < MAX_TIMER_COUNT; i++)
				{
					if (timer_table[i].id == ident && timer_table[i].cb_handle != NULL)
					{
						timer_table[i].is_started = start;
						if (start)
						{
							if (time_count == 0)
							{
								timer_table[i].activate_count = timer_table[i].reload;
							}
							else
							{
								timer_table[i].activate_count = time_count;
							}							
						}
						rc = SR_TRUE;
						break;
					}
				}
				mutex.unLock();
			}
		}
		return rc;
	}
	Boolean startTimer(const TimerId ident, U32 time_count /*= 0*/)
	{
		Boolean rc = changTimerStatus(ident, SR_TRUE, time_count);
		return rc;
	}

	Boolean stopTimer(const TimerId ident)
	{
		Boolean rc = changTimerStatus(ident, SR_FALSE);
		return rc;
	}

	//????????????
	Boolean destroyTimer()
	{
		Boolean rc = SR_FALSE;
		if (mmr_timerid != (MMRESULT)0)
		{
			timeKillEvent(mmr_timerid);
			rc = SR_TRUE;
		}
		return rc;
	}
}

