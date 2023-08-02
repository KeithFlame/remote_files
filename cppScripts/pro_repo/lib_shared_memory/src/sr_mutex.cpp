/*****************************************************************************/
/*                                                                           */
/*              Copyright (c) SURGERII. All rights reserved                  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* File name: sr_mutex.cpp           Version: 1.0                            */
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
#include "sr_mutex.h"
#include <Windows.h>
namespace sr
{
	class CSRMutexPrivate
	{
	public:
		CSRMutexPrivate();
		~CSRMutexPrivate();
		Boolean lock();
		Boolean unLock();
	private:
		CRITICAL_SECTION m_criticalsection;
	};


	//## class CSRMutexNT
	CSRMutexPrivate::CSRMutexPrivate() {
		//#[ operation CSRMutexNT()
		InitializeCriticalSection(&m_criticalsection);
		//#]
	}



	CSRMutexPrivate::~CSRMutexPrivate() {
		//#[ operation ~CSRMutexNT()
		DeleteCriticalSection(&m_criticalsection);
		//#]
	}



	Boolean CSRMutexPrivate::lock() {
		//#[ operation lock()
		EnterCriticalSection(&m_criticalsection);

		return SR_TRUE;
		//#]
	}

	Boolean CSRMutexPrivate::unLock() {
		//#[ operation unLock()
		LeaveCriticalSection(&m_criticalsection);

		return SR_TRUE;
		//#]
	}


	CSRMutex::CSRMutex()
	{
		m_p = new CSRMutexPrivate();
	}


	CSRMutex::~CSRMutex()
	{
		if (m_p != NULL)
		{
			delete m_p;
		}
	}

	Boolean CSRMutex::lock()
	{
		return	m_p->lock();
	}

	Boolean CSRMutex::unLock()
	{
		return m_p->unLock();
	}
}