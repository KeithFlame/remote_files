/*****************************************************************************/
/*                                                                           */
/*              Copyright (c) SURGERII. All rights reserved                  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* File name: sr_semaphore.cpp       Version: 1.0                            */
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
#include "sr_semaphore.h"
#include <Windows.h>
namespace sr
{
//
class CSRSemaphorePrivate{
public:
	CSRSemaphorePrivate();
	~CSRSemaphorePrivate();

	void reset();

	Boolean take(const U32 block_ms = NO_TIME_OUT);

	Boolean release();

private:
	HANDLE event_sem;
};

CSRSemaphorePrivate::CSRSemaphorePrivate()
{
	event_sem = ::CreateEvent(0, FALSE, FALSE, 0);
}
CSRSemaphorePrivate::~CSRSemaphorePrivate(){
	::CloseHandle(event_sem);
}

void CSRSemaphorePrivate::reset()
{
	::ResetEvent(event_sem);
}

Boolean CSRSemaphorePrivate::take(const U32 block_ms/* = NO_TIME_OUT*/)
{
	return (::WaitForSingleObject(event_sem, (DWORD)block_ms) == WAIT_OBJECT_0 ? SR_TRUE : SR_FALSE);
}

Boolean CSRSemaphorePrivate::release()
{
	//#[ operation unLock()      
	return (SetEvent(event_sem) == TRUE ? SR_TRUE : SR_FALSE);
	//#]
}


CSRSemaphore::CSRSemaphore()
{
	m_p = new CSRSemaphorePrivate();
}


CSRSemaphore::~CSRSemaphore()
{
	if (m_p != NULL){
		delete m_p;
	}
}

void CSRSemaphore::reset()
{
	m_p->reset();
}

Boolean CSRSemaphore::take(const U32 block_ms/* = NO_TIME_OUT*/)
{
	return m_p->take(block_ms);
}

Boolean CSRSemaphore::release()
{    
	return m_p->release();
}
}