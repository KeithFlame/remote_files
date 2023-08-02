/*****************************************************************************/
/*                                                                           */
/*              Copyright (c) SURGERII. All rights reserved                  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* File name: sr_mutex.h            Version: 1.0                             */
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

#ifndef _SR_MUTEX_H_
#define _SR_MUTEX_H_
#include "sr_globaldefine.h"
namespace sr
{
	class CSRMutexPrivate;
	class CSRMutex
	{

	public:

		CSRMutex();
		~CSRMutex();
		Boolean lock();
		Boolean unLock();
	private:
		CSRMutexPrivate* m_p;
	};
}
#endif