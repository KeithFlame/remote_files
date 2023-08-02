/*****************************************************************************/
/*                                                                           */
/*              Copyright (c) SURGERII. All rights reserved                  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* File name: sr_semaphore.h            Version: 1.0                         */
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

#ifndef _SR_SEMAPHORE_H_
#define _SR_SEMAPHORE_H_
#include "sr_globaldefine.h"

namespace sr
{
class CSRSemaphorePrivate;
class CSRSemaphore 
{
public:

	CSRSemaphore();
	~CSRSemaphore();
	void reset();

	Boolean take(const U32 block_ms = NO_TIME_OUT);

	Boolean release();
private:
	CSRSemaphorePrivate* m_p;

};
}
#endif