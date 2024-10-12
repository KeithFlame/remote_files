/*****************************************************************************/
/*                                                                           */
/*              Copyright (c) SURGERII. All rights reserved                  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* File name: sr_msgqueue.h         Version: 1.0                             */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Type: class definition            Programming language: C++               */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Author: Dai Liang                 Date:18.03.2019                         */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Description:                                                              */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
#ifndef _SR_MSG_QUEUE_H_
#define _SR_MSG_QUEUE_H_
#include "sr_globaldefine.h"
namespace sr
{
	const U32 MAX_QUEUE_NAME_LENGTH = 32;
	class CSRMsgQueuePrivate;
	class CSRMsgQueue
	{
	public:
		CSRMsgQueue();
		~CSRMsgQueue();
		Boolean create(const S8* const pname, U32 depth, U32 width);
		Boolean send(const void* const msg, const U32 len, const U32 wait_ms = 0);
		Boolean vSend(const void* const msg, const U32 len, const U32 wait_ms = 0);
		U32 recv(void* const msg, const U32 len, const U32 block_ms = NO_TIME_OUT);
		U32 peek(void* const msg, const U32 len, const U32 block_ms = 0);
		Boolean pop();
		Boolean clean();
		Boolean isEmpty();
	private:
		S8 m_queue_name[MAX_QUEUE_NAME_LENGTH];
		CSRMsgQueuePrivate* m_p;
	};
}
#endif