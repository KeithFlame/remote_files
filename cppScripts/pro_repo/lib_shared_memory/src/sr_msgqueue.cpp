/*****************************************************************************/
/*                                                                           */
/*              Copyright (c) SURGERII. All rights reserved                  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* File name: sr_msgqueue.cpp        Version: 1.0                            */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Type: class definition            Programming language: C++               */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Author: Dai Liang                 Date: 18.03.2019                        */
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
#include "sr_msgqueue.h"
#include "sr_semaphore.h"
#include "sr_mutex.h"
#include "sr_common.h"
#include <string.h>
#include <stdlib.h> 
namespace sr
{
	class CSRMsgQueuePrivate
	{
	public:
		CSRMsgQueuePrivate();
		~CSRMsgQueuePrivate();
		Boolean create(const S8* const pname, U32 depth, U32 width);
		Boolean send(const void* const msg, const U32 len, const U32 wait_ms = 0U);
		Boolean vSend(const void* const msg, const U32 len, const U32 wait_ms = 0U);
		U32 recv(void* const msg, const U32 len, const U32 block_ms = NO_TIME_OUT);
		U32 peek(void* const msg, const U32 len, const U32 block_ms = NO_TIME_OUT);
		Boolean pop();
		Boolean clean();
		Boolean isEmpty();
		Boolean isFull();
	private:
		Boolean send(const void* const msg, const U32 len, const U32 wait_ms, Boolean is_vmsg);
		void copyToBuffer(const void* const msg, const U32 len, Boolean is_vmsg = SR_FALSE);
		U32 recv(void* const msg, const U32 len, const U32 block_ms, Boolean is_pop);
		U32 copyToUser(void* const buffer, const U32 buffer_size, Boolean is_pop = SR_TRUE);
	private:
		S8 *m_buffer;
		U32 m_head;
		U32 m_tail;
		U32 m_depth;
		U32 m_width;
		CSRSemaphore m_semaphore_send;
		CSRSemaphore m_semaphore_recv;
		CSRMutex m_mutex_head;
		CSRMutex m_mutex_tail;
	};

	CSRMsgQueuePrivate::CSRMsgQueuePrivate()
		:m_buffer(NULL), m_head(0), m_tail(0), m_depth(0), m_width(0)
	{
		m_buffer = NULL;
	}

	CSRMsgQueuePrivate::~CSRMsgQueuePrivate()
	{
		if (m_buffer)
		{
			free(m_buffer);
			m_buffer = NULL;
		}
	}
	Boolean CSRMsgQueuePrivate::create(const S8* const pname, U32 depth, U32 width)
	{
		m_buffer = (S8*)malloc(depth*(sizeof(U64) + width));
		if (m_buffer == NULL)
		{
			return SR_FALSE;
		}
		m_depth = depth;
		m_width = width;
		for(U32 i=0;i < (m_depth-1);i++)
		{
			m_semaphore_recv.release();
		}
		return SR_TRUE;
	}
	Boolean CSRMsgQueuePrivate::send(const void* const msg, const U32 len, const U32 wait_ms, Boolean is_vmsg)
	{
		Boolean rc = SR_FALSE;
		if (wait_ms == 0)
		{			
			if (isFull())//buffer is full
			{
				return SR_FALSE;
			}
			//m_semaphore_recv.reset();//can not delete
			copyToBuffer(msg, len, is_vmsg);
			return SR_TRUE;
		}
		m_semaphore_recv.reset();//can not delete
		if (isFull())//buffer is full
		{
			Boolean take_result = SR_FALSE;
			take_result = m_semaphore_recv.take(wait_ms);
			if (isFull())//buffer is full
			{
				return SR_FALSE;
			}
			//m_semaphore_recv.reset();//can not delete
			copyToBuffer(msg, len, is_vmsg);
			return SR_TRUE;
		}
		//m_semaphore_recv.reset();//can not delete
		copyToBuffer(msg, len, is_vmsg);
		return SR_TRUE;
	}
	void CSRMsgQueuePrivate::copyToBuffer(const void* const msg, const U32 len, Boolean is_vmsg /*= SR_FALSE*/)
	{
		m_mutex_head.lock();
		S8* base_addr = m_buffer + (sizeof(U32) + m_width)*m_head + sizeof(U32);
		U32* write_len = (U32*)(m_buffer + (sizeof(U32) + m_width)*m_head);
		if (is_vmsg){
			copyVmsg(base_addr, (S8*)msg, len);
			*write_len = getVmsgLen((S8*)msg, len);
		}
		else{
			memcpy(base_addr, (S8*)msg, len);
			*write_len = len;
		}
		m_head = (m_head + 1) % m_depth;
		m_semaphore_send.release();
		m_mutex_head.unLock();
	}
	Boolean CSRMsgQueuePrivate::vSend(const void* const msg, const U32 len, const U32 wait_ms /*= 0*/)
	{
		//Boolean rc = SR_FALSE;
		U16 true_len = getVmsgLen((S8*)msg, len);
		if (true_len > m_width)
		{
			return SR_FALSE;
		}
		return send(msg, len, wait_ms, SR_TRUE);
	}
	Boolean CSRMsgQueuePrivate::send(const void* const msg, const U32 len, const U32 wait_ms/* = 0*/)
	{
		//Boolean rc = SR_FALSE;
		if (len > m_width)
		{
			return SR_FALSE;
		}
		return send(msg, len, wait_ms, SR_FALSE);
	}
	U32 CSRMsgQueuePrivate::copyToUser(void* const buffer, const U32 buffer_size, Boolean is_pop /*= SR_TRUE*/)
	{
		U32 rc = 0;
		m_mutex_tail.lock();
		U32 data_len = *(U32*)(m_buffer + (sizeof(U32) + m_width)*m_tail) > buffer_size ? buffer_size : *(U32*)(m_buffer + (sizeof(U32) + m_width)*m_tail);// min(, len);
		memcpy(buffer, m_buffer + (sizeof(U32) + m_width)*m_tail + sizeof(U32), data_len);
		if (is_pop)
		{
			m_tail = (m_tail + 1) % m_depth;
			m_semaphore_recv.release();
		}
		rc = data_len;
		m_mutex_tail.unLock();
		return rc;
	}
	U32 CSRMsgQueuePrivate::recv(void* const msg, const U32 len, const U32 block_ms, Boolean is_pop)
	{
		//U32 rc = 0;
		if (block_ms == 0)
		{
			if (isEmpty())
			{
				return 0;
			}
			//m_semaphore_send.take();
			return copyToUser(msg, len, is_pop);
		}
		m_semaphore_send.reset();//can not delete
		if (isEmpty())
		{
			Boolean take_result = SR_FALSE;		
			take_result = m_semaphore_send.take(block_ms);
			if (isEmpty())
			{
				return 0;
			}
			return copyToUser(msg, len, is_pop);
		}
		return copyToUser(msg, len, is_pop);
	}
	U32 CSRMsgQueuePrivate::recv(void* const msg, const U32 len, const U32 block_ms /*= NO_TIME_OUT*/)
	{
		return recv(msg, len, block_ms,SR_TRUE);
	}

	U32 CSRMsgQueuePrivate::peek(void* const msg, const U32 len, const U32 block_ms /*= NO_TIME_OUT*/)
	{
		return recv(msg, len, block_ms, SR_FALSE);
	}
	Boolean  CSRMsgQueuePrivate::pop()
	{
		m_mutex_tail.lock();
		Boolean rc = SR_FALSE;
		if (m_tail != m_head) //buffer is empty
		{
			m_tail = (m_tail + 1) % m_depth;
			m_semaphore_recv.release();
			rc = SR_TRUE;
		}
		m_mutex_tail.unLock();
		return rc;
	}
	Boolean  CSRMsgQueuePrivate::clean()
	{
		m_mutex_tail.lock();
		m_tail = m_head;
		m_mutex_tail.unLock();
		m_semaphore_recv.release();
		return SR_TRUE;
	}
	Boolean CSRMsgQueuePrivate::isEmpty()
	{
		Boolean rc = SR_FALSE;
		m_mutex_tail.lock();
		rc = m_tail == m_head ? SR_TRUE : SR_FALSE;
		m_mutex_tail.unLock();
		return rc;
	}
	Boolean CSRMsgQueuePrivate::isFull()
	{
		Boolean rc = SR_FALSE;
		m_mutex_head.lock();
		rc = (m_head + 1) % m_depth == m_tail ? SR_TRUE : SR_FALSE;
		m_mutex_head.unLock();
		return rc;
	}

	CSRMsgQueue::CSRMsgQueue()
	{
		m_p = new CSRMsgQueuePrivate();
	}
	CSRMsgQueue::~CSRMsgQueue()
	{
		if (m_p != NULL)
		{
			delete m_p;
		}
	}
	Boolean CSRMsgQueue::create(const S8* const pname, U32 depth, U32 width)
	{
		memcpy(m_queue_name, pname, sizeof(m_queue_name)>(strlen(pname) + 1) ? (strlen(pname) + 1) : sizeof(m_queue_name)/*min(sizeof(m_queue_name), strlen(pname)+1)*/);
		return m_p->create(pname, depth, width);
	}
	Boolean CSRMsgQueue::vSend(const void* const msg, const U32 len, const U32 wait_ms/* = 0*/)
	{
		return m_p->vSend(msg, len, wait_ms);
	}
	Boolean CSRMsgQueue::send(const void* const msg, const U32 len, const U32 wait_ms/* = 0*/)
	{
		return m_p->send(msg, len, wait_ms);
	}
	U32 CSRMsgQueue::recv(void* const msg, const U32 len, const U32 block_ms /*= NO_TIME_OUT*/)
	{
		return m_p->recv(msg, len, block_ms);
	}
	U32 CSRMsgQueue::peek(void* const msg, const U32 len, const U32 block_ms /*= NO_TIME_OUT*/)
	{
		return m_p->peek(msg, len, block_ms);
	}
	Boolean CSRMsgQueue::pop()
	{
		return m_p->pop();
	}
	Boolean CSRMsgQueue::clean()
	{
		return m_p->clean();
	}
	Boolean CSRMsgQueue::isEmpty()
	{
		return m_p->isEmpty();
	}
}
