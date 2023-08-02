/*****************************************************************************/
/*                                                                           */
/*              Copyright (c) SURGERII. All rights reserved                  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* File name: sr_mbx.cpp                   Version: 1.0                      */
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
#include "sr_mbx.h"
#include "sr_common.h"
#include <Windows.h>
#include <stdio.h>
namespace sr
{
	struct MailboxAttributeTag {
		U16 depth;		//## attribute wNrMsg
		U32 width;		//## attribute wMsgSize
		U16 index_last;		//## attribute wLastMessageIndex
		U16 index_first;		//## attribute wFirstMessageIndex
		U16 msg_count;		//## attribute lMessageCount
	};

	//## type MailboxAttributeType
	typedef MailboxAttributeTag MailboxAttributeType;

	//## type MailboxMessageTag
	struct MailboxMessageTag {
		U32 len;		//## attribute wMsgLength
		S8 msg;		//## attribute cMsgBegin
	};

	//## type MailboxMessageType
	typedef MailboxMessageTag MailboxMessageType;

	class CSrMbxPrivate{
	public:
		CSrMbxPrivate();
		~CSrMbxPrivate();
		Boolean create(S8 * mailbox_name, U16 depth, U32 width);
		Boolean open(S8 * mailbox_name);
		//## operation send(char*,axn_word)
		Boolean send(S8* msg, U32 len, U32 wait_ms = NO_TIME_OUT);

		Boolean vSend(S8* msg, U32 len, U32 wait_ms = NO_TIME_OUT);
		//## operation recv(char*,axn_uint *,axn_long)
		U32 recv(S8* buffer, U32 buffersize, U32 wait_ms = NO_TIME_OUT);

		//## operation close()
		Boolean close();
	private:
		Boolean createShm(S8 * mailbox_name, U16 depth, U32 width);
		S8* addPrefixToMbxName(S8 * mailbox_name);
		Boolean setSecureList(HANDLE secure_obj);
		S8* mapViewOfFile(U32 len);
		Boolean createSendMsgQueue();
		Boolean createRcvMsgQueue();
		Boolean createMutex();
		Boolean openSendMsgQueue();
		Boolean openRcvMsgQueue();
		Boolean openMutex();
		void lockQueue();
		void unlockQueue();
		Boolean checkSendParamIsValid(S8* msg, U32 len, U32 wait_ms = NO_TIME_OUT);
		Boolean checkIsSkipSend(U32 wait_ms = NO_TIME_OUT);
	private:
		// attributes
		HANDLE m_hmailbox;
		HANDLE m_hmutex;
		HANDLE m_hmsgqueue_send;
		HANDLE m_hmsgqueue_rcv;
		S8 m_mailbox_name[MAX_RESOURCE_NAME_LENGTH];
		S8 m_temp_mailbox_name[MAX_RESOURCE_NAME_LENGTH + sizeof("Global")];
		U16 m_depth;
		U32 m_width;
		S8* m_msgbuffer;
		MailboxAttributeType * m_mailbox_attribute;		//## attribute ptMailboxAttribute
	};

	CSrMbxPrivate::CSrMbxPrivate()
	{
		m_hmailbox = NULL;
		m_hmutex = NULL;
		m_hmsgqueue_send = NULL;
		m_hmsgqueue_rcv = NULL;
		memset(m_mailbox_name, 0, sizeof(m_mailbox_name));
		memset(m_temp_mailbox_name, 0, sizeof(m_temp_mailbox_name));
		m_depth = 0;
		m_width = 0;
		m_msgbuffer = NULL;
		m_mailbox_attribute = NULL;
	}
	CSrMbxPrivate::~CSrMbxPrivate()
	{
		this->close();
	}
	Boolean CSrMbxPrivate::create(S8 * mailbox_name, U16 depth, U32 width)
	{
		if (createShm(mailbox_name, depth, width) != SR_TRUE)
		{
			return SR_FALSE;
		}
		S8* p = mapViewOfFile(depth * (width + sizeof(MailboxMessageType)) + sizeof(MailboxAttributeType));
		if (p != NULL)
		{
			m_mailbox_attribute = (MailboxAttributeType*)p;
			m_mailbox_attribute->depth = depth;
			m_mailbox_attribute->width = width;
			this->m_depth = depth;
			this->m_width = width;
		}
		if (createSendMsgQueue() != SR_TRUE)
		{
			return SR_FALSE;
		}
		if (createRcvMsgQueue() != SR_TRUE)
		{
			return SR_FALSE;
		}
		if (createMutex() != SR_TRUE)
		{
			return SR_FALSE;
		}
		int i;
		lockQueue();
		m_mailbox_attribute->index_last = 0;
		m_mailbox_attribute->index_first = 0;
		m_mailbox_attribute->msg_count = 0;
		m_msgbuffer = (S8*)&m_mailbox_attribute[1];
		for (i = 0; i < (S32)(depth * (width + sizeof(MailboxMessageType))); i++)
		{
			m_msgbuffer[i] = 1;
		}
		unlockQueue();
		return SR_TRUE;
	}
	Boolean CSrMbxPrivate::open(S8 * mailbox_name)
	{
		S8* temp_mailbox_name = addPrefixToMbxName(mailbox_name);
		m_hmailbox = OpenFileMapping(FILE_MAP_ALL_ACCESS, TRUE, temp_mailbox_name);
		if (m_hmailbox == NULL){
			return SR_FALSE;
		}
		memcpy(m_mailbox_name, mailbox_name, min(sizeof(m_mailbox_name), strlen(mailbox_name)));
		S8* p = mapViewOfFile(sizeof(MailboxAttributeType));
		if (p != NULL)
		{
			m_mailbox_attribute = (MailboxAttributeType*)p;
			this->m_depth = m_mailbox_attribute->depth;
			this->m_width = m_mailbox_attribute->width;
		}
		if (UnmapViewOfFile(m_mailbox_attribute) == FALSE) // disconnect to shared memory
		{
			CloseHandle(m_hmailbox);
			m_hmailbox = NULL;
			m_mailbox_attribute = 0;
			return SR_FALSE;
		}
		S8* p1 = mapViewOfFile(this->m_depth * (this->m_width + sizeof(MailboxMessageType)) + sizeof(MailboxAttributeType));
		if (p1 != NULL)
		{
			m_mailbox_attribute = (MailboxAttributeType*)p1;
			m_msgbuffer = (S8*)&m_mailbox_attribute[1];
		}
		if (openSendMsgQueue() != SR_TRUE)
		{
			return SR_FALSE;
		}
		if (openRcvMsgQueue() != SR_TRUE)
		{
			return SR_FALSE;
		}
		if (openMutex() != SR_TRUE)
		{
			return SR_FALSE;
		}
		return SR_TRUE;
	}
	Boolean  CSrMbxPrivate::vSend(S8* msg, U32 len, U32 wait_ms /*= NO_TIME_OUT*/)
	{
		U16 true_len = getVmsgLen(msg, len);
		MailboxMessageType* pmessage_actual;
		if (checkSendParamIsValid(msg, true_len, wait_ms) != SR_TRUE)
		{
			return SR_FALSE;
		}
		if (checkIsSkipSend(wait_ms) == SR_TRUE)
		{
			return SR_FALSE;
		}
		// point to the message begin
		pmessage_actual = (MailboxMessageType*)&m_msgbuffer[m_mailbox_attribute->index_last
			* (sizeof(U32) + this->m_width)];
		// write buffer length before the message
		pmessage_actual->len = true_len;
		// copy the buffer in the shared memory
		S8* base_addr = &pmessage_actual->msg;
		copyVmsg(base_addr, (S8*)msg, len);
		// increment message index
		(m_mailbox_attribute->index_last)++;
		// turn around index count if necessary
		if (m_mailbox_attribute->index_last >= m_depth)
		{
			m_mailbox_attribute->index_last = 0;
		}
		// increment message count
		(m_mailbox_attribute->msg_count)++;
		// set event signaled for waiting receiver
		if (SetEvent(m_hmsgqueue_send) == NULL)
		{
			unlockQueue();
			return SR_FALSE;
		}
		unlockQueue();
		return SR_TRUE;
	}
	Boolean CSrMbxPrivate::send(S8* msg, U32 len, U32 wait_ms /*= NO_TIME_OUT*/)
	{
		Boolean rc = SR_FALSE;
		MailboxMessageType* pmessage_actual;
		if (checkSendParamIsValid(msg, len, wait_ms) != SR_TRUE)
		{
			return SR_FALSE;
		}
		if (checkIsSkipSend(wait_ms) == SR_TRUE)
		{
			return SR_FALSE;
		}
		// point to the message begin
		pmessage_actual = (MailboxMessageType*)&m_msgbuffer[m_mailbox_attribute->index_last
			* (sizeof(U32) + this->m_width)];
		// write buffer length before the message
		pmessage_actual->len = len;
		// copy the buffer in the shared memory
		memcpy(&(pmessage_actual->msg), msg, len);
		// increment message index
		(m_mailbox_attribute->index_last)++;
		// turn around index count if necessary
		if (m_mailbox_attribute->index_last >= m_depth)
		{
			m_mailbox_attribute->index_last = 0;
		}
		// increment message count
		(m_mailbox_attribute->msg_count)++;
		// set event signaled for waiting receiver
		if (SetEvent(m_hmsgqueue_send) == NULL)
		{
			unlockQueue();
			return SR_FALSE;
		}
		unlockQueue();
		return SR_TRUE;
	}
	U32 CSrMbxPrivate::recv(S8* buffer, U32 buffersize, U32 wait_ms /*= NO_TIME_OUT*/)
	{
		//#[ operation recv(char*,axn_uint *,axn_long)
		DWORD wait;
		U32 rc = 0;
		MailboxMessageType* pmessage_actual;
		if ((m_mailbox_attribute == NULL))
		{
			return 0;
		}
		if (m_msgbuffer == NULL)
		{
			return 0;
		}
		lockQueue();
		while (m_mailbox_attribute->msg_count <= 0)
		{
			if (ResetEvent(m_hmsgqueue_send) == NULL)
			{
				unlockQueue();
				return 0;
			}
			unlockQueue();
			wait = WaitForSingleObject(m_hmsgqueue_send, (DWORD)wait_ms);      // wait for timeout
			switch (wait)
			{
			case WAIT_FAILED:
			{
				return 0;
				//break ;
			}
			case WAIT_TIMEOUT:
			{
				return 0;
			}
			default:
			{
				break;             // no error
			}
			}
			// try again
			lockQueue();
		}
		pmessage_actual = (MailboxMessageType*)&m_msgbuffer[m_mailbox_attribute->index_first * (sizeof(U32) + m_width)];
		U32 len = pmessage_actual->len;
		rc = min(len, buffersize);
		memcpy(buffer, &pmessage_actual->msg, rc);
		(m_mailbox_attribute->msg_count)--;
		(m_mailbox_attribute->index_first)++;
		if (m_mailbox_attribute->index_first >= m_mailbox_attribute->depth)
		{
			m_mailbox_attribute->index_first = 0;
		}
		if (SetEvent(m_hmsgqueue_rcv) == NULL)
		{
			unlockQueue();
			return 0;
		}
		unlockQueue();
		return rc;
	}
	Boolean CSrMbxPrivate::close()
	{
		//#[ operation close()
		BOOL bool_result = FALSE;
#ifdef DEBUGMBX
		_tprintf(TEXT("sr_status CAxnMbxNT::Close()\n"));
#endif
		if (m_mailbox_attribute != NULL)
		{
			bool_result = UnmapViewOfFile(m_mailbox_attribute);
			m_mailbox_attribute = NULL;
		}
		if (m_hmailbox != NULL)
		{
			bool_result = CloseHandle(m_hmailbox) || bool_result;
			m_hmailbox = NULL;
		}
		if (m_hmsgqueue_send != NULL)
		{
			bool_result = CloseHandle(m_hmsgqueue_send) || bool_result;
			m_hmsgqueue_send = NULL;
		}
		if (m_hmsgqueue_rcv != NULL)
		{
			bool_result = CloseHandle(m_hmsgqueue_rcv) || bool_result;
			m_hmsgqueue_rcv = NULL;
		}
		if (m_hmutex != NULL)
		{
			bool_result = CloseHandle(m_hmutex) || bool_result;
			m_hmutex = 0;
		}
		if (bool_result == FALSE)
		{
#ifdef DEBUGMBX
			_tprintf(TEXT(" !!! error closing mailbox\n"));
#endif
			return SR_FALSE;
		}
		else
		{
			m_mailbox_name[0] = 0;
			return SR_TRUE;
		}
		//#]
	}
	Boolean CSrMbxPrivate::checkSendParamIsValid(S8* msg, U32 len, U32 wait_ms /*= NO_TIME_OUT*/)
	{
		if (m_mailbox_attribute == NULL)
		{
#ifdef DEBUGMBX
			_tprintf(TEXT("###Error CAxnMbxNT::Send Fail as Attr NONE\n"));
#endif
			return SR_FALSE;
		}
		if ((msg == NULL) || (len>m_mailbox_attribute->width))
		{
			return SR_FALSE;
		}
		if (m_msgbuffer == NULL)
		{
			return SR_FALSE;
		}
		return SR_TRUE;
	}
	Boolean CSrMbxPrivate::checkIsSkipSend(U32 wait_ms /*= NO_TIME_OUT*/)
	{
		U32 wait;
		Boolean skip_send = SR_FALSE;
		lockQueue();
		while (m_mailbox_attribute->msg_count >= m_depth)
		{
			if (0 == wait_ms)//NO Timeout
			{
				unlockQueue();
				skip_send = SR_TRUE;
				break;
			}
			if (ResetEvent(m_hmsgqueue_rcv) == NULL)
			{
				unlockQueue();
				skip_send = SR_TRUE;
				break;
			}
			unlockQueue();
			wait = WaitForSingleObject(m_hmsgqueue_rcv, (DWORD)wait_ms);    // waits indefinitely 
			if (wait == WAIT_FAILED)
			{
				skip_send = SR_TRUE;
				break;
			}
			if (WAIT_TIMEOUT == wait)
			{
				skip_send = SR_TRUE;
				break;
			}
			// try again
			lockQueue();
		}
		return skip_send;
	}
	Boolean CSrMbxPrivate::createShm(S8 * mailbox_name, U16 depth, U32 width)
	{
		S8* temp_mailbox_name = addPrefixToMbxName(mailbox_name);
		m_hmailbox = CreateFileMapping(
			(HANDLE)INVALID_HANDLE_VALUE // handle to file in system page file
			, NULL
			, PAGE_READWRITE
			, 0
			, depth * (width + sizeof(MailboxMessageType)) + sizeof(MailboxAttributeType)
			, temp_mailbox_name);
		if (m_hmailbox == NULL){
			return SR_FALSE;
		}
		if (setSecureList(m_hmailbox) != SR_TRUE)
		{
			CloseHandle(m_hmailbox);
			m_hmailbox = NULL;
			return SR_FALSE;
		}
		if (GetLastError() == ERROR_ALREADY_EXISTS)
		{
			CloseHandle(m_hmailbox);
			m_hmailbox = NULL;
			return (SR_FALSE);
		}
		memcpy(m_mailbox_name, mailbox_name, min(sizeof(m_mailbox_name), strlen(mailbox_name)));
		return SR_TRUE;
	}
	S8* CSrMbxPrivate::addPrefixToMbxName(S8 * mailbox_name)
	{
		memset(m_temp_mailbox_name, 0x00, sizeof(m_temp_mailbox_name));
		memcpy(m_temp_mailbox_name, "Global", sizeof("Global"));
		m_temp_mailbox_name[sizeof("Global")] = '\0';
		U64 len= strlen(m_temp_mailbox_name);
		strcat_s(m_temp_mailbox_name,mailbox_name);
		return(m_temp_mailbox_name);
	}
	Boolean CSrMbxPrivate::setSecureList(HANDLE secure_obj)
	{
		Boolean rc = SR_FALSE;
		if (secure_obj == NULL)
		{
			return SR_FALSE;
		}
		PSECURITY_DESCRIPTOR psd;
		psd = (PSECURITY_DESCRIPTOR)LocalAlloc(LPTR, SECURITY_DESCRIPTOR_MIN_LENGTH);
		if (psd == NULL)
		{
			return SR_FALSE;
		}
		if (!InitializeSecurityDescriptor(psd, SECURITY_DESCRIPTOR_REVISION))
		{
			return SR_FALSE;
		}
		if (!SetSecurityDescriptorDacl(psd,TRUE, (PACL)NULL,FALSE))
		{
			return SR_FALSE;
		}
		if (!SetKernelObjectSecurity(secure_obj,DACL_SECURITY_INFORMATION,psd))
		{
			if (GetLastError() == ERROR_CALL_NOT_IMPLEMENTED)
			{
				LocalFree((HLOCAL)psd);
				return (SR_TRUE);
			}
			else
			{
				return SR_FALSE;
			}
		}
		LocalFree((HLOCAL)psd);
		return (SR_TRUE);
	}

	S8*  CSrMbxPrivate::mapViewOfFile(U32 len)
	{
		S8* p = NULL;
		p = (S8*)MapViewOfFile(m_hmailbox, FILE_MAP_ALL_ACCESS, 0, 0, len);
		if (p == NULL)
		{
			(void)CloseHandle(m_hmailbox);
			m_hmailbox = NULL;
			return NULL;
		}
		return p;
	}

	Boolean CSrMbxPrivate::createSendMsgQueue()
	{
		memset(m_temp_mailbox_name, 0x00, sizeof(m_temp_mailbox_name));
		memcpy(m_temp_mailbox_name, m_mailbox_name, strlen(m_mailbox_name));
		strcat_s(m_temp_mailbox_name, "_event_send");
		m_hmsgqueue_send = CreateEvent(NULL, TRUE, TRUE, m_temp_mailbox_name);
#ifdef DEBUGMBX
		printf("createSendMsgQueue is %lld.\n", m_hmsgqueue_send);
#endif
		if (m_hmsgqueue_send == NULL)
		{
			(void)UnmapViewOfFile(m_mailbox_attribute);
			CloseHandle(m_hmailbox);
			m_hmailbox = NULL;
			m_mailbox_attribute = NULL;
			return SR_FALSE;
		}
		if (setSecureList(m_hmsgqueue_send) != SR_TRUE)
		{
			UnmapViewOfFile(m_mailbox_attribute);
			CloseHandle(m_hmailbox);
			CloseHandle(m_hmsgqueue_send);
			m_hmailbox = NULL;
			m_hmsgqueue_send = NULL;
			m_mailbox_attribute = NULL;
			return SR_FALSE;
		}
		return SR_TRUE;
	}
	Boolean CSrMbxPrivate::createRcvMsgQueue()
	{
		memset(m_temp_mailbox_name, 0x00, sizeof(m_temp_mailbox_name));
		memcpy(m_temp_mailbox_name, m_mailbox_name, strlen(m_mailbox_name));
		strcat_s(m_temp_mailbox_name, "_event_rcv");
		m_hmsgqueue_rcv = CreateEvent(NULL, TRUE, TRUE, m_temp_mailbox_name);
		if (m_hmsgqueue_rcv == NULL)
		{
			UnmapViewOfFile(m_mailbox_attribute);
			CloseHandle(m_hmailbox);
			CloseHandle(m_hmsgqueue_send);
			m_hmailbox = NULL;
			m_hmsgqueue_send = NULL;
			m_mailbox_attribute = NULL;
			return SR_FALSE;
		}
		if (setSecureList(m_hmsgqueue_rcv) != SR_TRUE)
		{
			UnmapViewOfFile(m_mailbox_attribute);
			CloseHandle(m_hmailbox);
			CloseHandle(m_hmailbox);
			CloseHandle(m_hmsgqueue_rcv);
			m_hmailbox = NULL;
			m_hmsgqueue_send = NULL;
			m_hmsgqueue_rcv = NULL;
			m_mailbox_attribute = NULL;
			return SR_FALSE;
		}
		return SR_TRUE;
	}
	Boolean CSrMbxPrivate::createMutex()
	{
		memset(m_temp_mailbox_name, 0x00, sizeof(m_temp_mailbox_name));
		memcpy(m_temp_mailbox_name, m_mailbox_name, strlen(m_mailbox_name));
		strcat_s(m_temp_mailbox_name, "_mutex");
		m_hmutex = CreateMutex(NULL, FALSE, m_temp_mailbox_name);
		if (m_hmutex == NULL)
		{
			UnmapViewOfFile(m_mailbox_attribute);
			CloseHandle(m_hmailbox);
			CloseHandle(m_hmailbox);
			CloseHandle(m_hmsgqueue_rcv);
			m_hmailbox = NULL;
			m_hmsgqueue_send = NULL;
			m_hmsgqueue_rcv = NULL;
			m_mailbox_attribute = NULL;
			return SR_FALSE;
		}
		if (setSecureList(m_hmutex) != SR_TRUE)
		{
			UnmapViewOfFile(m_mailbox_attribute);
			CloseHandle(m_hmailbox);
			CloseHandle(m_hmailbox);
			CloseHandle(m_hmsgqueue_rcv);
			CloseHandle(m_hmutex);
			m_hmailbox = NULL;
			m_hmsgqueue_send = NULL;
			m_hmsgqueue_rcv = NULL;
			m_hmutex = NULL;
			m_mailbox_attribute = NULL;
			return SR_FALSE;
		}
		return SR_TRUE;
	}
	Boolean CSrMbxPrivate::openSendMsgQueue()
	{
		memset(m_temp_mailbox_name, 0x00, sizeof(m_temp_mailbox_name));
		memcpy(m_temp_mailbox_name, m_mailbox_name, strlen(m_mailbox_name));
		strcat_s(m_temp_mailbox_name, "_event_send");
		m_hmsgqueue_send = OpenEvent(EVENT_MODIFY_STATE | SYNCHRONIZE, FALSE, m_temp_mailbox_name);  // CreateEvent(NULL, TRUE, TRUE, m_temp_mailbox_name);
#ifdef DEBUGMBX
		printf("openSendMsgQueue is %lld.\n", m_hmsgqueue_send);
#endif
		if (m_hmsgqueue_send == NULL)
		{
			(void)UnmapViewOfFile(m_mailbox_attribute);
			CloseHandle(m_hmailbox);
			m_hmailbox = NULL;
			m_mailbox_attribute = NULL;
			return SR_FALSE;
		}
		return SR_TRUE;
	}
	Boolean CSrMbxPrivate::openRcvMsgQueue()
	{
		memset(m_temp_mailbox_name, 0x00, sizeof(m_temp_mailbox_name));
		memcpy(m_temp_mailbox_name, m_mailbox_name, strlen(m_mailbox_name));
		strcat_s(m_temp_mailbox_name, "_event_rcv");
		m_hmsgqueue_rcv = OpenEvent(EVENT_MODIFY_STATE | SYNCHRONIZE, FALSE, m_temp_mailbox_name);  // CreateEvent(NULL, TRUE, TRUE, m_temp_mailbox_name);
		if (m_hmsgqueue_rcv == NULL)
		{
			UnmapViewOfFile(m_mailbox_attribute);
			CloseHandle(m_hmailbox);
			CloseHandle(m_hmsgqueue_send);
			m_hmailbox = NULL;
			m_hmsgqueue_send = NULL;
			m_mailbox_attribute = NULL;
			return SR_FALSE;
		}
		return SR_TRUE;
	}
	Boolean CSrMbxPrivate::openMutex()
	{
		memset(m_temp_mailbox_name, 0x00, sizeof(m_temp_mailbox_name));
		memcpy(m_temp_mailbox_name, m_mailbox_name, strlen(m_mailbox_name));
		strcat_s(m_temp_mailbox_name, "_mutex");
		m_hmutex = OpenMutex(SYNCHRONIZE, FALSE, m_temp_mailbox_name);//CreateMutex(NULL, FALSE, m_temp_mailbox_name);
		if (m_hmutex == NULL)
		{
			UnmapViewOfFile(m_mailbox_attribute);
			CloseHandle(m_hmailbox);
			CloseHandle(m_hmailbox);
			CloseHandle(m_hmsgqueue_rcv);
			m_hmailbox = NULL;
			m_hmsgqueue_send = NULL;
			m_hmsgqueue_rcv = NULL;
			m_mailbox_attribute = NULL;
			return SR_FALSE;
		}
		return SR_TRUE;
	}


	void CSrMbxPrivate::lockQueue()
	{
		U32 wait;
		// check mailbox is initialized
		if (m_mailbox_attribute == NULL)
		{
#ifdef DEBUGMBX
			_tprintf( TEXT("!!! error :unable to lock queue \n") );
#endif
		}
		// wait for mutex variable
		wait = WaitForSingleObject(m_hmutex,(DWORD)NO_TIME_OUT);    // waits indefinitely
		if ((wait == WAIT_FAILED) || (wait == WAIT_ABANDONED))
		{
#ifdef DEBUGMBX
			_tprintf(TEXT("!!! error : waiting on mutex variable \n"));
#endif
		}
#ifdef DEBUGMBX
		_tprintf(TEXT(" ... done \n"));
#endif
	}
	void CSrMbxPrivate::unlockQueue()
	{
		//#[ operation unlockQueue()
		if (m_mailbox_attribute == NULL)
		{
#ifdef DEBUGMBX
			_tprintf(TEXT("!!! error :unable to lock queue \n"));
#endif
		}

		if (ReleaseMutex(m_hmutex) == FALSE)
		{
#ifdef DEBUGMBX
			_tprintf(TEXT("!!! error : unable to release mutex variable"));
#endif
		}

#ifdef DEBUGMBX
		_tprintf(TEXT(" ... done  "));
		//_tprintf( TEXT("GetLastError %d \n"), (int)GetLastError () );
#endif
		//#]
	}






	CSrMbx::CSrMbx()
	{
		m_p = new CSrMbxPrivate();
	}


	CSrMbx::~CSrMbx()
	{
		if (m_p != NULL)
		{
			delete m_p;
		}
	}

	Boolean CSrMbx::create(S8 * mailbox_name, U16 depth, U32 width)
	{
		return m_p->create(mailbox_name, depth, width);
	}
	Boolean CSrMbx::open(S8 * mailbox_name)
	{
		return m_p->open(mailbox_name);
	}
	Boolean CSrMbx::send(S8* msg, U32 len, U32 wait_ms /*= NO_TIME_OUT*/)
	{
		return m_p->send(msg, len, wait_ms);
	}
	Boolean CSrMbx::vSend(S8* msg, U32 len, U32 wait_ms /*= NO_TIME_OUT*/)
	{
		return m_p->vSend(msg, len, wait_ms);
	}
	U32 CSrMbx::recv(S8* buffer, U32 buffersize, U32 wait_ms /*= NO_TIME_OUT*/)
	{
		return m_p->recv(buffer, buffersize, wait_ms);
	}
	Boolean  CSrMbx::close()
	{
		return m_p->close();
	}
}
