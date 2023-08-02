
/*****************************************************************************/
/*                                                                           */
/*              Copyright (c) SURGERII. All rights reserved                  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* File name: sr_mbx.h              Version: 1.0                             */
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

#ifndef _SR_MBX_H_
#define _SR_MBX_H_
#include "sr_globaldefine.h"
namespace sr
{
	class CSrMbxPrivate;
	const U32 MAX_RESOURCE_NAME_LENGTH = 32;
	class CSrMbx
	{
	public:
		CSrMbx();
		~CSrMbx();
		Boolean create(S8 * mailbox_name, U16 depth, U32 width);
		Boolean open(S8 * mailbox_name);
		//## operation send(char*,axn_word)
		Boolean send(S8* msg, U32 len, U32 wait_ms = NO_TIME_OUT);
		//## operation send(char*,axn_word)
		Boolean vSend(S8* msg, U32 len, U32 wait_ms = NO_TIME_OUT);
		//## operation recv(char*,axn_uint *,axn_long)
		U32 recv(S8* buffer, U32 buffersize, U32 wait_ms = NO_TIME_OUT);

		//## operation close()
		Boolean close();
	private:
		CSrMbxPrivate* m_p;
	};

}
#endif /*SR_ALARM_H*/


