/*****************************************************************************/
/*                                                                           */
/*              Copyright (c) SURGERII. All rights reserved                  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* File name: sr_comm.cpp                  Version: 1.0                      */
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
#include "sr_common.h"
#include <Windows.h>
namespace sr
{
	void sleep_ms(U32 milli_seconds)
	{
		Sleep(milli_seconds);
	}
	U16 getVmsgLen(S8* msg, const U32 len){
		U16 rc = 0;
		if (msg != NULL){
			VMsg* v_msg = (VMsg*)msg;
			for (U32 i = 0; i < len; i++){
				rc = rc + v_msg[i].len;
			}
		}
		return rc;
	}
	void copyVmsg(S8* base_addr, S8* vmsg, U32 len)
	{
		VMsg* v_msg = (VMsg*)vmsg;
		for (U16 i = 0; i < len; i++){
			memcpy(base_addr, v_msg[i].msg, v_msg[i].len);
			base_addr = base_addr + v_msg[i].len;
		}
	}

	void shutdownSystem(){
		system("shutdown -s -f -t 0");
	}
}