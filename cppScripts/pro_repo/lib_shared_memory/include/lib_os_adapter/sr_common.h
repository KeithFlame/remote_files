/*****************************************************************************/
/*                                                                           */
/*              Copyright (c) SURGERII. All rights reserved                  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* File name: sr_comm.h             Version: 1.0                             */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Type: class definition            Programming language: C++               */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Author: Dai Liang                 Date:15.03.2019                         */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Description:                                                              */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
#ifndef _SR_COMMON_H_
#define _SR_COMMON_H_
#include "sr_globaldefine.h"

namespace sr
{
void sleep_ms(U32 milli_seconds);
U16 getVmsgLen(S8* v_msg, U32 len);
void copyVmsg(S8* base_addr, S8* v_msg, U32 len);
void shutdownSystem();
}
#endif