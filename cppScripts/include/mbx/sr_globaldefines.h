/*****************************************************************************/
/*                                                                           */
/*              Copyright (c) SURGERII. All rights reserved                  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* File name: sr_globaldefines.h     Version: 1.0                            */
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
#ifndef _SR_GLOBALDEFINE_H_
#define _SR_GLOBALDEFINE_H_
#include "sr_portable.h"
#include <stddef.h>
const Boolean SR_TRUE = 1U;
const Boolean SR_FALSE = 0U;

namespace sr
{
typedef S32 Status;

const U32 NO_TIME_OUT = ~0U;

#define SR_MAX_NAME_LENGTH 32

const Status SR_SUCCESS = 0;
const Status SR_FAILED = -1;
const Status SR_ERR_TIMEOUT = -2;
const Status SR_ERR_QUEUE_FULL = -3;
const Status SR_ERR_PARA_ILLEGAL = -4;
const Status SR_ERR_BUFF_INSUFFCIENT = -5;

struct VMsg
{
	S8* msg ;
	U16 len ;
};
}
#endif