/*****************************************************************************/
/*                                                                           */
/*              Copyright (c) SURGERII. All rights reserved                  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* File name: *******.cpp            Version: 1.0                            */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Type: class definition            Programming language: C++               */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Author: Dai Liang                 Date: 10.05.2019                        */
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
#include "sr_dlhelper.h"
#include <stdio.h>
#include <Windows.h>

namespace sr
{
	void* openDll(void* const dl_name)
	{
		void* handle = NULL;
		char full_name[256];
		sprintf((char*)full_name, "%s%s", dl_name, ".dll");
		handle = LoadLibrary((char*)full_name);
		return handle;
	}

	void* getFuncAddr(void* const handle, void* const func_name)
	{
		return GetProcAddress((HMODULE)handle, (char*)func_name);
	}

	void  closeDll(void* const handle)
	{
		FreeLibrary((HMODULE)handle);
	}
}
