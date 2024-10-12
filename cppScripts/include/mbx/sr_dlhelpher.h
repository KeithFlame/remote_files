/*****************************************************************************/
/*                                                                           */
/*              Copyright (c) SURGERII. All rights reserved                  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* File name: *******.h              Version: 1.0                            */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Type: class definition            Programming language: C++               */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Author: Dai Liang                 Date:10.05.2019                         */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Description:                                                              */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
#if !defined(EA_8739289E_E70B_4a67_9CA1_5D135D4FB6BC__INCLUDED_)
#define EA_8739289E_E70B_4a67_9CA1_5D135D4FB6BC__INCLUDED_
namespace sr
{
	void* openDll(void* const dl_name);
	void* getFuncAddr(void* const handle, void* const func_name);
	void  closeDll(void* const handle);
}

#endif // !defined(EA_8739289E_E70B_4a67_9CA1_5D135D4FB6BC__INCLUDED_)