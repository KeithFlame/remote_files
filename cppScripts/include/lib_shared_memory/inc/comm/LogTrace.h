#pragma once

#include <windows.h>
#include "DbgHelp.h"
#define LOG_TAG (char*)__FILE__,(unsigned long)__LINE__
#define SSME_LOG_CHANNEL "/ssme_log_channel"
#define PORT 2222 /*port for Log Manage*/
//%.19s.%hu
namespace SR
{
	typedef signed long long	sint64;
	typedef unsigned long long	uint64;
	typedef unsigned long		uint32;
	typedef signed long			sint32;
	typedef char				char_t;
	typedef unsigned char		uint8;
	typedef int					sint1632;

	//default remote record level
	const extern int debug_level_remote;
	//default local record level
	const extern int debug_level_local;
	//const extern char* logTag;
	//the local log file's path and name prefix
	//const extern char* FILE_PATH_HEAD;


	const sint1632 LOG_EMERG	= 0;	//    system is unusable 
	const sint1632 LOG_ALERT	= 1;	//    action must be taken immediately 
	const sint1632 LOG_CRIT		= 2;	//    critical conditions 
	const sint1632 LOG_ERR		= 3;	//    error conditions 
	const sint1632 LOG_WARNING	= 4;	//    warning conditions 
	const sint1632 LOG_NOTICE	= 5;	//    normal, but significant, condition 
	const sint1632 LOG_INFO		= 6;	//    informational message 
	const sint1632 LOG_DEBUG	= 7;	//    debug-level message



	//the depth of queue that used to record. If the application is very busy,
	//maybe the trace log miss if this depth was set too small.But the more lager the depth, the more memory it costs
	const int QUEUE_SZ = 1024;

	//the max length of every trace log to be recorded. It includes some SysLog protoc0l data.
	//Maybe the trace log will be trim if it's too long.
	//But the more longer the max length, the more memory it costs
	const int QUEUE_LENGTH = 256U;	


	

#define MAX_MODULE_NAME_LEN 32
#define MAX_ORIGIN_NAME_LEN 16
#define MAX_GEN_TIME_BUF_SZ 20
	/* log struct */
	typedef struct{
		uint8  logSeverity;
		char   origin[MAX_ORIGIN_NAME_LEN];  //Name of Program
		char   strModuleName[MAX_MODULE_NAME_LEN];//Happens on which module and line
		uint32 logLine;	
		char    GenTm[MAX_GEN_TIME_BUF_SZ];
		char   log[QUEUE_LENGTH];
	}logInfo_t;

	typedef struct{
		unsigned short commandPassword;
		int            remoteLevel;
		int            localLevel;
	}logM_t;

	/*------------------------------------------------------------------------------
	* Function: InitLogM
	* Brief:	Init the Log Manage Mechanism. This function is not necessary if you
	Wouldn't like to manage the Log Mechanism.
	* Param:	N.A.
	* Retval:	N.A.
	* History:  2016-2-5, Create
	* Author:	Gong Hui
	* Date:		2016-2-5
	------------------------------------------------------------------------------*/	
	extern void InitLogM();


	/*------------------------------------------------------------------------------
	* Function: Log
	* Brief:	The Interface used to call to record the trace log
	* Param:	level	: trace log's level
	*           pathName: File name the code call this function,you can give __FILE__
	*           _line   : line number the code call this funtion,you can give __LINE__
	*           fmt		: format output the trace message
	* Retval:	N.A.
	* History:  2016-2-5, Introduce messageQueue Mechanism in
	* Author:	Gong Hui
	* Date:		2013-02-16
	------------------------------------------------------------------------------*/
	extern void Log(const int level,
		char pathName[],
		const uint32 _line, 
		const char* const fmt, ...);
	
	extern void writeLog(const int level,
		char pathName[],
		const uint32 _line,
		const char* const fmt, va_list& argptr);

	extern char* CurTm(uint64& curMmS);

	//
	//写文件数据的定义
	#define FileWriter_MBX_NAME "/sr_FileWriter_mbx" 
	const int LogData_Length = 1024U;	
    #define Type_Hex 1
    #define Type_Txt 2
    #define FILE_CLOSE 1
    #define FILE_NOTCLOSE 0
	typedef struct{
		int logObjID;
		int length;
		char data[LogData_Length];
	}HexFile_Data;
	typedef struct{
		uint8 fileType;
		uint8 isClose;
		union
		{
			HexFile_Data Hexdata;
			logInfo_t Txtdata;
		}data;		
	}File_Data_frame_t;
	//write hex file
	extern void writeFile(int objID,int length,char* data);
	extern void CloseFile(int objID);
	//write txt file
	extern void writeFile(const int level,
		char pathName[],
		const uint32 _line,
		const char* const fmt, ...);

	extern void doWriteFile(const int level,
		char pathName[],
		const uint32 _line,
		const char* const fmt, va_list & argptr);

	extern  LONG WINAPI ExceptionFilter(LPEXCEPTION_POINTERS lpExceptionInfo);

}
