#include <time.h>
#include <ctime>
#include <stdio.h>
#include <atltime.h>
#include "sr_portable.h"


using namespace std;

namespace sr
{
	void getFormattedTime(char * const date, int size)
	{
		time_t rawtime;
		struct tm timeinfo;
		time (&rawtime);
		localtime_s(&timeinfo, &rawtime);
		strftime(date, size, "%Y-%m-%d-%H-%M-%S", &timeinfo);
	}

	void getFormattedTime_ms(char * const date, U32 size)
	{
		SYSTEMTIME sysTime;
		char strTime[30];
		GetLocalTime(&sysTime);
		_stprintf_s(strTime, _T("%04d-%02d-%02d %02d:%02d:%02d:%03d"), sysTime.wYear, sysTime.wMonth,
			sysTime.wDay, sysTime.wHour, sysTime.wMinute, sysTime.wSecond, sysTime.wMilliseconds);

		memcpy(date, strTime, sizeof(strTime) < size ? sizeof(strTime) : size);
	}
}

