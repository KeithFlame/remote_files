#include <time.h>
#include <ctime>
#include <stdio.h>
#include "sr_portable.h"
//#include <stdlib.h>

namespace sr
{
	char *  sr_itoa( int value, char * buffer, int radix )
	{
		return _itoa_s(value, buffer, radix);
	}
}

