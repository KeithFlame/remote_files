/******************************************************************************
**                                                                           
**                Copyright (c) SURGERII. All rights reserved                  
**                                                                            
*******************************************************************************
**                                                                            
**    Description:    sr_string.cpp
**                 
******************************************************************************/

#include "sr_string.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

size_t sr_snprintf(char* const buf, size_t buf_size, const char * fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	size_t ret = vsnprintf(buf, buf_size - 1, fmt, ap);
	buf[ret] = '\0';
	va_end(ap);
	return ret;
}
