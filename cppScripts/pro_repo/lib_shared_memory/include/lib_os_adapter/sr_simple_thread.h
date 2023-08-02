#ifndef __SR_SIMPLE_THREAD_H__
#define __SR_SIMPLE_THREAD_H__
#include "sr_globaldefine.h"

namespace sr
{	
typedef U16 TaskPriority;
const U16 CFG_MAX_PRIORITIES = 20;
const TaskPriority REALTIME = (CFG_MAX_PRIORITIES - 1);
const TaskPriority HIGHEST = (CFG_MAX_PRIORITIES - 2);
const TaskPriority VERYHIGH = (CFG_MAX_PRIORITIES - 3);
const TaskPriority HIGH = (CFG_MAX_PRIORITIES - 4);
const TaskPriority MODERATE = (CFG_MAX_PRIORITIES - 5);
const TaskPriority LOWER = (CFG_MAX_PRIORITIES - 6);
const TaskPriority VERYLOW = (CFG_MAX_PRIORITIES - 7);
const TaskPriority LOWEST = 1;
typedef U32 StackSize;

const StackSize MINI = 0x300;
const StackSize SMALL = 0x600;
const StackSize MEDIUM = 0xC00;
const StackSize LARGE = 0x1000;

typedef void* THREAD_HANDLE;

THREAD_HANDLE createThread(void* (*thread_fuc)(void*),
                           void* const parm,
                           const TaskPriority priority = MODERATE,
                           const StackSize stacksize = MINI);
Boolean exitThread(THREAD_HANDLE handle);

}
#endif
