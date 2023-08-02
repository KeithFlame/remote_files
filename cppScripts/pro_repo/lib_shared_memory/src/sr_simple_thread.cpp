#include "sr_simple_thread.h"
#include <Windows.h>
//#include "sr_object_allocator.h"
namespace sr
{
	class SrTask
	{
	public:
		SrTask()
			:m_thread_handle(0)
		{

		}
		~SrTask()
		{
			deleteTask();
		}

		Boolean createTask(void* (*thread_fuc)(void*),
			void* const parm,
			const TaskPriority priority = MODERATE,
			const StackSize stacksize = MINI)
		{
			DWORD id = 0;
			m_thread_handle = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)thread_fuc, (LPVOID)(parm), NULL, &id);
			if (m_thread_handle == NULL)
			{
				return SR_FALSE;
			}
			return SR_TRUE;
		}

		void deleteTask()
		{
			if (m_thread_handle != NULL)
			{
				CloseHandle(m_thread_handle);
				m_thread_handle = NULL;
			}
		}

	private:
		HANDLE m_thread_handle;
	};


	THREAD_HANDLE createThread(void* (*thread_fuc)(void*),
		void* const parm,
		const TaskPriority priority /*= MODERATE*/,
		const StackSize stacksize /*= MINI*/)
	{
		SrTask* task;
		task = new SrTask();
		if (SR_TRUE != task->createTask(thread_fuc, parm, priority, stacksize))
		{
			delete task;
			return NULL;
		}
		return (THREAD_HANDLE)task;
	}
	Boolean exitThread(THREAD_HANDLE handle)
	{
		if (handle != NULL)
		{
			((SrTask*)handle)->deleteTask();
			return SR_TRUE;
		}
		return SR_FALSE;
	}
}
