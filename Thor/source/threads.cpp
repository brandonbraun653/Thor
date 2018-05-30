#include <Thor/include/threads.hpp>

#include <boost/container/static_vector.hpp>

#ifdef USING_FREERTOS
using namespace Thor::Definitions::Threading;
	
namespace Thor
{
	namespace Threading
	{
		TaskHandle_t INIT_THREAD;
		static boost::container::static_vector<Thread_t, maxThreads> registeredThreads;

		/* Implements a simple timeout while waiting for a newly created thread to complete
		 * its initialization sequence and signal back to the init thread. */
		BaseType_t threadInitTimeout(TaskHandle_t* threadHandle)
		{
			volatile BaseType_t error = pdPASS;
			TickType_t lastTimeWoken = xTaskGetTickCount();
			uint32_t timeoutCounter = 0;

			while (!ulTaskNotifyTake(pdTRUE, 0))
			{
				vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(threadInitCheckDelay_ms));
				timeoutCounter += threadInitCheckDelay_ms;

				if (timeoutCounter > maxThreadInitTimeout_ms)
				{
					error = pdFAIL;
					break;
				}
			}

			return error;
		}

		void initThreads(void* arguments)
		{
			volatile BaseType_t error = pdPASS;
			Thread_t thread;

			/* Allow thread startup code to run and then halt */
			for (int i = 0; i < registeredThreads.size(); i++)
			{
				thread = registeredThreads[i];
				error = xTaskCreate(thread.func, thread.name, thread.stackDepth, thread.funcParams, thread.priority, thread.handle);
					
				if (error == pdPASS && threadInitTimeout(thread.handle) == pdPASS)
					registeredThreads[i].handle = thread.handle;
				else
				{
					/* If you get stuck here, it's because you did not call back to this thread after initialization code was
					 * completed. Place "sendMessageAndWait(INIT_THREAD, 1u);" after setup code and just before the infinite loop.
					 * Alternatively, call "signalThreadSetupComplete()".
					 */
					vTaskSuspendAll();
					while (1);
				}
			}

			/* Resume threads in the order which they were registered */
			for (int i = 0; i < registeredThreads.size(); i++)
				vTaskResume(registeredThreads[i].handle);

			/* Cleanly exit this thread */
			vTaskDelete(NULL);
		}

		Status startScheduler()
		{
			xTaskCreate(initThreads, "thor_init", 500, NULL, 1, &INIT_THREAD);
			vTaskStartScheduler();
		}

		BaseType_t addThread(TaskFunction_t threadFunc, const char* threadName, const uint16_t stackDepth, void* const threadFuncParams,
			UBaseType_t threadPriority, TaskHandle_t* const threadHandle)
		{
			volatile BaseType_t error = pdPASS;

			if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
				error = xTaskCreate(threadFunc, threadName, stackDepth, threadFuncParams, threadPriority, threadHandle);
			else
				registeredThreads.push_back({ threadFunc, threadName, stackDepth, threadFuncParams, threadPriority, threadHandle });

			return error;
		}

		BaseType_t addThread(Thread_t& thread)
		{
			return addThread(thread.func, thread.name, thread.stackDepth, thread.funcParams, thread.priority, thread.handle);
		}

		void deleteThread(TaskHandle_t task)
		{
			vTaskDelete(task);
		}

		BaseType_t signalThreadSetupComplete()
		{
			return sendMessageAndWait(INIT_THREAD, 1u);
		}

		BaseType_t sendMessageAndWait(TaskHandle_t task, const uint32_t msg)
		{
			if (task)
				return xTaskNotify(task, msg, eSetValueWithOverwrite);
			else
				return pdFAIL;
		}

		unsigned long queryStackMaxSize(TaskHandle_t task)
		{
			return uxTaskGetStackHighWaterMark(task);
		}

		unsigned long getTotalThreads()
		{
			return uxTaskGetNumberOfTasks();
		}
	}
}

#endif /* !USING_FREERTOS */