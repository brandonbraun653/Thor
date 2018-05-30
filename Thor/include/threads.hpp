#pragma once
#ifndef THOR_THREADING_HPP
#define THOR_THREADING_HPP
	
#include <Thor/include/thor.hpp>

#ifdef USING_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/** @namespace Thor*/
namespace Thor
{
	/** @namespace Thor::Threading */
	namespace Threading
	{
		extern TaskHandle_t INIT_THREAD;

		struct Thread_t
		{
			TaskFunction_t func;
			const char* name;
			uint16_t stackDepth;
			void* funcParams;
			UBaseType_t priority;
			TaskHandle_t* handle;
		};

		/** Starts the FreeRTOS scheduler and initializes execution of all registered threads.
		 *  This implementation extends the basic FreeRTOS vTaskStartScheduler() function by
		 *	automatically supporting user setup code in each thread.
		 **/
		extern Thor::Definitions::Threading::Status startScheduler();

		/** Adds a new thread to the FreeRTOS kernel. If the scheduler has been started already, the
		 *	correct initialization sequence will be followed. Otherwise, the thread will be suspended
		 *	until startScheduler() has been called.
		 * 
		 *  @param[in]	 threadFunc
		 *	@param[in]   threadName
		 *  @param[in]   stackDepth
		 *  @param[in]   threadFuncParams
		 *  @param[in]   threadPriority
		 *  @param[out]  threadHandle		Generated task handle for reference 
		 **/
		extern BaseType_t addThread(
			TaskFunction_t threadFunc,
			const char* threadName,
			const uint16_t stackDepth,
			void* const threadFuncParams,
			UBaseType_t threadPriority,
			TaskHandle_t* const threadHandle);

		extern BaseType_t addThread(Thread_t& thread);

		/** Safely removes a thread from existence 
		 *	@param 
		 **/
		extern void deleteThread(TaskHandle_t task);

		extern BaseType_t signalThreadSetupComplete();

		extern BaseType_t sendMessageAndWait(TaskHandle_t task, const uint32_t msg);

		extern unsigned long queryStackMaxSize(TaskHandle_t task);

		extern unsigned long getTotalThreads();
	}
}
#endif /* !USING_FREERTOS */
#endif /* !THOR_THREADING_HPP */