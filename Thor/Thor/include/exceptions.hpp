#pragma once
#ifndef EXCEPTIONS_H_
#define EXCEPTIONS_H_

#include <stdlib.h>
#include <string>


#define STRINGIZE_DETAIL(x) #x
#define STRINGIZE(x) STRINGIZE_DETAIL(x)
#define logError(msg) (__FILE__ " line " STRINGIZE(__LINE__) ": " msg)

extern void BasicErrorHandler(std::string err_msg);


#ifdef __cplusplus
extern "C" {
#endif

	void HardFault_Handler();
    void HardFault_HandlerC(unsigned long *hardfault_args) __attribute__((used));

#ifdef __cplusplus
}
#endif

#endif /*! EXCEPTIONS_H_*/
