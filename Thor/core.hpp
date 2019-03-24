#ifndef CORE_H_
#define CORE_H_

#include <Thor/preprocessor.hpp>

#ifdef __cplusplus
extern "C"
{
#endif
  void SysTick_Handler();

#if defined( USING_FREERTOS )
  void vApplicationTickHook( void );
#endif
#ifdef __cplusplus
}
#endif

extern void ThorSystemClockConfig();


#endif /*! CORE_H_ */
