#ifndef CORE_H_
#define CORE_H_

#include <Thor/headers.hpp>
#include <Thor/preprocessor.hpp>
#include <Chimera/types/common_types.hpp>

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


static inline constexpr Chimera::Status_t convertHALStatus( const HAL_StatusTypeDef status )
{
  switch ( status )
  {
    case HAL_ERROR:
      return Chimera::CommonStatusCodes::FAIL;
      break;

    case HAL_BUSY:
      return Chimera::CommonStatusCodes::BUSY;
      break;

    case HAL_OK:
      return Chimera::CommonStatusCodes::OK;
      break;

    case HAL_TIMEOUT:
      return Chimera::CommonStatusCodes::TIMEOUT;
      break;

    default:
      return Chimera::CommonStatusCodes::UNKNOWN_ERROR;
      break;
  }
}

#endif /*! CORE_H_ */
