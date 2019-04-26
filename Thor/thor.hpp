#ifndef THOR_H_
#define THOR_H_

#include <Thor/headers.hpp>
#include <Thor/config.hpp>
#include <Thor/core.hpp>

extern void ThorInit();

#if defined( USING_CHIMERA )
/** A function Chimera expects to exist in order to initialize the host system properly */
extern void cSystemInit();
#endif

/** @namespace Thor */
namespace Thor
{
  uint32_t millis();
  void delayMilliseconds( uint32_t ms );
  void delayMicroseconds( uint32_t us );
}    // namespace Thor

#endif /* THOR_H_ */
