#ifndef THOR_H_
#define THOR_H_

#include <Thor/headers.hpp>
#include <Thor/config.hpp>
#include <Thor/core.hpp>

namespace Thor
{
  void prjIncSysTick();

  size_t millis();
  void delayMilliseconds( const size_t ms );
  void delayMicroseconds( const size_t us );
}    // namespace Thor

#endif /* THOR_H_ */
