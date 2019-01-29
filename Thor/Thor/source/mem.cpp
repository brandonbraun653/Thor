#include <Thor/include/mem.hpp>

#if defined( USING_FREERTOS ) && !defined( USING_ERPC ) && defined __cplusplus
#include "FreeRTOS.h"
#include "new"

void *operator new( size_t size )
{
  void *p = pvPortMalloc( size );

#ifdef __EXCEPTIONS
  if ( p == 0 )                // did pvPortMalloc succeed?
    throw std::bad_alloc();    // ANSI/ISO compliant behavior
#endif
  return p;
}

void *operator new[]( size_t size )
{
  void *p = pvPortMalloc( size );

#ifdef __EXCEPTIONS
  if ( p == 0 )                // did pvPortMalloc succeed?
    throw std::bad_alloc();    // ANSI/ISO compliant behavior
#endif
  return p;
}

void operator delete( void *p ) noexcept
{
  vPortFree( p );
}

#endif /* USING_FREERTOS */