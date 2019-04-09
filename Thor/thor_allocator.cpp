/********************************************************************************
 * File Name:
 *	  thor_allocator.cpp
 *
 * Description:
 *	  Provides overloads for common memory allocators and deleters
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#include <Thor/allocator.hpp>

#if defined( USING_FREERTOS ) && defined( __cplusplus )
#include "FreeRTOS.h"
#include "portable.h"

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
