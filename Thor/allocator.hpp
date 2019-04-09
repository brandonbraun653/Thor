/********************************************************************************
 * File Name:
 *	  allocator.hpp
 *
 * Description:
 *	  Provides overloads for common memory allocators and deleters
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_ALLOCATOR_HPP
#define THOR_ALLOCATOR_HPP

#include <Thor/preprocessor.hpp>

/*------------------------------------------------
Redirect the new/delete operators into the FreeRTOS
memory management functions. Without this, all hell
will break loose.
------------------------------------------------*/
#if defined( USING_FREERTOS ) && defined( __cplusplus )
#include "new"

void *operator new( size_t size );

void *operator new[]( size_t size );

void operator delete( void *p ) noexcept;

#endif /* USING_FREERTOS */

#endif /* THOR_ALLOCATOR_HPP*/
