/********************************************************************************
 * File Name:
 *	  thor_flash.cpp
 *
 * Description:
 *	  Interface to the internal flash of the Thor MCU
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Thor Includes */
#include <Thor/flash.hpp>


namespace Thor
{
  namespace Memory
  {
    InternalFlash::InternalFlash()
    {
    }

    InternalFlash::~InternalFlash()
    {
    }

    Chimera::Status_t InternalFlash::write( const uint32_t address, const uint8_t *const data, const uint32_t length )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    Chimera::Status_t InternalFlash::read( const uint32_t address, uint8_t *const data, const uint32_t length )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    Chimera::Status_t InternalFlash::erase( const uint32_t address, const uint32_t length )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    Chimera::Status_t InternalFlash::writeCompleteCallback( const Chimera::Function::void_func_uint32_t func )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    Chimera::Status_t InternalFlash::readCompleteCallback( const Chimera::Function::void_func_uint32_t func )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    Chimera::Status_t InternalFlash::eraseCompleteCallback( const Chimera::Function::void_func_uint32_t func )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    bool InternalFlash::isInitialized()
    {
      return false;
    }
  }
}
