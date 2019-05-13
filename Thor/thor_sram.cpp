/********************************************************************************
 * File Name:
 *	  thor_sram.cpp
 *
 * Description:
 *	  Interface to the internal SRAM of the Thor MCU
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Thor Includes */
#include <Thor/sram.hpp>


namespace Thor
{
  namespace Memory
  {
    InternalSRAM::InternalSRAM()
    {
    }

    InternalSRAM::~InternalSRAM()
    {
    }

    Chimera::Status_t InternalSRAM::write( const size_t address, const uint8_t *const data, const size_t length )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    Chimera::Status_t InternalSRAM::read( const size_t address, uint8_t *const data, const size_t length )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    Chimera::Status_t InternalSRAM::erase( const size_t address, const size_t length )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    Chimera::Status_t InternalSRAM::writeCompleteCallback( const Chimera::Function::void_func_uint32_t func )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    Chimera::Status_t InternalSRAM::readCompleteCallback( const Chimera::Function::void_func_uint32_t func )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    Chimera::Status_t InternalSRAM::eraseCompleteCallback( const Chimera::Function::void_func_uint32_t func )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    bool InternalSRAM::isInitialized()
    {
      return false;
    }
  }
}