/********************************************************************************
 * File Name:
 *	  thor_sram.cpp
 *
 * Description:
 *	  Interface to the internal SRAM of the Thor MCU
 *
 * 2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Thor Includes */
#include <Thor/cfg>
//#include <Thor/sram.hpp>

#if defined( THOR_STM32HAL_DRIVERS ) && ( THOR_STM32HAL_DRIVERS == 1 )
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
      return Chimera::Status::NOT_SUPPORTED;
    }

    Chimera::Status_t InternalSRAM::read( const size_t address, uint8_t *const data, const size_t length )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    Chimera::Status_t InternalSRAM::erase( const size_t address, const size_t length )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    Chimera::Status_t InternalSRAM::writeCompleteCallback( const Chimera::Function::void_func_uint32_t func )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    Chimera::Status_t InternalSRAM::readCompleteCallback( const Chimera::Function::void_func_uint32_t func )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    Chimera::Status_t InternalSRAM::eraseCompleteCallback( const Chimera::Function::void_func_uint32_t func )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    bool InternalSRAM::isInitialized()
    {
      return false;
    }
  }
}

#endif