/********************************************************************************
 * File Name:
 *	  sram.hpp
 *
 * Description:
 *	  Interface to the internal SRAM of the Thor MCU
 *
 * 2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_INTERNAL_SRAM_HPP
#define THOR_INTERNAL_SRAM_HPP

/* C++ Includes */
#include <cstdint>

/* Thor Includes */
#include <Thor/headers.hpp>

#if defined( THOR_STM32HAL_DRIVERS ) && ( THOR_STM32HAL_DRIVERS == 1 )

namespace Thor
{
  namespace Memory
  {
    class InternalSRAM : public Chimera::Modules::Memory::Device
    {
    public:
      InternalSRAM();
      ~InternalSRAM();

      Chimera::Status_t write( const size_t address, const uint8_t *const data, const size_t length ) final override;

      Chimera::Status_t read( const size_t address, uint8_t *const data, const size_t length ) final override;

      Chimera::Status_t erase( const size_t address, const size_t length ) final override;

      Chimera::Status_t writeCompleteCallback( const Chimera::Function::void_func_uint32_t func ) final override;

      Chimera::Status_t readCompleteCallback( const Chimera::Function::void_func_uint32_t func ) final override;

      Chimera::Status_t eraseCompleteCallback( const Chimera::Function::void_func_uint32_t func ) final override;

      bool isInitialized() final override;

    private:
    };
  }    // namespace Memory
}    // namespace Thor

#endif /* THOR_INTERNAL_SRAM_HPP */

#endif 