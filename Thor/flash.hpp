/********************************************************************************
 * File Name:
 *	  flash.hpp
 *
 * Description:
 *	  Interface to the internal flash of the Thor MCU
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_INTERNAL_FLASH_HPP
#define THOR_INTERNAL_FLASH_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/modules/memory/device.hpp>

/* Thor Includes */
#include <Thor/headers.hpp>

#if defined( THOR_STM32HAL_DRIVERS ) && ( THOR_STM32HAL_DRIVERS == 1 )
namespace Thor::Memory
{
  class InternalFlash : public Chimera::Modules::Memory::Device
  {
  public:
    InternalFlash();
    ~InternalFlash();

    Chimera::Status_t write( const size_t address, const uint8_t *const data, const size_t length ) final override;

    Chimera::Status_t read( const size_t address, uint8_t *const data, const size_t length ) final override;

    /**
     * Function deviates slightly from Chimera specification. See declaration.
     */
    Chimera::Status_t erase( const size_t address, const size_t length ) final override;

    Chimera::Status_t writeCompleteCallback( const Chimera::Function::void_func_uint32_t func ) final override;

    Chimera::Status_t readCompleteCallback( const Chimera::Function::void_func_uint32_t func ) final override;

    Chimera::Status_t eraseCompleteCallback( const Chimera::Function::void_func_uint32_t func ) final override;

    bool isInitialized() final override;

    /**
     *  Converts a STM32HAL flash error into the appropriate Chimera error
     *
     *  @param[in]  error         The error to convert
     *  @return Chimera::Status_t
     */
    static Chimera::Status_t convertHALError( const uint32_t error );

    /**
     *  Gets the number of bytes programmed in a given program type
     *
     *  @param[in]  TypeProgram   STM32HAL program parallelism type
     *  @return uint32_t
     */
    static uint32_t getPgmByteWidth( const uint32_t TypeProgram );

    /**
     *  Converts an address into a MCU specific flash sector number
     *
     *  @param[in]  address       The address to convert
     *  @return uint32_t
     */
    static uint32_t addressToSector( const uint32_t address );

  private:
    FLASH_EraseInitTypeDef eraseInit;
  };


  /**
   *  Device Specific System Flash Boundaries
   */
#if defined( STM32F446xx )
#define FLASH_MEMORY_MAPPING_INITIALIZED

  /* 16kB */
  static constexpr uint32_t FLASH_SECTOR_0_START_ADDR = 0x08000000;
  static constexpr uint32_t FLASH_SECTOR_0_END_ADDR   = 0x08003FFF;
  static constexpr uint32_t FLASH_SECTOR_0_SIZE       = FLASH_SECTOR_0_END_ADDR - FLASH_SECTOR_0_START_ADDR;

  /* 16kB */
  static constexpr uint32_t FLASH_SECTOR_1_START_ADDR = 0x08004000;
  static constexpr uint32_t FLASH_SECTOR_1_END_ADDR   = 0x08007FFF;
  static constexpr uint32_t FLASH_SECTOR_1_SIZE       = FLASH_SECTOR_1_END_ADDR - FLASH_SECTOR_1_START_ADDR;

  /* 16kB */
  static constexpr uint32_t FLASH_SECTOR_2_START_ADDR = 0x08008000;
  static constexpr uint32_t FLASH_SECTOR_2_END_ADDR   = 0x0800BFFF;
  static constexpr uint32_t FLASH_SECTOR_2_SIZE       = FLASH_SECTOR_2_END_ADDR - FLASH_SECTOR_2_START_ADDR;

  /* 16kB */
  static constexpr uint32_t FLASH_SECTOR_3_START_ADDR = 0x0800C000;
  static constexpr uint32_t FLASH_SECTOR_3_END_ADDR   = 0x0800FFFF;
  static constexpr uint32_t FLASH_SECTOR_3_SIZE       = FLASH_SECTOR_3_END_ADDR - FLASH_SECTOR_3_START_ADDR;

  /* 64kB */
  static constexpr uint32_t FLASH_SECTOR_4_START_ADDR = 0x08010000;
  static constexpr uint32_t FLASH_SECTOR_4_END_ADDR   = 0x0801FFFF;
  static constexpr uint32_t FLASH_SECTOR_4_SIZE       = FLASH_SECTOR_4_END_ADDR - FLASH_SECTOR_4_START_ADDR;

  /* 128kB */
  static constexpr uint32_t FLASH_SECTOR_5_START_ADDR = 0x08020000;
  static constexpr uint32_t FLASH_SECTOR_5_END_ADDR   = 0x0803FFFF;
  static constexpr uint32_t FLASH_SECTOR_5_SIZE       = FLASH_SECTOR_5_END_ADDR - FLASH_SECTOR_5_START_ADDR;

  /* 128kB */
  static constexpr uint32_t FLASH_SECTOR_6_START_ADDR = 0x08040000;
  static constexpr uint32_t FLASH_SECTOR_6_END_ADDR   = 0x0805FFFF;
  static constexpr uint32_t FLASH_SECTOR_6_SIZE       = FLASH_SECTOR_6_END_ADDR - FLASH_SECTOR_6_START_ADDR;

  /* 128kB */
  static constexpr uint32_t FLASH_SECTOR_7_START_ADDR = 0x08060000;
  static constexpr uint32_t FLASH_SECTOR_7_END_ADDR   = 0x0807FFFF;
  static constexpr uint32_t FLASH_SECTOR_7_SIZE       = FLASH_SECTOR_7_END_ADDR - FLASH_SECTOR_7_START_ADDR;

  static constexpr std::array<uint32_t, 8> sectorStartBoundary = { FLASH_SECTOR_0_START_ADDR, FLASH_SECTOR_1_START_ADDR,
                                                                   FLASH_SECTOR_2_START_ADDR, FLASH_SECTOR_3_START_ADDR,
                                                                   FLASH_SECTOR_4_START_ADDR, FLASH_SECTOR_5_START_ADDR,
                                                                   FLASH_SECTOR_6_START_ADDR, FLASH_SECTOR_7_START_ADDR };

#else
#warning Flash memory mapping not defined for the current device. Flash interface will not work correctly.
#endif
}    // namespace Thor::Memory

#endif 

#endif /* !THOR_INTERNAL_FLASH_HPP */
