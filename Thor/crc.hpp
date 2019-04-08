/********************************************************************************
 *   File Name:
 *    crc.hpp
 *
 *   Description:
 *    CRC interface for Thor
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_CRC_HPP
#define THOR_CRC_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/interface.hpp>

/* Thor Includes */
#include <Thor/headers.hpp>

namespace Thor
{
  namespace HWCRC
  {
    class HW : public Chimera::HWCRC::HWInterface
    {
    public:
      HW();
      ~HW();

      Chimera::Status_t init( const uint32_t polynomial, const uint8_t crcWidth ) final override;

      uint32_t accumulate( const uint32_t *const buffer, const uint32_t length ) final override;

      uint32_t calculate( const uint32_t *const buffer, const uint32_t length ) final override;
    
    private:
      CRC_HandleTypeDef crc;
    };
  }    // namespace CRC
}    // namespace Thor


#endif /* !THOR_CRC_HPP */