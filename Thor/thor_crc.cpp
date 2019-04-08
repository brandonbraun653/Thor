/********************************************************************************
 *   File Name:
 *    thor_crc.cpp
 *
 *   Description:
 *    Implements the CRC functionality for Thor
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Thor Includes */
#include <Thor/crc.hpp>


namespace Thor
{
  namespace HWCRC
  {
    HW::HW()
    {
      
    }

    HW::~HW()
    {
      
    }

    Chimera::Status_t HW::init( const uint32_t polynomial, const uint8_t crcWidth )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    uint32_t HW::accumulate( const uint32_t *const buffer, const uint32_t length )
    {
      return 0u;
    }

    uint32_t HW::calculate( const uint32_t *const buffer, const uint32_t length )
    {
      return 0u;
    }
  }
}