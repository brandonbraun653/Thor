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
#include <Thor/headers.hpp>
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
      Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

      #if defined( STM32F446xx ) || defined( STM32F767xx)
      crcHandle.Instance = CRC;
      #else
      #error Unknown CRC instance!
      #endif 
      
      if ( HAL_CRC_Init( &crcHandle ) != HAL_OK )
      {
        error = Chimera::CommonStatusCodes::FAIL;
      }

      return error;
    }

    uint32_t HW::accumulate( const uint32_t *const buffer, const uint32_t length )
    {
      /* Shame on me for the const_cast, but there is not much I can do */
      return HAL_CRC_Accumulate( &crcHandle, const_cast<uint32_t *>(buffer), length );
    }

    uint32_t HW::calculate( const uint32_t *const buffer, const uint32_t length )
    {
      /* Shame on me for the const_cast, but there is not much I can do */
      return HAL_CRC_Calculate( &crcHandle, const_cast<uint32_t *>( buffer ), length );
    }

    uint32_t HW::getPolynomial()
    {
      uint32_t val = 0u;
      
#if defined( STM32F446xx )
      val = 0x4C11DB7; /* Reference manual RM0390: pg.87 */
#endif 
      
      return val;
    }
  }
}