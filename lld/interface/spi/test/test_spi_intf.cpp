/********************************************************************************
 *  File Name:
 *    test_spi_intf.cpp
 *
 *  Description:
 *    Tests the interface layer for the low leve SPI drivers
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/utility>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/spi/spi_intf.hpp>
#include <Thor/lld/interface/spi/spi_prv_data.hpp>

#if defined( TARGET_LLD_TEST ) && defined( THOR_LLD_SPI )

/* GTest Includes */
#include <gtest/gtest.h>

namespace Thor::LLD::SPI
{
  static Driver fakeDriverList[ NUM_SPI_PERIPHS ];


  /*-------------------------------------------------------------------------------
  FUT: isSupported()
  Range: Normal & Robust
  -------------------------------------------------------------------------------*/
  TEST( Robust_SPI_IsSupported, InvalidInputs )
  {
    EXPECT_EQ( false, isSupported( Chimera::SPI::Channel::NOT_SUPPORTED ) );
    EXPECT_EQ( false, isSupported( Chimera::SPI::Channel::NUM_OPTIONS ) );
  }

  TEST( Normal_SPI_IsSupported, ValidInputs )
  {
    EXPECT_EQ( true, isSupported( Chimera::SPI::Channel::SPI1 ) );
    EXPECT_EQ( false, isSupported( Chimera::SPI::Channel::SPI2 ) );
    EXPECT_EQ( true, isSupported( Chimera::SPI::Channel::SPI3 ) );
    EXPECT_EQ( false, isSupported( Chimera::SPI::Channel::SPI4 ) );
  }


  /*-------------------------------------------------------------------------------
  FUT: getResourceIndex()
  Range: Normal & Robust
  -------------------------------------------------------------------------------*/
  TEST( Robust_SPI_ResourceIndex, InvalidInputs )
  {
    // Type 1: Chimera enum
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getResourceIndex( Chimera::SPI::Channel::NOT_SUPPORTED ) );
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getResourceIndex( Chimera::SPI::Channel::NUM_OPTIONS ) );

    // Type 2: Peripheral Address
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getResourceIndex( reinterpret_cast<std::uintptr_t>( nullptr ) ) );
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getResourceIndex( std::numeric_limits<std::uintptr_t>::max() ) );
  }

  TEST( Normal_SPI_ResourceIndex, ValidInputs )
  {
    // Type 1: Chimera enum
    EXPECT_EQ( SPI1_RESOURCE_INDEX, getResourceIndex( Chimera::SPI::Channel::SPI1 ) );
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getResourceIndex( Chimera::SPI::Channel::SPI2 ) );
    EXPECT_EQ( SPI3_RESOURCE_INDEX, getResourceIndex( Chimera::SPI::Channel::SPI3 ) );
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getResourceIndex( Chimera::SPI::Channel::SPI4 ) );

    // Type 2: Peripheral Address
    EXPECT_EQ( SPI1_RESOURCE_INDEX, getResourceIndex( reinterpret_cast<std::uintptr_t>( SPI1_PERIPH ) ) );
    EXPECT_EQ( SPI3_RESOURCE_INDEX, getResourceIndex( reinterpret_cast<std::uintptr_t>( SPI3_PERIPH ) ) );
  }


  /*-------------------------------------------------------------------------------
  FUT: getChannel()
  Range: Normal & Robust
  -------------------------------------------------------------------------------*/
  TEST( Robust_SPI_GetChannel, InvalidInputs )
  {
    EXPECT_EQ( Chimera::SPI::Channel::NOT_SUPPORTED, getChannel( reinterpret_cast<std::uintptr_t>( nullptr ) ) );
    EXPECT_EQ( Chimera::SPI::Channel::NOT_SUPPORTED, getChannel( std::numeric_limits<std::uintptr_t>::max() ) );
  }

  TEST( Normal_SPI_GetChannel, ValidInputs )
  {
    EXPECT_EQ( Chimera::SPI::Channel::SPI1, getChannel( reinterpret_cast<std::uintptr_t>( SPI1_PERIPH ) ) );
    EXPECT_EQ( Chimera::SPI::Channel::SPI3, getChannel( reinterpret_cast<std::uintptr_t>( SPI3_PERIPH ) ) );
  }


  /*-------------------------------------------------------------------------------
  FUT: attachDriverInstances()
  Range: Normal & Robust
  -------------------------------------------------------------------------------*/
  TEST( Robust_SPI_AttachDriver, InvalidInputs )
  {
    EXPECT_EQ( false, attachDriverInstances( nullptr, 1 ) );
    EXPECT_EQ( false, attachDriverInstances( fakeDriverList, 0 ) );
    EXPECT_EQ( false, attachDriverInstances( fakeDriverList, NUM_SPI_PERIPHS + 1 ) );
    EXPECT_EQ( false, attachDriverInstances( fakeDriverList, ARRAY_COUNT( fakeDriverList ) - 1 ) );
  }

  TEST( Normal_SPI_AttachDriver, ValidInputs )
  {
    EXPECT_EQ( true, attachDriverInstances( fakeDriverList, ARRAY_COUNT( fakeDriverList ) ) );
  }

}    // namespace Thor::LLD::SPI

#endif
