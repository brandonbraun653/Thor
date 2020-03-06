/********************************************************************************
 *  File Name:
 *    rcc_mock.hpp
 *
 *  Description:
 *    Mock file for RCC LLD
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_LLD_RCC_MOCK_HPP
#define THOR_LLD_RCC_MOCK_HPP

/* Google Includes */
#include "gmock/gmock.h"

/* Thor Includes */
#include <Thor/lld/interface/rcc/rcc_intf.hpp>
#include <Thor/lld/interface/rcc/rcc_types.hpp>


namespace Thor::LLD::RCC
{
  class SystemClockMock : public IClockTree
  {
  public:
    MOCK_METHOD0( configureProjectClocks, Chimera::Status_t() );
    MOCK_METHOD2( setPeriphClock, Chimera::Status_t( const Chimera::Peripheral::Type, const size_t ) );
    MOCK_METHOD1( setCoreClock, Chimera::Status_t( const size_t ) );
    MOCK_METHOD1( setCoreClockSource, Chimera::Status_t( const Thor::Clock::Source ) );
    MOCK_METHOD2( getClockFrequency, Chimera::Status_t( const ClockType_t, size_t *const ) );
    MOCK_METHOD3( getPeriphClock, Chimera::Status_t( const Chimera::Peripheral::Type, const std::uintptr_t, size_t *const ) );
  };

  class PeripheralControllerMock : public IPeripheralController
  {
  public:
    MOCK_METHOD2( reset, Chimera::Status_t( const Chimera::Peripheral::Type, const size_t ) );
    MOCK_METHOD2( enableClock, Chimera::Status_t( const Chimera::Peripheral::Type, const size_t ) );
    MOCK_METHOD2( disableClock, Chimera::Status_t( const Chimera::Peripheral::Type, const size_t ) );
    MOCK_METHOD2( enableClockLowPower, Chimera::Status_t( const Chimera::Peripheral::Type, const size_t ) );
    MOCK_METHOD2( disableClockLowPower, Chimera::Status_t( const Chimera::Peripheral::Type, const size_t ) );
  };
}


#endif  /* !THOR_LLD_RCC_MOCK_HPP */  
