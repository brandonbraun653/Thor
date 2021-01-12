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

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/rcc/rcc_intf.hpp>
#include <Thor/lld/interface/rcc/rcc_types.hpp>

#if defined( THOR_LLD_RCC_MOCK )

/* Google Includes */
#include "gmock/gmock.h"

namespace Thor::LLD::RCC::Mock
{
  /*-------------------------------------------------------------------------------
  Mock Interfaces
  -------------------------------------------------------------------------------*/
  /**
   *  Encapsulates the C-style interface to RCC so that it can be
   *  mocked appropriately. Useless outside of testing purposes.
   */
  class IModule
  {
  public:
    virtual ~IModule() = default;

    virtual void initialize()                            = 0;
    virtual void clearResetReason()                      = 0;
    virtual Chimera::System::ResetEvent getResetReason() = 0;
    virtual ICoreClock *getCoreClock()                   = 0;
    virtual IPeripheralClock *getPeripheralClock()       = 0;
  };


  /*-------------------------------------------------------------------------------
  Mock Classes
  -------------------------------------------------------------------------------*/
  class ModuleMock : public IModule
  {
  public:
    MOCK_METHOD( void, initialize, (), ( override ) );
    MOCK_METHOD( void, clearResetReason, (), ( override ) );
    MOCK_METHOD( Chimera::System::ResetEvent, getResetReason, (), ( override ) );
    MOCK_METHOD( ICoreClock *, getCoreClock, (), ( override ) );
    MOCK_METHOD( IPeripheralClock *, getPeripheralClock, (), ( override ) );
  };


  class CoreClockMock : public ICoreClock
  {
  public:
    MOCK_METHOD( void, enableClock, ( const Chimera::Clock::Bus ), ( override ) );
    MOCK_METHOD( void, disableClock, ( const Chimera::Clock::Bus ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, configureProjectClocks, (), ( override ) );
    MOCK_METHOD( Chimera::Status_t, setCoreClockSource, ( const Chimera::Clock::Bus ), ( override ) );
    MOCK_METHOD( Chimera::Clock::Bus, getCoreClockSource, (), ( override ) );
    MOCK_METHOD( Chimera::Status_t, setClockFrequency, ( const Chimera::Clock::Bus, const size_t, const bool ), ( override ) );
    MOCK_METHOD( size_t, getClockFrequency, ( const Chimera::Clock::Bus ), ( override ) );
    MOCK_METHOD( size_t, getPeriphClock, ( const Chimera::Peripheral::Type, const std::uintptr_t ), ( override ) );
  };


  class PeripheralClockMock : public IPeripheralClock
  {
  public:
    MOCK_METHOD( Chimera::Status_t, reset, ( const Chimera::Peripheral::Type, const size_t ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, enableClock, ( const Chimera::Peripheral::Type, const size_t ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, disableClock, ( const Chimera::Peripheral::Type, const size_t ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, enableClockLowPower, ( const Chimera::Peripheral::Type, const size_t ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, disableClockLowPower, ( const Chimera::Peripheral::Type, const size_t ), ( override ) );
  };


  /*-------------------------------------------------------------------------------
  Mock Public Functions
  -------------------------------------------------------------------------------*/
  /**
   *  Gets the mock object for this module
   *
   *  @return ModuleMock&
   */
  ModuleMock &getMockObject();

}    // namespace Thor::LLD::RCC::Mock

#endif /* THOR_LLD_RCC_MOCK */

#endif /* !THOR_LLD_RCC_MOCK_HPP */
