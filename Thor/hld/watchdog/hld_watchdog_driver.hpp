/********************************************************************************
 *  File Name:
 *    watchdog.hpp
 *
 *  Description:
 *    Thor interface to the STM32 watchdog hardware.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>
#include <Chimera/watchdog>

/* Thor Includes */
#include <Thor/hld/watchdog/hld_watchdog_types.hpp>

namespace Thor::Watchdog
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t reset();
  WindowDriver_rPtr getDriver( const Chimera::Watchdog::WChannel channel );
  IndependentDriver_rPtr getDriver( const Chimera::Watchdog::IChannel channel );

  /**
   *  Initialize the WWDG driver
   *
   *  @return Chimera::Status_t
   */
  Chimera::Status_t initializeWWDG();

  /**
   *  Initialize the WWDG driver
   *
   *  @return Chimera::Status_t
   */
  Chimera::Status_t initializeIWDG();


  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  /**
   *   A high resolution Watchdog peripheral driven by PCLK1 off the AHB bus. This
   *   watchdog is intended to protect against software faults and has more advanced
   *   capabilities than the Independent Watchdog.
   */
  class WindowDriver : public Chimera::Threading::LockableCRTP<WindowDriver>
  {
  public:
    WindowDriver();
    ~WindowDriver();

    Chimera::Status_t initialize( const Chimera::Watchdog::WChannel ch, const uint32_t timeout_mS, const uint8_t windowPercent );
    Chimera::Status_t start();
    Chimera::Status_t stop();
    Chimera::Status_t kick();
    Chimera::Status_t pauseOnDebugHalt( const bool enable );
    size_t getTimeout();
    size_t maxTimeout();
    size_t minTimeout();

  private:
    friend Chimera::Threading::LockableCRTP<WindowDriver>;
    Chimera::Threading::RecursiveTimedMutex mClsMutex;

    Chimera::Watchdog::WChannel mChannel;
  };

  /**
   *   A low resolution Watchdog peripheral driven by the LSI clock, which is
   *   independent from the main system clock. This particular watchdog is intended
   *   to protect against issues deriving from a faulty system clock that would not
   *   trip the window watchdog.
   */
  class IndependentDriver : public Chimera::Threading::LockableCRTP<IndependentDriver>
  {
  public:
    IndependentDriver();
    ~IndependentDriver();

    Chimera::Status_t initialize( Chimera::Watchdog::IChannel ch, const uint32_t timeout_mS );
    Chimera::Status_t start();
    Chimera::Status_t stop();
    Chimera::Status_t kick();
    Chimera::Status_t pauseOnDebugHalt( const bool enable );
    size_t getTimeout();
    size_t maxTimeout();
    size_t minTimeout();

  private:
    friend Chimera::Threading::LockableCRTP<IndependentDriver>;
    Chimera::Threading::RecursiveTimedMutex mClsMutex;

    Chimera::Watchdog::IChannel mChannel;
  };

}    // namespace Thor::Watchdog
