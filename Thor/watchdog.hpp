/********************************************************************************
 *   File Name:
 *       watchdog.hpp
 *
 *   Description:
 *       Thor interface to the STM32 watchdog hardware.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/chimera.hpp>
#include <Chimera/interface/watchdog_intf.hpp>

/* Thor Includes */
#include <Thor/thor.hpp>
#include <Thor/drivers/watchdog.hpp>

namespace Thor::Watchdog
{
  /**
   *   A high resolution Watchdog peripheral driven by PCLK1 off the AHB bus. This
   *   watchdog is intended to protect against software faults and has more advanced
   *   capabilities than the Independent Watchdog.
   */
  class Window : public Chimera::Watchdog::Interface
  {
  public:
    Window();
    ~Window();

    Chimera::Status_t initialize( const uint32_t timeout_mS, const uint8_t windowPercent = 100 ) final override;

    Chimera::Status_t start() final override;

    Chimera::Status_t stop() final override;

    Chimera::Status_t kick() final override;

    Chimera::Status_t pauseOnDebugHalt( const bool enable ) final override;

    size_t getTimeout() final override;

    size_t maxTimeout() final override;

    size_t minTimeout() final override;

  private:
    uint32_t currentPrescaler;
    Thor::Driver::WWDG::Driver_uPtr hwDriver;
  };

  /**
   *   A low resolution Watchdog peripheral driven by the LSI clock, which is
   *   independent from the main system clock. This particular watchdog is intended
   *   to protect against issues deriving from a faulty system clock that would not
   *   trip the window watchdog.
   */
  class Independent : public Chimera::Watchdog::Interface
  {
  public:
    Independent();
    ~Independent();

    Chimera::Status_t initialize( const uint32_t timeout_mS, const uint8_t windowPercent = 0 ) final override;

    Chimera::Status_t start() final override;

    Chimera::Status_t stop() final override;

    Chimera::Status_t kick() final override;

    Chimera::Status_t pauseOnDebugHalt( const bool enable ) final override;

    size_t getTimeout() final override;

    size_t maxTimeout() final override;

    size_t minTimeout() final override;

  private:
    uint32_t currentPrescaler;
    Thor::Driver::IWDG::Driver_uPtr hwDriver;
  };
}    // namespace Thor::Watchdog
