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

/* Thor Includes */
#include <Thor/thor.hpp>
#include <Thor/definitions.hpp>

/* Chimera Includes */
#include <Chimera/chimera.hpp>
#include <Chimera/interface/watchdog_intf.hpp>

namespace Thor
{
  namespace Watchdog
  {
#if defined( WWDG )
    /**
      *   A high resolution Watchdog peripheral driven by PCLK1 off the AHB bus. This
      *   watchdog is intended to protect against software faults and has more advanced
      *   capabilities than the Independent Watchdog.
      */
    class WindowWatchdog : public Chimera::Watchdog::Interface
    {
    public:
      Chimera::Status_t initialize( const uint32_t timeout_mS, const uint8_t windowPercent = 100 ) final override;

      Chimera::Status_t start() final override;

      Chimera::Status_t stop() final override;

      Chimera::Status_t kick() final override;

      Chimera::Status_t getTimeout( uint32_t &timeout_mS ) final override;

      Chimera::Status_t pauseOnDebugHalt( const bool enable ) final override;

      WindowWatchdog();
      ~WindowWatchdog() = default;

    private:
      WWDG_HandleTypeDef handle;
      uint32_t actualTimeout_mS;

      static const uint8_t counterMax    = 0x7F;
      static const uint8_t counterMin    = 0x40;
      static const uint8_t counterMask   = 0x3F;
      static const uint8_t counterRange  = counterMax - counterMin;
      static const uint8_t numPrescalers = 4;

      /**
        *   Calculates the actual watchdog timeout to the precision of 1mS
        *
        *   @param[in]  pckl1       The clock frequency of PCLK in Hz
        *   @param[in]  prescaler   The watchdog prescaler value as given in the register (0, 1, 2, 3)
        *   @param[in]  counter     The starting value of the countdown timer
        *   @return Number of milliseconds until a timeout occurs
        */
      static uint32_t calculateTimeout_mS( const uint32_t pclk1, const uint8_t prescaler, const uint8_t counter );
    };
#endif /* !WWDG */

#if defined( IWDG )
    /**
      *   A low resolution Watchdog peripheral driven by the LSI clock, which is
      *   independent from the main system clock. This particular watchdog is intended
      *   to protect against issues deriving from a faulty system clock that would not
      *   trip the window watchdog.
      */
    class IndependentWatchdog : public Chimera::Watchdog::Interface
    {
    public:
      Chimera::Status_t initialize( const uint32_t timeout_mS, const uint8_t windowPercent ) final override;

      Chimera::Status_t start() final override;

      Chimera::Status_t stop() final override;

      Chimera::Status_t kick() final override;

      Chimera::Status_t getTimeout( uint32_t &timeout_mS ) final override;

      Chimera::Status_t pauseOnDebugHalt( const bool enable ) final override;

      IndependentWatchdog();
      ~IndependentWatchdog() = default;

    private:
      IWDG_HandleTypeDef handle;
      uint32_t actualTimeout_mS;

      static const uint16_t counterMax   = 0x0FFF;
      static const uint16_t counterMin   = 0x0000;
      static const uint16_t clockFreqHz  = 32000;
      static const uint8_t numPrescalers = 7;

      /**
        *   Calculates the actual watchdog timeout to the precision of 1mS
        *
        *   @param[in]  pckl1       The clock frequency of PCLK in Hz
        *   @param[in]  prescaler   The watchdog prescaler value as given in the register (0, 1, 2, 3)
        *   @param[in]  counter     The starting value of the countdown timer
        *   @return Number of milliseconds until a timeout occurs
        */
      static uint32_t calculateTimeout_mS( const uint32_t pclk1, const uint8_t prescaler, const uint8_t counter );
    };
#endif /* !IWDG */
  }    // namespace Watchdog
}    // namespace Thor
