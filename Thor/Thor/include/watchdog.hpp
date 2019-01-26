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
#include <Thor/include/thor.hpp>
#include <Thor/include/definitions.hpp>

/* Chimera Includes */
#if defined(USING_CHIMERA)
#include <Chimera/chimera.hpp>
#include <Chimera/interface.hpp>
#endif 

namespace Thor
{
    namespace Peripheral
    {
        namespace Watchdog
        {
            #if defined(WWDG)
            /**
            *   A high resolution Watchdog peripheral driven by PCLK1 off the AHB bus. This 
            *   watchdog is intended to protect against software faults and has more advanced
            *   capabilities than the Independent Watchdog.
            */
            class WindowWatchdog
            {
            public:
                /**
                *   Initializes the low level hardware needed to configure the watchdog peripheral. 
                *   This does not start the timer.
                *
                *   @note   Guarantees a minimum resolution of +/- 500uS around the specified timeout
                *
                *   @note   If the timeout is 45mS and the window is 20, then the dog can only be kicked once
                *           the counter only has 9mS left before expiring. 
                *
                *   @param[in]  timeout_mS      How many milliseconds can elapse before watchdog expires   
                *   @param[in]  windowPercent   Percentage (integer) away from timeout expiring before the dog can be kicked
                *   @return Status::PERIPH_OK if the initialization was a success, Status::PERIPH_ERROR if not
                */
                Thor::Definitions::Status initialize(const uint32_t timeout_mS, const uint8_t windowPercent = 100);

                /**
                *   Starts the watchdog timer. If successful, Interface::kick() must
                *   be called at regular intervals to prevent the watchdog from firing.
                *
                *   @return True if the watchdog was started, false if not
                */
                Thor::Definitions::Status start();
                
                /**
                *   Stops the watchdog timer.
                *
                *   @return True if the watchdog was stopped, false if not
                */
                Thor::Definitions::Status stop();

                /**
                *   Kicks the watchdog timer, starting a new countdown cycle.
                *
                *   @return void
                */
                Thor::Definitions::Status kick();

                /**
                *   Gets the actual timeout value achieved by the hardware
                *   
                *   @return Timeout value in milliseconds
                */
                Thor::Definitions::Status getTimeout(uint32_t &timeout_mS);

                /**
                *   Configures the watchdog to stop on connection to a debugger
                *
                *   @param[in]  enable      If true, allows the watchdog to stop. Otherwise, it continues running
                *   @return Peripheral status
                */
                Thor::Definitions::Status pauseOnDebugHalt(const bool enable);

                WindowWatchdog();
                ~WindowWatchdog() = default;

            private:
                WWDG_HandleTypeDef handle;
                uint32_t actualTimeout_mS;

                static const uint8_t counterMax = 0x7F;
                static const uint8_t counterMin = 0x40;
                static const uint8_t counterMask = 0x3F;
                static const uint8_t counterRange = counterMax - counterMin;
                static const uint8_t numPrescalers = 4;

                /**
                *   Calculates the actual watchdog timeout to the precision of 1mS
                *   
                *   @param[in]  pckl1       The clock frequency of PCLK in Hz
                *   @param[in]  prescaler   The watchdog prescaler value as given in the register (0, 1, 2, 3)
                *   @param[in]  counter     The starting value of the countdown timer
                *   @return Number of milliseconds until a timeout occurs
                */
                static uint32_t calculateTimeout_mS(const uint32_t pclk1, const uint8_t prescaler, const uint8_t counter);
            };
            #endif /* !WWDG */

            #if defined(IWDG)
            /**
            *   A low resolution Watchdog peripheral driven by the LSI clock, which is 
            *   independent from the main system clock. This particular watchdog is intended
            *   to protect against issues deriving from a faulty system clock that would not 
            *   trip the window watchdog.
            */
            class IndependentWatchdog
            {
            public:
                /**
                *   Initializes the low level hardware needed to configure the watchdog peripheral. 
                *   This does not start the timer.
                *
                *   @note   Resolution is highly dependent on the accuracy of the LSI clock to 32KHz.
                *           In general, plan for about +/- 100mS around the specified timeout.
                *
                *   @param[in]  timeout_mS      How many milliseconds can elapse before watchdog expires   
                *   @return Status::PERIPH_OK if the initialization was a success, Status::PERIPH_ERROR if not
                */
                Thor::Definitions::Status initialize(const uint32_t timeout_mS);

                /**
                *   Starts the watchdog timer. If successful, Interface::kick() must
                *   be called at regular intervals to prevent the watchdog from firing.
                *
                *   @return True if the watchdog was started, false if not
                */
                Thor::Definitions::Status start();
                
                /**
                *   Stops the watchdog timer.
                *
                *   @return True if the watchdog was stopped, false if not
                */
                Thor::Definitions::Status stop();

                /**
                *   Kicks the watchdog timer, starting a new countdown cycle.
                *
                *   @return void
                */
                Thor::Definitions::Status kick();

                /**
                *   Gets the actual timeout value achieved by the hardware
                *   
                *   @return Timeout value in milliseconds
                */
                Thor::Definitions::Status getTimeout(uint32_t &timeout_mS);

                /**
                *   Configures the watchdog to stop on connection to a debugger
                *
                *   @param[in]  enable      If true, allows the watchdog to stop. Otherwise, it continues running
                *   @return Peripheral status
                */
                Thor::Definitions::Status pauseOnDebugHalt(const bool enable);

                IndependentWatchdog();
                ~IndependentWatchdog() = default;

            private:
                IWDG_HandleTypeDef handle;
                uint32_t actualTimeout_mS;

                static const uint16_t counterMax = 0x0FFF;
                static const uint16_t counterMin = 0x0000;
                static const uint16_t clockFreqHz = 32000;
                static const uint8_t numPrescalers = 7;

                /**
                *   Calculates the actual watchdog timeout to the precision of 1mS
                *   
                *   @param[in]  pckl1       The clock frequency of PCLK in Hz
                *   @param[in]  prescaler   The watchdog prescaler value as given in the register (0, 1, 2, 3)
                *   @param[in]  counter     The starting value of the countdown timer
                *   @return Number of milliseconds until a timeout occurs
                */
                static uint32_t calculateTimeout_mS(const uint32_t pclk1, const uint8_t prescaler, const uint8_t counter);
            };
            #endif /* !IWDG */

            #if defined(USING_CHIMERA)
            class ChimeraWatchdog : public Chimera::Watchdog::Interface
            {
            public:
                Chimera::Watchdog::Status initialize(const uint32_t timeout_mS) override;

                Chimera::Watchdog::Status start() override;

                Chimera::Watchdog::Status stop() override;

                Chimera::Watchdog::Status kick() override;

                Chimera::Watchdog::Status getTimeout(uint32_t &timeout) override;

                Chimera::Watchdog::Status pauseOnDebugHalt(const bool enable) override;

                ChimeraWatchdog() = default;
                ~ChimeraWatchdog() = default;

            private:
                WatchdogClass watchdog;
            };

            #endif 

        }
    }
}
