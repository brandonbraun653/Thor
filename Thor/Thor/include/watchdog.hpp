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

/* Thor Includes */


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
            class WatchdogClass
            {
            public:

            private:

            };

            #if defined(USING_CHIMERA)
            class ChimeraWatchdog : public Chimera::Watchdog::Interface
            {
            public:
            
            private:

            };
            #endif 

        }
    }
}



