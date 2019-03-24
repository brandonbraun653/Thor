#pragma once
#ifndef THOR_I2C_HPP
#define THOR_I2C_HPP

// /* C/C++ Includes */
// #include <stdlib.h>
// #include <stdint.h>
// #include <string.h>

// /* Boost Includes */
// #include <boost/shared_ptr.hpp>
// #include <boost/make_shared.hpp>
// #include <boost/move/unique_ptr.hpp>
// #include <boost/circular_buffer.hpp>

// /* FreeRTOS Includes */
// #if defined(USING_FREERTOS)
// #include "FreeRTOS.h"
// #include "semphr.h"
// #endif

// /* Thor Includes */
// #include <Thor/config.hpp>
// #include <Thor/definitions.hpp>
// #include <Thor/defaults.hpp>
// #include <Thor/interrupt.hpp>
// #include <Thor/gpio.hpp>
// #include <Thor/ringbuffer.hpp>
// #include <Thor/exceptions.hpp>

// namespace Thor
// {
//     namespace Peripheral
//     {
//         namespace I2C
//         {
//             using namespace Thor;

//             class I2CClass
//             {
//                 public:
//                     // Variables
//                     uint8_t masters[];
//                     uint8_t slaves[];

//                     // Methods
//                     bool beginRX(uint8_t master);
//                     bool beginTX(uint8_t slave);
//                     bool dataRX();
//                     bool dataTX(uint8_t data);
//                     bool endRX();
//                     bool endRX();

//                 private:
//                     // Variables
//                     bool transmitting = false;
//                     bool receiving = false;
//                     bool txFailed = false;
//                     bool rxFailed = false; // maybe don't need this?
//                     bool txComplete = true;
//                     bool rxComplete = true;
//                     uint8_t sdaPin = 0; // Default
//                     uint8_t sclPin = 1; // Default

//                     // Methods
//                     bool pauseRX(uint8_t master);

//                 public:
//                     // Setter/Getter Methods
//                     void setTxComplete(bool value)
//                     {
//                         txComplete = value;
//                     }

//                     void setRxComplete(bool value)
//                     {
//                         rxComplete = value;
//                     }

//                     bool getTxComplete()
//                     {
//                         return txComplete;
//                     }

//                     bool getRxComplete()
//                     {
//                         return rxComplete;
//                     }
//             }
//         }
//     }
// }

#endif /* I2C_H_*/
