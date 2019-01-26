/********************************************************************************
*   File Name:
*       main.cpp
*       
*   Description:
*       A simple test of the watchdog timer on one of the nucleo boards
*   
*   2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

#include <Thor/include/thor.hpp>
#include <Thor/include/threads.hpp>
#include <Thor/include/gpio.hpp>
#include <Thor/include/nucleo.hpp>
#include <Thor/include/watchdog.hpp>

#include <CppUTest/CommandLineTestRunner.h>

#include "SysprogsProfiler.h"

using namespace Thor::Peripheral;
using namespace Thor::Threading;
using namespace Thor::Definitions::GPIO;

void heartBeatThread(void *argument);
void windowWatchdogThread(void *argument);
void independentWatchdogThread(void *argument);

int main(void)
{
    ThorInit();

    InitializeInstrumentingProfiler();

    addThread(heartBeatThread, "led", 300, NULL, 2, NULL);
    addThread(windowWatchdogThread, "wwd", 300, NULL, 2, NULL);
    addThread(independentWatchdogThread, "iwd", 300, NULL, 2, NULL);
    startScheduler(true);

    while (1)
    {
        
    }
    return 0;
}

void heartBeatThread(void *argument)
{
    Thor::Peripheral::GPIO::GPIOClass led;
    led.init(Thor::Nucleo::RED_LED_PORT, Thor::Nucleo::RED_LED_PIN);
    led.mode(PinMode::OUTPUT_PP, PinPull::NOPULL);
    led.write(LogicLevel::LOW);

    signalThreadSetupComplete();

    for (;;)
    {
        led.toggle();
        Thor::delayMilliseconds(500);
    }
}

void windowWatchdogThread(void *argument)
{   
    auto dog = Thor::Peripheral::Watchdog::IndependentWatchdog();

    Thor::Peripheral::GPIO::GPIOClass blueLed;
    blueLed.init(Thor::Nucleo::BLUE_LED_PORT, Thor::Nucleo::BLUE_LED_PIN);
    blueLed.mode(PinMode::OUTPUT_PP, PinPull::NOPULL);
    blueLed.write(LogicLevel::LOW);

    Thor::Peripheral::GPIO::GPIOClass greenLed;
    greenLed.init(Thor::Nucleo::GREEN_LED_PORT, Thor::Nucleo::GREEN_LED_PIN);
    greenLed.mode(PinMode::OUTPUT_PP, PinPull::NOPULL);
    greenLed.write(LogicLevel::LOW);

    uint8_t count = 0;
    while (count < 6)
    {
        blueLed.toggle();
        greenLed.toggle();
        count++;
        Thor::delayMilliseconds(50);
    }
    
    signalThreadSetupComplete();

    dog.initialize(35);
    dog.pauseOnDebugHalt(true);
    dog.start();
    
    for (;;)
    {
        dog.kick();
        greenLed.toggle();
        Thor::delayMilliseconds(30);
    }
}

void independentWatchdogThread(void *argument)
{   
    auto dog = Thor::Peripheral::Watchdog::IndependentWatchdog();
    
    signalThreadSetupComplete();

    dog.initialize(1500);
    dog.pauseOnDebugHalt(true);
    dog.start();
    
    for (;;)
    {
        dog.kick();
        Thor::delayMilliseconds(1400);
    }
}