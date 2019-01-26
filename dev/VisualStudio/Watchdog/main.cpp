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
using namespace Thor::Definitions::GPIO;

void ledThread(void *argument);
void watchdogThread(void *argument);

int main(void)
{
    ThorInit();

    InitializeInstrumentingProfiler();

    //Thor::Threading::addThread(ledThread, "led", 300, NULL, 2, NULL);
    Thor::Threading::addThread(watchdogThread, "wd", 300, NULL, 2, NULL);
    
    Thor::Threading::startScheduler(true);

    while (1)
    {
        
    }

//    const char *p = "";
//    CommandLineTestRunner::RunAllTests(0, &p);
    return 0;
}

void ledThread(void *argument)
{
    Thor::Peripheral::GPIO::GPIOClass led;
    led.init(Thor::Nucleo::BLUE_LED_PORT, Thor::Nucleo::BLUE_LED_PIN);
    led.mode(PinMode::OUTPUT_PP, PinPull::NOPULL);
    led.write(LogicLevel::LOW);

    uint8_t count = 0;
    while (count < 6)
    {
        led.toggle();
        count++;
        Thor::delayMilliseconds(50);
    }

    Thor::Threading::signalThreadSetupComplete();

    for (;;)
    {
        led.toggle();
        Thor::delayMilliseconds(500);
    }
}

void watchdogThread(void *argument)
{   
    auto dog = Thor::Peripheral::Watchdog::WindowWatchdog();

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
    Thor::Threading::signalThreadSetupComplete();

    dog.initialize(25);
    dog.pauseOnDebugHalt(true);
    dog.start();
    
    for (;;)
    {
        dog.kick();
        greenLed.toggle();
        Thor::delayMilliseconds(20);
    }
}