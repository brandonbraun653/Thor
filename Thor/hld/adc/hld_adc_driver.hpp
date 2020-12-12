/********************************************************************************
 *  File Name:
 *    hld_adc_driver.hpp
 *
 *  Description:
 *    Thor ADC high level driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HLD_ADC_HPP
#define THOR_HLD_ADC_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/adc>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/hld/adc/hld_adc_types.hpp>

namespace Thor::ADC
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Driver_rPtr getDriver( const Chimera::ADC::Converter periph );
  Driver_sPtr getDriverShared( const Chimera::ADC::Converter periph );
  bool featureSupported( const Chimera::ADC::Converter periph, const Chimera::ADC::Feature feature );


  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  /**
   *  ADC Peripheral Driver
   *  Methods here at a minimum implement the interface specified in Chimera.
   *  Inheritance is avoided to minimize cost of virtual function lookup table.
   */
  class Driver : public Chimera::Threading::Lockable
  {
  public:
    Driver();
    ~Driver();

    /*-------------------------------------------------
    Interface: Hardware
    -------------------------------------------------*/
    Chimera::Status_t open( const Chimera::ADC::DriverConfig &init );
    void close();
    void setPowerState( const bool state );
    Chimera::ADC::Sample_t sampleChannel( const Chimera::ADC::Channel ch );
    Chimera::ADC::Sample_t sampleSensor( const Chimera::ADC::Sensor sensor );
    Chimera::Status_t groupConfig( const Chimera::ADC::GroupInit &cfg );
    Chimera::Status_t groupStartSample( const Chimera::ADC::SampleGroup grp );
    Chimera::Status_t groupGetSample( const Chimera::ADC::SampleGroup grp, Chimera::ADC::Sample_t *const out,
                                      const size_t len );
    Chimera::Status_t groupSetDMABuffer( const Chimera::ADC::SampleGroup grp, Chimera::ADC::Sample_t *const out,
                                         const size_t len );
    Chimera::Status_t setSampleTime( const Chimera::ADC::Channel ch, const size_t cycles );
    void setWatchdogThreshold( const Chimera::ADC::Watchdog wd, const Chimera::ADC::Sample_t low,
                               const Chimera::ADC::Sample_t high );
    void onInterrupt( const Chimera::ADC::Interrupt bmSignal, Chimera::ADC::ISRCallback cb );

  private:
    Chimera::ADC::Converter mPeriph;
    Chimera::ADC::DriverConfig mConfig;
  };
}    // namespace Thor::ADC

#endif /* THOR_HLD_ADC_HPP */
