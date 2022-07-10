/********************************************************************************
 *  File Name:
 *    hld_adc_driver.hpp
 *
 *  Description:
 *    Thor ADC high level driver
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HLD_ADC_HPP
#define THOR_HLD_ADC_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/adc>
#include <Chimera/callback>
#include <Chimera/common>
#include <Chimera/thread>
#include <Thor/hld/adc/hld_adc_types.hpp>
#include <cstdint>
#include <cstdlib>

namespace Thor::ADC
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Driver_rPtr       getDriver( const Chimera::ADC::Peripheral periph );
  bool              featureSupported( const Chimera::ADC::Peripheral periph, const Chimera::ADC::Feature feature );


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  /**
   *  ADC Peripheral Driver
   *  Methods here at a minimum implement the interface specified in Chimera.
   *  Inheritance is avoided to minimize cost of virtual function lookup table.
   */
  class Driver : public Chimera::Thread::Lockable<Driver>
  {
  public:
    Driver();
    ~Driver();

    /*-------------------------------------------------------------------------
    Chimera Interface: Hardware
    -------------------------------------------------------------------------*/
    Chimera::Status_t    open( const Chimera::ADC::DriverConfig &init );
    void                 close();
    Chimera::Status_t    setSampleTime( const Chimera::ADC::Channel ch, const size_t cycles );
    Chimera::ADC::Sample sampleChannel( const Chimera::ADC::Channel ch );
    Chimera::Status_t    configSequence( const Chimera::ADC::SequenceInit &cfg );
    void                 startSequence();
    void                 stopSequence();
    bool                 nextSeqSample( const Chimera::ADC::Channel ch, Chimera::ADC::Sample &sample );
    size_t multiSeqSample( const Chimera::ADC::Channel *ch_arr, Chimera::ADC::Sample *sample_arr, const size_t size );
    void   onInterrupt( const Chimera::ADC::Interrupt signal, Chimera::ADC::ISRCallback cb );
    float  toVoltage( const Chimera::ADC::Sample sample );

    /*-------------------------------------------------------------------------
    Thor Interface
    -------------------------------------------------------------------------*/
    /**
     *  User-space handler for processing ISR events
     *  @return void
     */
    void postISRProcessing();

  private:
    friend Chimera::Thread::Lockable<Driver>;

    Chimera::ADC::Peripheral    mPeriph;    /**< Which peripheral is in use */
    Chimera::ADC::DriverConfig  mConfig;    /**< Driver configuration settings */
    Chimera::ADC::SamplingMode  mSeqMode;   /**< Sequence sampling mode */
  };
}    // namespace Thor::ADC

#endif /* THOR_HLD_ADC_HPP */
