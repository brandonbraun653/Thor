/******************************************************************************
 *  File Name:
 *    adc_intf.hpp
 *
 *  Description:
 *    STM32 LLD ADC Interface Spec
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_ADC_DRIVER_INTERFACE_HPP
#define THOR_LLD_ADC_DRIVER_INTERFACE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/adc>
#include <Chimera/common>
#include <Chimera/dma>
#include <Chimera/thread>
#include <Thor/cfg>
#include <Thor/lld/common/interrupts/adc_interrupt_vectors.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/adc/adc_detail.hpp>
#include <Thor/lld/interface/adc/adc_types.hpp>
#include <Thor/lld/interface/hw_base_peripheral.hpp>

namespace Thor::LLD::ADC
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   *  Initializes the low level driver
   */
  Chimera::Status_t initialize();

  /**
   *  Gets a shared pointer to the ADC driver for a particular periph
   *
   *  @param[in] periph         The ADC periph to grab (1 indexed)
   *  @return IDriver_rPtr      Instance of the ADC driver for the requested periph
   */
  Driver_rPtr getDriver( const Chimera::ADC::Peripheral periph );

  /**
   *  Checks if the hardware ADC feature is supported
   *
   *  @param[in]  periph        Which peripheral to check
   *  @param[in]  feature       The feature to check
   *  @return bool
   */
  bool featureSupported( const Chimera::ADC::Peripheral periph, const Chimera::ADC::Feature feature );


  /*---------------------------------------------------------------------------
  Public Functions (Implemented at the interface layer)
  ---------------------------------------------------------------------------*/
  /**
   *  Checks if the given hardware periph is supported on this device.
   *
   *  @param[in]  periph        The periph number to be checked
   *  @return bool
   */
  bool isSupported( const Chimera::ADC::Peripheral periph );

  /**
   *  Gets the resource index associated with a particular periph. If not
   *  supported, will return INVALID_RESOURCE_INDEX
   *
   *  @param[in]  periph        The periph number to be checked
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const Chimera::ADC::Peripheral periph );

  /**
   *  Looks up a resource index based on a raw peripheral instance
   *
   *  @param[in]  address       The peripheral address
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const std::uintptr_t address );

  /**
   *  Gets the ADC periph associated with a peripheral address
   *
   *  @param[in]  address       Memory address the peripheral is mapped to
   *  @return Chimera::ADC::Peripheral
   */
  Chimera::ADC::Peripheral getChannel( const std::uintptr_t address );

  /**
   *  Initializes the ADC drivers by attaching the appropriate peripheral
   *
   *  @param[in]  driverList    List of driver objects to be initialized
   *  @param[in]  numDrivers    How many drivers are in driverList
   *  @return bool
   */
  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers );


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
#if defined( THOR_ADC )
  class Driver : public Chimera::Thread::Lockable<Driver>
  {
  public:
    Driver();
    ~Driver();

    /*-------------------------------------------------------------------------
    Implemented at LLD Interface Layer
    -------------------------------------------------------------------------*/
    Chimera::Status_t reset();
    void              clockReset();
    void              clockEnable();
    void              clockDisable();
    void              disableInterrupts();
    void              enableInterrupts();
    void              onInterrupt( const Chimera::ADC::Interrupt signal, Chimera::ADC::ISRCallback cb );

    /*-------------------------------------------------------------------------
    Driver Specific Layer
    -------------------------------------------------------------------------*/
    /**
     *  Attaches a peripheral instance to the interaction model
     *
     *  @param[in]  peripheral    Memory mapped struct of the desired ADC peripheral
     *  @return void
     */
    Chimera::Status_t attach( RegisterMap *const peripheral );

    /**
     *  Initializes the peripheral with the desired settings
     *
     *  @param[in]  cfg           Configuration info
     *  @return Chimera::Status_t
     */
    Chimera::Status_t initialize( const Chimera::ADC::DriverConfig &cfg );

    /**
     *  Sets the sample time for a given channel. The value is applied
     *  just before the next conversion to prevent accidental overwrites.
     *
     *  @param[in]  ch            Which channel to modify
     *  @param[in]  time          ADC clock cycles to sample for
     *  @return Chimera::Status_t
     */
    Chimera::Status_t setSampleTime( const Chimera::ADC::Channel ch, const SampleTime time );

    /**
     *  Sets up the ADC to run a sequence of conversions
     *
     * @param[in] sequence      Sample sequence config
     * @return Chimera::Status_t
     */
    Chimera::Status_t setupSequence( const Chimera::ADC::SequenceInit &sequence );

    /**
     *  Immediately reads a single ADC channel and returns the result
     *
     *  @param[in]  channel     The channel to read
     *  @return Chimera::ADC::Sample
     */
    Chimera::ADC::Sample sampleChannel( const Chimera::ADC::Channel channel );

    /**
     * @brief Gets the currently configured ADC resolution
     * @return float
     */
    float resolution() const;

    /**
     * @brief Gets the analog reference voltage for conversions
     * @return float
     */
    float analogReference() const;

    /**
     *  Converts a raw sample to the equivalent voltage
     *
     *  @param[in]  sample        The raw sample value to convert
     *  @return float
     */
    float toVoltage( const Chimera::ADC::Sample sample );

    /**
     * @brief Configures the Analog Watchdog to monitor a channel
     *
     * @param cfg   Configuration to use
     * @return Chimera::Status_t
     */
    Chimera::Status_t monitorChannel( const Chimera::ADC::WatchdogConfig &cfg );

    /**
     * @brief (Re)starts the sequence conversions
     */
    void startSequence();

    /**
     * @brief Stops any in-progress sequence conversions
     */
    void stopSequence();

    /**
     * @brief Copies latest ADC data into the HLD queues on-demand
     *
     * Assumes operation inside of an interrupt free context. The call should be
     * wrapped with stopSequence()/startSequence().
     */
    void syncSequence();

    /**
     * @brief ISRHandler shared between all enabled interrupts
     */
    void ISRHandler();

  protected:
    void dma_isr_transfer_complete_callback( const Chimera::DMA::TransferStats &stats );

  private:
    friend Chimera::Thread::Lockable<Driver>;

    /*-------------------------------------------------------------------------
    Memory mapped structures to peripheral registers
    -------------------------------------------------------------------------*/
    RegisterMap       *mPeriph; /**< Peripheral memory map */
    CommonRegisterMap *mCommon; /**< Shared settings memory map */

    /*-------------------------------------------------------------------------
    Driver attributes
    -------------------------------------------------------------------------*/
    size_t                      mResourceIndex; /**< Hardware resource index for this peripheral */
    Chimera::ADC::DriverConfig  mCfg;           /**< Basic driver configuration data */
    Chimera::ADC::SequenceInit  mSeqCfg;        /**< Sequence configuration */
    Chimera::ADC::CallbackArray mCallbacks;     /**< Associated event callbacks */
    Chimera::DMA::RequestId     mDMAPipeID;     /**< Unique ID for dedicated DMA pipe */

    /*-------------------------------------------------------------------------
    Sequence Data
    -------------------------------------------------------------------------*/
    DMASample<NUM_ADC_CHANNELS_PER_PERIPH> mDMASampleBuffer; /**< Buffer for the HW to work on */
  };

#endif /* THOR_LLD_ADC */

}    // namespace Thor::LLD::ADC

#endif /* THOR_LLD_ADC_DRIVER_INTERFACE_HPP */
