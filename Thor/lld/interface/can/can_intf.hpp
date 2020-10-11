/********************************************************************************
 *  File Name:
 *    can_intf.hpp
 *
 *  Description:
 *    STM32 LLD CAN Interface Spec
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_LLD_CAN_DRIVER_INTERFACE_HPP
#define THOR_LLD_CAN_DRIVER_INTERFACE_HPP

/* STL Includes */
#include <limits>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/can>
#include <Chimera/gpio>

/* Thor Includes */
#include <Thor/lld/common/interrupts/can_interrupt_vectors.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/can/can_types.hpp>

namespace Thor::LLD::CAN
{
  /*-------------------------------------------------------------------------------
  Public Functions (Implemented by the project)
  -------------------------------------------------------------------------------*/
  /**
   *  Initializes the low level driver
   *
   *  @return Chimera::Status_t
   */
  Chimera::Status_t initialize();

  /**
   *  Gets a raw pointer to the CAN driver for a particular channel
   *
   *  @param[in]  channel     The CAN channel to get
   *  @return IDriver_sPtr    Instance of the CAN driver for the requested channel
   */
  Driver_rPtr getDriver( const Chimera::CAN::Channel channel );

  /*-------------------------------------------------------------------------------
  Public Functions (Implemented at the interface layer)
  -------------------------------------------------------------------------------*/
  /**
   *  Checks if the given hardware channel is supported on this device.
   *
   *  @param[in]  port        The CAN port to grab
   *  @param[in]  pin         Which pin on the given port
   *  @return bool
   */
  bool isSupported( const Chimera::CAN::Channel channel );

  /**
   *  Looks up an index value that can be used to access distributed resources
   *  associated with a peripheral instance. If the address is invalid, this will
   *  return INVALID_RESOURCE_INDEX.
   *
   *  @param[in]  address       Memory address the peripheral is mapped to
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const std::uintptr_t address );

  /**
   *  Looks up an index value that can be used to access distributed resources
   *  associated with a peripheral instance. If the port is invalid, this will
   *  return INVALID_RESOURCE_INDEX.
   *
   *  @param[in]  port          Which port to get the resource index for
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const Chimera::CAN::Channel channel );

  /**
   *  Gets the CAN port associated with a peripheral address
   *
   *  @param[in]  address       Memory address the peripheral is mapped to
   *  @return Chimera::CAN::Channel
   */
  Chimera::CAN::Channel getChannel( const std::uintptr_t address );

  /**
   *  Initializes the CAN drivers by attaching the appropriate peripheral
   *
   *  @param[in]  driverList  List of driver objects to be initialized
   *  @param[in]  numDrivers  How many drivers are in driverList
   *  @return bool
   */
  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers );


  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  /*-------------------------------------------------
  Virtual class that defines the expected interface.
  Useful for mocking purposes.
  -------------------------------------------------*/
  class IDriver
  {
  public:
    virtual ~IDriver() = default;

    /*-------------------------------------------------------------------------------
    Configuration
    -------------------------------------------------------------------------------*/
    /**
     *  Attaches a peripheral instance to the interaction model
     *
     *  @param[in]  peripheral    Memory mapped struct of the desired CAN peripheral
     *  @return void
     */
    virtual void attach( RegisterMap *const peripheral ) = 0;

    /**
     *  Configure the CAN bus driver with the appropriate settings. These are
     *  more generic settings that apply across all CAN implementations.
     *
     *  @param[in]  cfg           The configuration info
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t configure( const Chimera::CAN::DriverConfig &cfg ) = 0;

    virtual Chimera::Status_t applyFilter( const Chimera::CAN::Filter &filter ) = 0;

    /**
     *  Turns on interrupts for the given signal type, if supported.
     *
     *  @param[in]  signal        The ISR event to enable
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t enableISRSignal( const Chimera::CAN::InterruptType signal ) = 0;

    /**
     *  Turns off interrupts for the given signal type, if supported.
     *
     *  @param[in]  signal        The ISR event to enable
     *  @return void
     */
    virtual void disableISRSignal( const Chimera::CAN::InterruptType signal ) = 0;

    virtual void freezeOnDebug( const bool doFreeze ) = 0;

    virtual void invokeMasterResetRequest() = 0;

    virtual void useAutoRetransmit( const bool doAutoRTX ) = 0;

    virtual void lockRXOnOverrun( const bool doLock ) = 0;

    virtual void useTXPriorityScheme( const TXPriority scheme ) = 0;

    virtual void enterSleepMode() = 0;

    virtual void exitSleepMode() = 0;

    virtual void enterDebugMode( const Chimera::CAN::DebugMode mode ) = 0;

    virtual void exitDebugMode() = 0;


    /*-------------------------------------------------------------------------------
    Transmit & Receive Operations
    -------------------------------------------------------------------------------*/
    /**
     *  Looks through the hardware TX FIFOs to see if a mailbox is available to
     *  transmit on.
     *
     *  @param[out] which         If a mailbox is available, will return its ID
     *  @return bool
     */
    virtual bool txMailboxAvailable( Mailbox &which ) = 0;

    /**
     *  Looks through the hardware RX FIFOs to see if any mailbox contains a message.
     *
     *  @param[out] which         If a mailbox is available, will return its ID
     *  @return bool
     */
    virtual bool rxMailboxAvailable( Mailbox &which ) = 0;

    /**
     *  Places the given frame into the TX FIFOs for transmission.
     *
     *  @param[in]  which       Mailbox to place the frame into
     *  @param[in]  frame       The frame to be transmitted
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t send( const Mailbox which, const Chimera::CAN::BasicFrame &frame ) = 0;

    /**
     *  Attempts to read a frame off the RX FIFO
     *
     *  @param[in]  which       Mailbox to read out of
     *  @param[out] frame       The frame to place the received message into
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t receive( const Mailbox which, Chimera::CAN::BasicFrame &frame ) = 0;

    /*-------------------------------------------------------------------------------
    Asynchronous Operation
    -------------------------------------------------------------------------------*/
    /**
     *  Gets the driver's semaphore associated with an ISR event. The semaphore
     *  will be given to upon event occurance, unblocking a task that is pending.
     *
     *  @warning Only a single thread should consume the signal. The purpose of
     *  this function is to allow a high priority thread in the HLD to process
     *  the event outside of the limited ISR context.
     *
     *  @param[in]  signal        Which ISR signal to get the semaphore for
     *  @return Chimera::Threading::BinarySemaphore *
     */
    virtual Chimera::Threading::BinarySemaphore* getISRSignal( Chimera::CAN::InterruptType signal ) = 0;

    /**
     *  Gets any information that was posted in relation to the last
     *  ISR event.
     *
     *  @note This is limited to read-only because of the single producer
     *  many consumer data model. The ISR produces, you consume. Copy out
     *  the data inside of a critical section to guarantee the ISR won't
     *  modify the structure while reading.
     *
     *  @param[in]  isr           The ISR event type to get the context for
     *
     *  @return const ISREventContext *const
     */
    virtual const ISREventContext *const getISRContext( const Chimera::CAN::InterruptType isr ) = 0;
  };


  /*-------------------------------------------------
  Concrete driver declaration. Implements the interface
  of the virtual class, but doesn't inherit due to the
  memory penalties. Definition is done project side.
  -------------------------------------------------*/
  class Driver
  {
  public:
    Driver();
    ~Driver();

    /*-------------------------------------------------------------------------------
    Configuration
    -------------------------------------------------------------------------------*/
    void attach( RegisterMap *const peripheral );
    Chimera::Status_t configure( const Chimera::CAN::DriverConfig &cfg );
    Chimera::Status_t applyFilter( const Chimera::CAN::Filter &filter );
    Chimera::Status_t enableISRSignal( const Chimera::CAN::InterruptType signal );
    void disableISRSignal( const Chimera::CAN::InterruptType signal );
    void enterDebugMode( const Chimera::CAN::DebugMode mode );
    void exitDebugMode();

    /*-------------------------------------------------------------------------------
    Transmit & Receive Operations
    -------------------------------------------------------------------------------*/
    bool txMailboxAvailable(  Mailbox &which );
    bool rxMailboxAvailable(  Mailbox &which );
    Chimera::Status_t send( const Mailbox which, const Chimera::CAN::BasicFrame &frame );
    Chimera::Status_t receive( const Mailbox which, Chimera::CAN::BasicFrame &frame );

    /*-------------------------------------------------------------------------------
    Asynchronous Operation
    -------------------------------------------------------------------------------*/
    Chimera::Threading::BinarySemaphore* getISRSignal( Chimera::CAN::InterruptType signal );
    const ISREventContext *const getISRContext( const Chimera::CAN::InterruptType isr );

    /*-------------------------------------------------------------------------------
    ISR Protection Mechanisms
    -------------------------------------------------------------------------------*/
    void enterCriticalSection();
    void exitCriticalSection();

  protected:
    /*-------------------------------------------------------------------------------
    ISR Handlers
    -------------------------------------------------------------------------------*/
    void CAN1_TX_IRQHandler();
    void CAN1_FIFO0_IRQHandler();
    void CAN1_FIFO1_IRQHandler();
    void CAN1_ERR_STS_CHG_IRQHandler();

  private:
    friend void( ::CAN1_FIFO0_IRQHandler )();
    friend void( ::CAN1_TX_IRQHandler )();
    friend void( ::CAN1_ERR_STS_CHG_IRQHandler )();
    friend void( ::CAN1_FIFO1_IRQHandler )();

    /*-------------------------------------------------
    Peripheral descriptive information
    -------------------------------------------------*/
    RegisterMap *mPeriph;
    size_t mResourceIndex;

    /*-------------------------------------------------
    ISR signaling and context buffers
    -------------------------------------------------*/
    ISREventContext mISREventContext[ NUM_CAN_IRQ_HANDLERS ];
    Chimera::Threading::BinarySemaphore mISREventSignal[ NUM_CAN_IRQ_HANDLERS ];

    /*-------------------------------------------------
    GPIO handles for configuring TX/RX pins appropriately
    -------------------------------------------------*/
    Chimera::GPIO::Driver_sPtr mTXPin;
    Chimera::GPIO::Driver_sPtr mRXPin;
  };
}    // namespace Thor::LLD::CAN

#endif /* !THOR_LLD_CAN_DRIVER_INTERFACE_HPP */
