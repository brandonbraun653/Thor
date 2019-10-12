/********************************************************************************
 *   File Name:
 *    dma_model.hpp
 *
 *   Description:
 *    STM32 Driver DMA Model
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_MODEL_DMA_HPP
#define THOR_DRIVER_MODEL_DMA_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/interface/event_intf.hpp>
#include <Chimera/threading.hpp>
#include <Chimera/types/common_types.hpp>
#include <Chimera/types/event_types.hpp>
#include <Chimera/types/dma_types.hpp>

/* Thor Includes */
#include <Thor/drivers/common/types/dma_types.hpp>

namespace Thor::Driver::DMA
{
  class PeripheralModel
  {
  public:
    virtual ~PeripheralModel() = default;

    /**
     *  Attaches an instance of a DMA peripheral for the class to control
     *  
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t attach( RegisterMap *const peripheral ) = 0;

    /**
     *  Enables the DMA peripheral clock
     *  
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t clockEnable() = 0;

    /**
     *  Disables the DMA peripheral clock
     *  
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t clockDisable() = 0;

    /**
     *  Completely resets the entire driver, including all instance resources.
     *  
     *  @note init() must be called again before the driver can be reused
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t reset() = 0;

    /**
     *  Performs low level driver initialization functionality
     *  
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t init() = 0;

    /**
     *  Reconfigures a stream for a new transfer
     *
     *  @param[in]  config    The stream's transfer configuration settings
     *  @return Chimera::Status_t
     *
     *  |  Return Value |               Explanation               |
     *  |:-------------:|:---------------------------------------:|
     *  |            OK | The stream was configured               |
     *  |        LOCKED | The stream is busy                      |
     *  | NOT_SUPPORTED | A configuration option wasn't supported |
     */
    virtual Chimera::Status_t configure( StreamX *const stream, StreamConfig *const config, TCB *const controlBlock ) = 0;

    virtual Chimera::Status_t start( StreamX *const stream ) = 0;

    virtual Chimera::Status_t abort( StreamX *const stream ) = 0;
  };

  class StreamModel : public Chimera::Event::ListenerInterface
  {
  public:
    virtual ~StreamModel() = default;

    virtual Chimera::Status_t attach( StreamX *const peripheral, RegisterMap *const parent ) = 0;

    /**
     *  Attaches a semaphore that will be given to in the ISR handler
     *  when any event completes. This could be transfer complete, transfer
     *  half complete, etc.
     *
     *  @param[in]  wakeup    Signal to be given to upon ISR events
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t attachISRWakeup( SemaphoreHandle_t wakeup ) = 0;
    
    /**
     *  Reconfigures a stream for a new transfer
     *
     *  @param[in]  config    The stream's transfer configuration settings
     *  @return Chimera::Status_t
     *
     *  |  Return Value |               Explanation               |
     *  |:-------------:|:---------------------------------------:|
     *  |            OK | The stream was configured               |
     *  |        LOCKED | The stream is busy                      |
     *  | NOT_SUPPORTED | A configuration option wasn't supported |
     */
    virtual Chimera::Status_t configure( StreamConfig *const config, TCB *const controlBlock ) = 0;

    virtual Chimera::Status_t start() = 0;

    virtual Chimera::Status_t abort() = 0;
  };
}    // namespace Thor::Driver::DMA

#endif /* !THOR_DRIVER_MODEL_DMA_HPP */