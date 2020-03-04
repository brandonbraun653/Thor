/********************************************************************************
 *  File Name:
 *    dma_mock.hpp
 *
 *  Description:
 *    Mocks the LLD DMA Interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_DMA_MOCK_HPP
#define THOR_LLD_DMA_MOCK_HPP

/* Google Includes */
#include "gmock/gmock.h"

/* Thor Includes */
#include <Thor/lld/interface/dma/dma_intf.hpp>
#include <Thor/lld/interface/dma/dma_types.hpp>

namespace Thor::LLD::DMA
{
  class StreamController;

  /**
   *  Initializes the low level driver
   *
   *  @return void
   */
  void initialize();

  /**
   *  Gets the current stream controller instance via lookup
   * 
   *  @param[in]  resourceIndex   LLD defined resource lookup index for a stream
   *  @return StreamController *  The instance of the controller
   */
  StreamController * getStreamController( const uint8_t resourceIndex );

  /**
   *  Models a stream within a DMA controller peripheral (channel)
   */
  class StreamController : public IStream, public Chimera::Threading::Lockable
  {
  public:
    MOCK_METHOD2( attach, Chimera::Status_t( StreamX *const, RegisterMap *const ) );
    MOCK_METHOD1( attachISRWakeup, Chimera::Status_t( Chimera::Threading::BinarySemaphore *const ) );
    MOCK_METHOD2( configure, Chimera::Status_t( StreamConfig *const, TCB *const ) );
    MOCK_METHOD0( start, Chimera::Status_t() );
    MOCK_METHOD0( abort, Chimera::Status_t() );
    MOCK_METHOD3( registerListener, Chimera::Status_t( Chimera::Event::Actionable &, const size_t, size_t & ) );
    MOCK_METHOD2( removeListener, Chimera::Status_t( const size_t, const size_t ) );
  };

  /**
   *  Models the interface to a full DMA controller, which is composed of many streams.
   *  For the STM32F4xxx chips, there is typically seven streams per channel.
   */
  class ChannelController : public IPeripheral
  {
  public:
    MOCK_METHOD1( attach, Chimera::Status_t( RegisterMap *const ) );
    MOCK_METHOD0( clockEnable, Chimera::Status_t() );
    MOCK_METHOD0( clockDisable, Chimera::Status_t() );
    MOCK_METHOD0( reset, Chimera::Status_t() );
    MOCK_METHOD0( init, Chimera::Status_t() );
    MOCK_METHOD3( configure, Chimera::Status_t( StreamX *const, StreamConfig *const, TCB *const ) );
    MOCK_METHOD1( start, Chimera::Status_t( StreamX *const ) );
    MOCK_METHOD1( abort, Chimera::Status_t( StreamX *const ) );
    MOCK_METHOD4( registerListener, Chimera::Status_t( StreamX *const, Chimera::Event::Actionable &, const size_t, size_t & ) );
    MOCK_METHOD3( removeListener, Chimera::Status_t( StreamX *const, const size_t, const size_t ) );
  };

}

#endif  /* !THOR_LLD_DMA_MOCK_HPP */
