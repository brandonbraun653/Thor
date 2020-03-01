/********************************************************************************
 *  File Name:
 *    dma_mock.cpp
 *
 *  Description:
 *    Mocks the LLD DMA Interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( TARGET_LLD_MOCK )

#include <Thor/lld/interface/dma/dma.hpp>

namespace Thor::LLD::DMA
{
  void initialize()
  {
  
  }

  StreamController * getStreamController( const uint8_t resourceIndex )
  {
    return nullptr;
  }
}

#endif 