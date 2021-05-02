/********************************************************************************
 *  File Name:
 *    hw_dma_driver.cpp
 *
 *  Description:
 *    STM32F4 DMA Driver Implementation
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <cstddef>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/dma>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_DMA )

namespace Thor::LLD::DMA
{
  /*-------------------------------------------------------------------------------
  Public Functions (Interface)
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    return Chimera::Status::OK;
  }


  Driver_rPtr getDriver( const Controller control )
  {
    return nullptr;
  }


  Stream_rPtr getStream( const Controller control, const Streamer stream )
  {
    return nullptr;
  }


  StreamMap *const streamView( RegisterMap *const periph, const size_t streamNum )
  {
    /*------------------------------------------------
    This equation taken directly from register spec in datasheet.
    See 9.5.5 in RM0390
    ------------------------------------------------*/
    static constexpr size_t fixedOffset  = 0x10;
    static constexpr size_t streamOffset = 0x18;

    auto address = reinterpret_cast<std::uintptr_t>( periph ) + fixedOffset + ( streamOffset * streamNum );
    return reinterpret_cast<StreamMap *const>( address );
  }

}    // namespace Thor::LLD::DMA

#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */
