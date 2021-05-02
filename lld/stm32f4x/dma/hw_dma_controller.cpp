/********************************************************************************
 *  File Name:
 *    hw_dma_controller.cpp
 *
 *  Description:
 *    DMA controller implementation
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Thor Includes */
#include <Thor/lld/interface/inc/dma>

namespace Thor::LLD::DMA
{
  /*-------------------------------------------------------------------------------
  Driver Class
  -------------------------------------------------------------------------------*/
  Driver::Driver() : periph( nullptr )
  {
  }

  Driver::~Driver()
  {
  }

  Chimera::Status_t Driver::attach( RegisterMap *const peripheral )
  {
    periph = peripheral;
    clockEnable();
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::init()
  {
    if ( !periph )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Reset the peripheral back to its default conditions
    ------------------------------------------------*/
    clockEnable();
    reset();
    clockEnable();

    /*------------------------------------------------
    Initialize all the stream objects for the DMA
    peripherals if one hasn't been created yet.
    ------------------------------------------------*/
    // static_assert( DMA_RESOURCE_INDEX_START == 0, "DMA1 resource index invalid" );

    // if ( periph == DMA1_PERIPH )
    // {
    //   for ( uint8_t x = DMA_RESOURCE_INDEX_START; x < DMA1_RESOURCE_INDEX_END; x++ )
    //   {
    //     if ( !streamObjects[ x ] )
    //     {
    //       /* x is already zero indexed, no need to convert it to get the proper stream */
    //       StreamMap *streamInstance = getStreamRegisters( periph, x );
    //       streamObjects[ x ]      = new StreamController();
    //       streamObjects[ x ]->attach( streamInstance, periph );
    //     }
    //   }
    // }
    // else if ( periph == DMA2_PERIPH )
    // {
    //   for ( uint8_t x = DMA2_RESOURCE_INDEX_START; x < DMA2_RESOURCE_INDEX_END; x++ )
    //   {
    //     if ( !streamObjects[ x ] )
    //     {
    //       /* Need to convert back to zero index to get the stream properly */
    //       auto streamInstance = getStreamRegisters( periph, ( x - DMA2_RESOURCE_INDEX_START ) );
    //       streamObjects[ x ]  = new StreamController();
    //       streamObjects[ x ]->attach( streamInstance, periph );
    //     }
    //   }
    // }

    return Chimera::Status::OK;
  }

  Chimera::Status_t Driver::configure( StreamMap *const stream, StreamConfig *const config, TCB *const controlBlock )
  {
    // auto streamIndex = StreamToResourceIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;

    // if ( streamObjects[ streamIndex ] )
    // {
    //   return streamObjects[ streamIndex ]->configure( config, controlBlock );
    // }
    // else
    // {
    //   return Chimera::Status::FAIL;
    // }
  }

  Chimera::Status_t Driver::start( StreamMap *const stream )
  {
    // auto streamIndex = StreamToResourceIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;

    // if ( streamObjects[ streamIndex ] )
    // {
    //   return streamObjects[ streamIndex ]->start();
    // }
    // else
    // {
    //   return Chimera::Status::FAIL;
    // }
  }

  Chimera::Status_t Driver::abort( StreamMap *const stream )
  {
    // auto streamIndex = StreamToResourceIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;

    // if ( streamObjects[ streamIndex ] )
    // {
    //   return streamObjects[ streamIndex ]->abort();
    // }
    // else
    // {
    //   return Chimera::Status::FAIL;
    // }
  }

}    // namespace Thor::LLD::DMA
