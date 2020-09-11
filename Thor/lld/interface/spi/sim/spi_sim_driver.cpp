/********************************************************************************
 *  File Name:
 *    spi_sim_driver.cpp
 *
 *  Description:
 *    Simulator driver for SPI
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/spi/spi_prv_data.hpp>
#include <Thor/lld/interface/spi/spi_intf.hpp>
#include <Thor/lld/interface/spi/spi_types.hpp>

#if defined( TARGET_LLD_TEST ) && defined( THOR_LLD_SPI )

namespace Thor::LLD::SPI
{
  /*-------------------------------------------------------------------------------
  Variables
  -------------------------------------------------------------------------------*/
  static Driver s_spi_drivers[ NUM_SPI_PERIPHS ];

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    initializeRegisters();

    /*-------------------------------------------------
    Attach all the expected peripherals to the drivers
    -------------------------------------------------*/
    if ( attachDriverInstances( s_spi_drivers, ARRAY_COUNT( s_spi_drivers ) ) )
    {
      return Chimera::Status::OK;
    }
    else
    {
      return Chimera::Status::FAIL;
    }
  }


  Driver_rPtr getDriver( const Chimera::SPI::Channel channel )
  {
    if ( auto idx = getResourceIndex( channel ); idx != INVALID_RESOURCE_INDEX )
    {
      return &s_spi_drivers[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  /*-------------------------------------------------------------------------------
  Low Level Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver() :
      ISRWakeup_external( nullptr ), mPeriph( nullptr ), periphConfig( nullptr ),
      resourceIndex( std::numeric_limits<size_t>::max() )
  {
    memset( &txfr, 0, sizeof( txfr ) );
    txfr.status = Chimera::SPI::Status::TRANSFER_COMPLETE;
  }

  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::attach( RegisterMap *const peripheral )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::reset()
  {
    return Chimera::Status::OK;
  }


  void Driver::clockEnable()
  {
  }


  void Driver::clockDisable()
  {
  }


  size_t Driver::getErrorFlags()
  {
    return 0;
  }


  size_t Driver::getStatusFlags()
  {
    return 0;
  }


  Chimera::Status_t Driver::configure( const Chimera::SPI::DriverConfig &setup )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::registerConfig( Chimera::SPI::DriverConfig *config )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::transfer( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::transferIT( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::transferDMA( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::killTransfer()
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  void Driver::attachISRWakeup( Chimera::Threading::BinarySemaphore *const wakeup )
  {
    ISRWakeup_external = wakeup;
  }


  HWTransfer Driver::getTransferBlock()
  {
    return HWTransfer();
  }


  inline void Driver::enterCriticalSection()
  {
  }


  inline void Driver::exitCriticalSection()
  {
  }


  void Driver::IRQHandler()
  {
  }

}  // namespace Thor::LLD::SPI

#endif
