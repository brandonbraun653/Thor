/********************************************************************************
 *   File Name:
 *    hw_rcc_driver_stm32f4.cpp
 *
 *   Description:
 *    Implements the low level driver for the Reset and Clock Control peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/types/peripheral_types.hpp>

/* Driver Includes */
#include <Thor/drivers/common/mapping/peripheral_mapping.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_driver.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_prj.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_types.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_mapping.hpp>
#include <Thor/drivers/model/rcc_model.hpp>

static constexpr uint8_t numPeriphs = static_cast<uint8_t>( Chimera::Peripheral::Type::NUM_SUPPORTED_TYPES );
static std::array<Thor::Driver::RCC::Peripheral *, numPeriphs> periphInstance;

namespace Thor::Driver::RCC
{
  static Peripheral *const getInstance( const Chimera::Peripheral::Type periph )
  {
    using namespace Thor::Driver::Mapping;
    
    auto iterator = PeriphTypeToIterator.find( periph );
    if ( iterator == PeriphTypeToIterator.end() )
    {
      return nullptr;
    }
    else
    {
      return periphInstance[ iterator->second ];
    }
  }

  void init()
  {
    periphInstance.fill( nullptr );
  }
  
  Chimera::Status_t enablePeripheralClock( const Chimera::Peripheral::Type periph, const size_t instance )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    if ( auto p = getInstance( periph ); p != nullptr )
    {
      result = p->enableClock( instance );
    }
    else
    {
      result = Chimera::CommonStatusCodes::FAIL;
    }

    return result;
  }

  GPIOPeriph::GPIOPeriph()
  {
    /*------------------------------------------------
    Register the singleton instance with the peripheral tracker
    ------------------------------------------------*/
    auto peripheralType        = Chimera::Peripheral::Type::GPIO;
    iterator                   = Thor::Driver::Mapping::PeriphTypeToIterator.find( peripheralType )->second;
    periphInstance[ iterator ] = reinterpret_cast<Peripheral *>( this );
  }

  GPIOPeriph::~GPIOPeriph()
  {
    free( periphInstance[ iterator ] );
    periphInstance[ iterator ] = nullptr;
  }

  Peripheral *const GPIOPeriph::get()
  {
    using namespace Thor::Driver::Mapping;

    /*------------------------------------------------
    Lookup the index where the GPIO singleton is stored
    ------------------------------------------------*/
    auto iterator = PeriphTypeToIterator.find( Chimera::Peripheral::Type::GPIO );
    
    if ( iterator == PeriphTypeToIterator.end() )
    {
      /*------------------------------------------------
      This peripheral type is not supported
      ------------------------------------------------*/
      return nullptr;
    }
    else
    {
      /*------------------------------------------------
      Create the new object if non-existant
      ------------------------------------------------*/
      if ( periphInstance[ iterator->second ] == nullptr )
      {
        periphInstance[ iterator->second ] = new GPIOPeriph();
      }

      return periphInstance[ iterator->second ];
    }
  }

  Chimera::Status_t GPIOPeriph::init()
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    return result;
  }

  Chimera::Status_t GPIOPeriph::reset( const size_t instance )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto prrConfig = ResetConfig_GPIO[ iterator ];
    uint32_t tmp   = *prrConfig.PRR;

    /*------------------------------------------------
    Begin the reset operation
    ------------------------------------------------*/
    tmp |= prrConfig.PRRMask;
    *prrConfig.PRR = tmp;

    /*------------------------------------------------
    Waste a few cycles to allow the reset to complete
    ------------------------------------------------*/
    // TODO

    /*------------------------------------------------
    Remove the reset flag as it is not cleared automatically by hardware
    ------------------------------------------------*/
    tmp &= ~prrConfig.PRRMask;
    *prrConfig.PRR = tmp;

    return result;
  }

  Chimera::Peripheral::Type GPIOPeriph::getType()
  {
    return Chimera::Peripheral::Type::GPIO;
  }

  Chimera::Status_t GPIOPeriph::enableClock( const size_t instance )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    Read-modify-write
    ------------------------------------------------*/
    auto ceConfig = ClockConfig_GPIO[ iterator ];
    *ceConfig.CER |= ceConfig.CERMask;

    return result;
  }

  Chimera::Status_t GPIOPeriph::disableClock( const size_t instance )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    Read-modify-write
    ------------------------------------------------*/
    auto ceConfig = ClockConfig_GPIO[ iterator ];
    *ceConfig.CER &= ~ceConfig.CERMask;
    
    return result;
  }

  Chimera::Status_t GPIOPeriph::enableClockLowPower( const size_t instance )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto celpConfig = ClockConfigLP_GPIO[ iterator ];
    *celpConfig.CELPR |= celpConfig.CELPMask;

    return result;
  }

  Chimera::Status_t GPIOPeriph::disableClockLowPower( const size_t instance )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto celpConfig = ClockConfigLP_GPIO[ iterator ];
    *celpConfig.CELPR &= ~celpConfig.CELPMask;
    
    return result;
  }
}    // namespace Thor::Driver::RCC
