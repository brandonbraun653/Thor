/********************************************************************************
 *  File Name:
 *    hw_rcc_driver_stm32l4.cpp
 *
 *  Description:
 *    RCC Low Level driver for the STM32L4 series chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* C++ Includes */
#include <array>
#include <cstring>
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/clock>
#include <Chimera/system>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/common/mapping/peripheral_mapping.hpp>
#include <Thor/lld/interface/rcc/rcc_intf.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_driver.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_prj.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_mapping.hpp>


#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_RCC )

namespace Thor::LLD::RCC
{
  /*------------------------------------------------
  Local Variables and Constants
  ------------------------------------------------*/
  static constexpr uint8_t numPeriphs = static_cast<uint8_t>( Chimera::Peripheral::Type::NUM_SUPPORTED_TYPES );

  /**
   *  Lookup table for all RCC peripheral control registers.
   */
  static std::array<PCC *, numPeriphs> periphLookupTables;

  /*------------------------------------------------
  Standalone Functions
  ------------------------------------------------*/
  void initialize()
  {
    using namespace Chimera::Peripheral;
    using namespace Thor::LLD::RCC;

    static bool initialized = false;

    if ( !initialized )
    {
      initializeRegisters();
      initializeMapping();

      /*------------------------------------------------
      Register the lookup tables with the system
      ------------------------------------------------*/
      periphLookupTables.fill( nullptr );

#if defined( THOR_LLD_DMA )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_DMA ) ] = &LookupTables::DMALookup;
#endif

#if defined( THOR_LLD_GPIO )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_GPIO ) ] = &LookupTables::GPIOLookup;
#endif

#if defined( THOR_LLD_SPI )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_SPI ) ] = &LookupTables::SPILookup;
#endif

#if defined( THOR_LLD_UART )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_UART ) ] = &LookupTables::UARTLookup;
#endif

#if defined( THOR_LLD_USART )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_USART ) ] = &LookupTables::USARTLookup;
#endif

#if defined( THOR_LLD_WWDG )
      periphLookupTables[ static_cast<uint8_t>( Type::PERIPH_WWDG ) ] = &LookupTables::WWDGLookup;
#endif

      initialized = true;
    }
  }

  void clearResetReason()
  {
    RMVFRSTF::set( RCC1_PERIPH, CSR_RMVF );
  }

  Chimera::System::ResetEvent getResetReason()
  {
    /*------------------------------------------------
    Read out the flag bits and then clear them to ensure we 
    get an accurate read the next time this function is called.
    ------------------------------------------------*/
    Reg32_t flags = RCC1_PERIPH->CSR & CSR_ResetFlags_Msk;
    clearResetReason();

    /*------------------------------------------------
    When debugging and powering on the board for the first time, usually there
    are two reset flags set. One is the brown out, the other is the pin reset.
    If more than just the brown out flag has been set, it's safe to mask it away 
    as a false positive. This is known to happen on the STM32 development boards.
    ------------------------------------------------*/
    if ( ( flags & ResetFlags::BROWN_OUT ) && ( flags != ResetFlags::BROWN_OUT ) ) 
    {
      flags &= ~ResetFlags::BROWN_OUT;
    }

    switch ( flags )
    {
      case ResetFlags::CLEARED:
        return Chimera::System::ResetEvent::CLEARED;
        break;

      case ResetFlags::PIN_RESET:
        return Chimera::System::ResetEvent::HW_EXTERNAL_PIN;
        break;

      case ResetFlags::IWDG:
        return Chimera::System::ResetEvent::HW_INDEPENDENT_WATCHDOG_TIMEOUT;
        break;

      case ResetFlags::WWDG:
        return Chimera::System::ResetEvent::HW_WINDOW_WATCHDOG_TIMEOUT;
        break;

      case ResetFlags::SOFTWARE:
        return Chimera::System::ResetEvent::SOFTWARE;
        break;

      case ResetFlags::BROWN_OUT:
        return Chimera::System::ResetEvent::BROWN_OUT;
        break;

      case ResetFlags::LOW_POWER:
      case ResetFlags::OPTION_BYTE:
      case ResetFlags::FIREWALL:
        return Chimera::System::ResetEvent::UNKNOWN;
        break;

      default:
        return Chimera::System::ResetEvent::NOT_SUPPORTED;
        break;
    }
  }


  /*------------------------------------------------
  SystemClock Class Implementation
  ------------------------------------------------*/
  IClockTree *getSystemClockController()
  {
    static SystemClock *ref = nullptr;
    if ( ref == nullptr )
    {
      ref = new SystemClock();
    }

    return ref;
  }

  SystemClock::SystemClock()
  {
    initialize();
  }

  SystemClock::~SystemClock()
  {
  }

  Chimera::Status_t SystemClock::configureProjectClocks()
  {
    using namespace Thor::Driver;

    Chimera::Status_t result          = Chimera::CommonStatusCodes::FAIL;
    const Chimera::Status_t prjResult = Chimera::CommonStatusCodes::OK;

    //    /*------------------------------------------------
    //    Turn on the main internal regulator output voltage
    //    ------------------------------------------------*/
    //    APB1ENR::PWREN::set( RCC1_PERIPH, APB1ENR::PWRENConfig::ON );
    //
    //    /*------------------------------------------------
    //    Set the voltage scaling to allow us to achieve max clock
    //    ------------------------------------------------*/
    //    PWR::CR::VOS::set( Thor::LLD::PWR::PWR_PERIPH, PWR::CR::VOS::VOLTAGE_SCALE_1 );
    //
    //    /*------------------------------------------------
    //    Configure the system clocks
    //    ------------------------------------------------*/
    //    ClockInit clkCfg;
    //    OscillatorInit oscCfg;
    //
    //    if ( ( prjGetOscillatorConfig( &oscCfg ) == prjResult ) && ( prjGetClockConfig( &clkCfg ) == prjResult ) )
    //    {
    //      /*------------------------------------------------
    //      Initialize the oscillators which drive the system clocks
    //      ------------------------------------------------*/
    //      result = OscillatorConfig( &oscCfg );
    //
    //      /*------------------------------------------------
    //      Initializes the CPU, AHB, and APB bus clocks
    //      ------------------------------------------------*/
    //      result = ClockConfig( &clkCfg );
    //    }

    return result;
  }

  Chimera::Status_t SystemClock::setPeriphClock( const Chimera::Peripheral::Type periph, const size_t freqHz )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::NOT_SUPPORTED;
    return result;
  }

  Chimera::Status_t SystemClock::setCoreClock( const size_t freqHz )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::NOT_SUPPORTED;
    return result;
  }

  Chimera::Status_t SystemClock::setCoreClockSource( const Thor::Clock::Source src )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::NOT_SUPPORTED;
    return result;
  }

  Chimera::Status_t SystemClock::getClockFrequency( const ClockType_t clock, size_t *const freqHz )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::FAIL;

    //    if ( freqHz )
    //    {
    //      switch ( clock )
    //      {
    //        case Configuration::ClockType::HCLK:
    //          result = prjGetHCLKFreq( freqHz );
    //          break;
    //
    //        case Configuration::ClockType::PCLK1:
    //          result = prjGetPCLK1Freq( freqHz );
    //          break;
    //
    //        case Configuration::ClockType::PCLK2:
    //          result = prjGetPCLK2Freq( freqHz );
    //          break;
    //
    //        case Configuration::ClockType::SYSCLK:
    //          result = prjGetSysClockFreq( freqHz );
    //          break;
    //
    //        default:
    //          // result = Chimera::CommonStatusCodes::FAIL;
    //          break;
    //      }
    //    }

    return result;
  }

  Chimera::Status_t SystemClock::getPeriphClock( const Chimera::Peripheral::Type periph, const std::uintptr_t address,
                                                 size_t *const freqHz )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::FAIL;

    auto clockLookupTable = periphLookupTables[ static_cast<uint8_t>( periph ) ]->clockSource;
    auto indexLookupTable = periphLookupTables[ static_cast<uint8_t>( periph ) ]->resourceIndexMap;

    // auto tmp = reinterpret_cast<Chimera::Container::LightFlatMap<std::uintptr_t, size_t, NUM_DMA_PERIPHS>

    if ( freqHz && clockLookupTable && indexLookupTable )
    {
      auto index       = indexLookupTable->find( address )->second;
      auto sourceClock = clockLookupTable[ index ];

      result = getClockFrequency( sourceClock, freqHz );
    }

    return result;
  }

  /*------------------------------------------------
  PeripheralController Class Implementation
  ------------------------------------------------*/
  IPeripheralController *getSystemPeripheralController()
  {
    static PeripheralController *ref = nullptr;

    if ( !ref )
    {
      ref = new PeripheralController();
    }

    return ref;
  }

  PeripheralController::PeripheralController()
  {
    initialize();
  }

  PeripheralController::~PeripheralController()
  {
  }

  Chimera::Status_t PeripheralController::reset( const Chimera::Peripheral::Type type, const size_t index )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto lookupTable = periphLookupTables[ static_cast<uint8_t>( type ) ]->reset;
    auto config      = lookupTable[ index ];
    uint32_t tmp     = *config.reg;

    /*------------------------------------------------
    Begin the reset operation
    ------------------------------------------------*/
    tmp |= config.mask;
    *config.reg = tmp;

    /*------------------------------------------------
    Remove the reset flag as it is not cleared automatically by hardware
    ------------------------------------------------*/
    tmp &= ~config.mask;
    *config.reg = tmp;

    return result;
  }

  Chimera::Status_t PeripheralController::enableClock( const Chimera::Peripheral::Type type, const size_t index )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto lookupTable = periphLookupTables[ static_cast<uint8_t>( type ) ]->clock;
    auto config      = lookupTable[ index ];
    *config.reg |= config.mask;

    return result;
  }

  Chimera::Status_t PeripheralController::disableClock( const Chimera::Peripheral::Type type, const size_t index )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto lookupTable = periphLookupTables[ static_cast<uint8_t>( type ) ]->clock;
    auto config      = lookupTable[ index ];
    *config.reg &= ~config.mask;

    return result;
  }

  Chimera::Status_t PeripheralController::enableClockLowPower( const Chimera::Peripheral::Type type, const size_t index )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto lookupTable = periphLookupTables[ static_cast<uint8_t>( type ) ]->clockLP;
    auto config      = lookupTable[ index ];
    *config.reg |= config.mask;

    return result;
  }

  Chimera::Status_t PeripheralController::disableClockLowPower( const Chimera::Peripheral::Type type, const size_t index )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    auto lookupTable = periphLookupTables[ static_cast<uint8_t>( type ) ]->clockLP;
    auto config      = lookupTable[ index ];
    *config.reg &= ~config.mask;

    return result;
  }
}    // namespace Thor::LLD::RCC

#endif /* TARGET_STM32L4 && THOR_LLD_RCC */
