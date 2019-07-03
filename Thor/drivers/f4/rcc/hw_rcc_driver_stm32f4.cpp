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
#include <cstring>

/* Chimera Includes */
#include <Chimera/interface/compiler_intf.hpp>
#include <Chimera/types/peripheral_types.hpp>

/* Driver Includes */
#include <Thor/drivers/common/mapping/peripheral_mapping.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_driver.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_prj.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_types.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_mapping.hpp>
#include <Thor/drivers/model/rcc_model.hpp>

static constexpr uint8_t numPeriphs = static_cast<uint8_t>( Chimera::Peripheral::Type::NUM_SUPPORTED_TYPES );
static std::array<Thor::Driver::RCC::Peripheral *, numPeriphs> periphSingletonInstances;

namespace Thor::Driver::RCC
{
  static Peripheral *const getPeriphInstance( const Chimera::Peripheral::Type periph )
  {
    using namespace Thor::Driver::Mapping;

    /*------------------------------------------------
    Find the iterator associated with the requested peripheral
    ------------------------------------------------*/
    auto iterator = PeriphTypeToIterator.find( periph );
    if ( iterator == PeriphTypeToIterator.end() )
    {
      return nullptr;
    }
    else
    {
      return periphSingletonInstances[ iterator->second ];
    }
  }

  void init()
  {
    static bool initialized = false;

    if ( !initialized )
    {
      periphSingletonInstances.fill( nullptr );
    }
  }

  /**
   *  Clock Configuration RCC Driver
   */

  static inline Chimera::Status_t HSEConfig( const OscInit *init )
  {
    auto result = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    When the HSE is used as system clock or configured as the 
    clock source for PLL, the HSE will not be disabled.
    ------------------------------------------------*/
    const auto clockSource = CFGR::SWS::get();
    const auto pllSource   = PLLCFGR::SRC::get();

    if ( ( clockSource == CFGR::SWS::HSE ) || ( ( clockSource == CFGR::SWS::PLL ) && ( pllSource == PLLCFGR::SRC::HSE ) ) )
    {
      if ( ( ( CR::HSERDY::get() == CR::locked ) || ( CR::HSEON::get() == CR::disabled ) ) &&
           ( init->HSEState == CR::HSEConfig::OFF ) )
      {
        result = Chimera::CommonStatusCodes::FAIL;
      }
    }
    else
    {
//      /* Set the new HSE configuration ---------------------------------------*/
//      __HAL_RCC_HSE_CONFIG( RCC_OscInitStruct->HSEState );
//
//      /* Check the HSE State */
//      if ( ( RCC_OscInitStruct->HSEState ) != RCC_HSE_OFF )
//      {
//        /* Get Start Tick */
//        tickstart = HAL_GetTick();
//
//        /* Wait till HSE is ready */
//        while ( __HAL_RCC_GET_FLAG( RCC_FLAG_HSERDY ) == RESET )
//        {
//          if ( ( HAL_GetTick() - tickstart ) > HSE_TIMEOUT_VALUE )
//          {
//            return HAL_TIMEOUT;
//          }
//        }
//      }
//      else
//      {
//        /* Get Start Tick */
//        tickstart = HAL_GetTick();
//
//        /* Wait till HSE is bypassed or disabled */
//        while ( __HAL_RCC_GET_FLAG( RCC_FLAG_HSERDY ) != RESET )
//        {
//          if ( ( HAL_GetTick() - tickstart ) > HSE_TIMEOUT_VALUE )
//          {
//            return HAL_TIMEOUT;
//          }
//        }
//      }
    }

    return result;
  }

  static inline Chimera::Status_t HSIConfig( const OscInit *init )
  {
    //    /* Check if HSI is used as system clock or as PLL source when PLL is selected as system clock */
    //    if ( ( __HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_HSI ) ||
    //         ( ( __HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_PLL ) &&
    //           ( ( RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC ) == RCC_PLLCFGR_PLLSRC_HSI ) ) )
    //    {
    //      /* When HSI is used as system clock it will not disabled */
    //      if ( ( __HAL_RCC_GET_FLAG( RCC_FLAG_HSIRDY ) != RESET ) && ( RCC_OscInitStruct->HSIState != RCC_HSI_ON ) )
    //      {
    //        return HAL_ERROR;
    //      }
    //      /* Otherwise, just the calibration is allowed */
    //      else
    //      {
    //        /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
    //        __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST( RCC_OscInitStruct->HSICalibrationValue );
    //      }
    //    }
    //    else
    //    {
    //      /* Check the HSI State */
    //      if ( ( RCC_OscInitStruct->HSIState ) != RCC_HSI_OFF )
    //      {
    //        /* Enable the Internal High Speed oscillator (HSI). */
    //        __HAL_RCC_HSI_ENABLE();
    //
    //        /* Get Start Tick*/
    //        tickstart = HAL_GetTick();
    //
    //        /* Wait till HSI is ready */
    //        while ( __HAL_RCC_GET_FLAG( RCC_FLAG_HSIRDY ) == RESET )
    //        {
    //          if ( ( HAL_GetTick() - tickstart ) > HSI_TIMEOUT_VALUE )
    //          {
    //            return HAL_TIMEOUT;
    //          }
    //        }
    //
    //        /* Adjusts the Internal High Speed oscillator (HSI) calibration value. */
    //        __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST( RCC_OscInitStruct->HSICalibrationValue );
    //      }
    //      else
    //      {
    //        /* Disable the Internal High Speed oscillator (HSI). */
    //        __HAL_RCC_HSI_DISABLE();
    //
    //        /* Get Start Tick*/
    //        tickstart = HAL_GetTick();
    //
    //        /* Wait till HSI is ready */
    //        while ( __HAL_RCC_GET_FLAG( RCC_FLAG_HSIRDY ) != RESET )
    //        {
    //          if ( ( HAL_GetTick() - tickstart ) > HSI_TIMEOUT_VALUE )
    //          {
    //            return HAL_TIMEOUT;
    //          }
    //        }
    //      }
    //    }
  }

  static inline Chimera::Status_t LSIConfig( const OscInit *init )
  {
    //    /* Check the LSI State */
    //    if ( ( RCC_OscInitStruct->LSIState ) != RCC_LSI_OFF )
    //    {
    //      /* Enable the Internal Low Speed oscillator (LSI). */
    //      __HAL_RCC_LSI_ENABLE();
    //
    //      /* Get Start Tick*/
    //      tickstart = HAL_GetTick();
    //
    //      /* Wait till LSI is ready */
    //      while ( __HAL_RCC_GET_FLAG( RCC_FLAG_LSIRDY ) == RESET )
    //      {
    //        if ( ( HAL_GetTick() - tickstart ) > LSI_TIMEOUT_VALUE )
    //        {
    //          return HAL_TIMEOUT;
    //        }
    //      }
    //    }
    //    else
    //    {
    //      /* Disable the Internal Low Speed oscillator (LSI). */
    //      __HAL_RCC_LSI_DISABLE();
    //
    //      /* Get Start Tick */
    //      tickstart = HAL_GetTick();
    //
    //      /* Wait till LSI is ready */
    //      while ( __HAL_RCC_GET_FLAG( RCC_FLAG_LSIRDY ) != RESET )
    //      {
    //        if ( ( HAL_GetTick() - tickstart ) > LSI_TIMEOUT_VALUE )
    //        {
    //          return HAL_TIMEOUT;
    //        }
    //      }
    //    }
  }

  static inline Chimera::Status_t LSEConfig( const OscInit *init )
  {
    //    FlagStatus pwrclkchanged = RESET;
    //
    //    /* Check the parameters */
    //    assert_param( IS_RCC_LSE( RCC_OscInitStruct->LSEState ) );
    //
    //    /* Update LSE configuration in Backup Domain control register    */
    //    /* Requires to enable write access to Backup Domain of necessary */
    //    if ( __HAL_RCC_PWR_IS_CLK_DISABLED() )
    //    {
    //      __HAL_RCC_PWR_CLK_ENABLE();
    //      pwrclkchanged = SET;
    //    }
    //
    //    if ( HAL_IS_BIT_CLR( PWR->CR, PWR_CR_DBP ) )
    //    {
    //      /* Enable write access to Backup domain */
    //      SET_BIT( PWR->CR, PWR_CR_DBP );
    //
    //      /* Wait for Backup domain Write protection disable */
    //      tickstart = HAL_GetTick();
    //
    //      while ( HAL_IS_BIT_CLR( PWR->CR, PWR_CR_DBP ) )
    //      {
    //        if ( ( HAL_GetTick() - tickstart ) > RCC_DBP_TIMEOUT_VALUE )
    //        {
    //          return HAL_TIMEOUT;
    //        }
    //      }
    //    }
    //
    //    /* Set the new LSE configuration -----------------------------------------*/
    //    __HAL_RCC_LSE_CONFIG( RCC_OscInitStruct->LSEState );
    //    /* Check the LSE State */
    //    if ( ( RCC_OscInitStruct->LSEState ) != RCC_LSE_OFF )
    //    {
    //      /* Get Start Tick*/
    //      tickstart = HAL_GetTick();
    //
    //      /* Wait till LSE is ready */
    //      while ( __HAL_RCC_GET_FLAG( RCC_FLAG_LSERDY ) == RESET )
    //      {
    //        if ( ( HAL_GetTick() - tickstart ) > RCC_LSE_TIMEOUT_VALUE )
    //        {
    //          return HAL_TIMEOUT;
    //        }
    //      }
    //    }
    //    else
    //    {
    //      /* Get Start Tick */
    //      tickstart = HAL_GetTick();
    //
    //      /* Wait till LSE is ready */
    //      while ( __HAL_RCC_GET_FLAG( RCC_FLAG_LSERDY ) != RESET )
    //      {
    //        if ( ( HAL_GetTick() - tickstart ) > RCC_LSE_TIMEOUT_VALUE )
    //        {
    //          return HAL_TIMEOUT;
    //        }
    //      }
    //    }
    //
    //    /* Restore clock configuration if changed */
    //    if ( pwrclkchanged == SET )
    //    {
    //      __HAL_RCC_PWR_CLK_DISABLE();
    //    }
  }

  static inline Chimera::Status_t PLLConfig( const OscInit *init )
  {
    //    /*-------------------------------- PLL Configuration -----------------------*/
    //    /* Check the parameters */
    //    assert_param( IS_RCC_PLL( RCC_OscInitStruct->PLL.PLLState ) );
    //    if ( ( RCC_OscInitStruct->PLL.PLLState ) != RCC_PLL_NONE )
    //    {
    //      /* Check if the PLL is used as system clock or not */
    //      if ( __HAL_RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS_PLL )
    //      {
    //        if ( ( RCC_OscInitStruct->PLL.PLLState ) == RCC_PLL_ON )
    //        {
    //          /* Check the parameters */
    //          assert_param( IS_RCC_PLLSOURCE( RCC_OscInitStruct->PLL.PLLSource ) );
    //          assert_param( IS_RCC_PLLM_VALUE( RCC_OscInitStruct->PLL.PLLM ) );
    //          assert_param( IS_RCC_PLLN_VALUE( RCC_OscInitStruct->PLL.PLLN ) );
    //          assert_param( IS_RCC_PLLP_VALUE( RCC_OscInitStruct->PLL.PLLP ) );
    //          assert_param( IS_RCC_PLLQ_VALUE( RCC_OscInitStruct->PLL.PLLQ ) );
    //
    //          /* Disable the main PLL. */
    //          __HAL_RCC_PLL_DISABLE();
    //
    //          /* Get Start Tick */
    //          tickstart = HAL_GetTick();
    //
    //          /* Wait till PLL is ready */
    //          while ( __HAL_RCC_GET_FLAG( RCC_FLAG_PLLRDY ) != RESET )
    //          {
    //            if ( ( HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE )
    //            {
    //              return HAL_TIMEOUT;
    //            }
    //          }
    //
    //          /* Configure the main PLL clock source, multiplication and division factors. */
    //          WRITE_REG( RCC->PLLCFGR, ( RCC_OscInitStruct->PLL.PLLSource | RCC_OscInitStruct->PLL.PLLM |
    //                                     ( RCC_OscInitStruct->PLL.PLLN << RCC_PLLCFGR_PLLN_Pos ) |
    //                                     ( ( ( RCC_OscInitStruct->PLL.PLLP >> 1U ) - 1U ) << RCC_PLLCFGR_PLLP_Pos ) |
    //                                     ( RCC_OscInitStruct->PLL.PLLQ << RCC_PLLCFGR_PLLQ_Pos ) ) );
    //          /* Enable the main PLL. */
    //          __HAL_RCC_PLL_ENABLE();
    //
    //          /* Get Start Tick */
    //          tickstart = HAL_GetTick();
    //
    //          /* Wait till PLL is ready */
    //          while ( __HAL_RCC_GET_FLAG( RCC_FLAG_PLLRDY ) == RESET )
    //          {
    //            if ( ( HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE )
    //            {
    //              return HAL_TIMEOUT;
    //            }
    //          }
    //        }
    //        else
    //        {
    //          /* Disable the main PLL. */
    //          __HAL_RCC_PLL_DISABLE();
    //
    //          /* Get Start Tick */
    //          tickstart = HAL_GetTick();
    //
    //          /* Wait till PLL is ready */
    //          while ( __HAL_RCC_GET_FLAG( RCC_FLAG_PLLRDY ) != RESET )
    //          {
    //            if ( ( HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE )
    //            {
    //              return HAL_TIMEOUT;
    //            }
    //          }
    //        }
    //      }
    //      else
    //      {
    //        return HAL_ERROR;
    //      }
    //    }
  }

  Chimera::Status_t OscConfig( OscInit *init )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    if ( !init )
    {
      result = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }
    else
    {
      switch ( init->source )
      {
        case OscillatorSource::HSE:
          result = HSEConfig( init );
          break;

        case OscillatorSource::HSI:
          result = HSIConfig( init );
          break;

        case OscillatorSource::LSE:
          result = LSEConfig( init );
          break;

        case OscillatorSource::LSI:
          result = LSIConfig( init );
          break;

        default:
          result = Chimera::CommonStatusCodes::FAIL;
          break;
      }
    }

    return result;
  }


  WEAKDECL OscInit prjGetOscConfig()
  {
    OscInit dummyConfig;
    memset( &dummyConfig, 0, sizeof( OscInit ) );
    return dummyConfig;
  }

  /**
   *  SystemClock Class Implementation
   */
  SystemClock::SystemClock()
  {
  }

  SystemClock::~SystemClock()
  {
  }

  SystemClock *const SystemClock::get()
  {
    static SystemClock *ref = nullptr;

    if ( ref == nullptr )
    {
      ref = new SystemClock();
    }

    return ref;
  }

  Chimera::Status_t SystemClock::setPeriphClock( const Chimera::Peripheral::Type periph, const size_t freqHz )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::FAIL;

    return result;
  }

  Chimera::Status_t SystemClock::setCoreClock( const size_t freqHz )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::FAIL;

    return result;
  }

  Chimera::Status_t SystemClock::setCoreClockSource( const Thor::Clock::Source src )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::FAIL;

    auto cfg = prjGetOscConfig();

    OscConfig( &cfg );

    return result;
  }

  size_t SystemClock::getCoreClock()
  {
    return 0;
  }

  Thor::Clock::Source SystemClock::getCoreClockSource()
  {
    return Thor::Clock::Source::CSI;
  }

  size_t SystemClock::getPeriphClock( const Chimera::Peripheral::Type periph )
  {
    return 0;
  }


  /**
   *  GPIOPeriph Class Implementation
   */

  GPIOPeriph::GPIOPeriph() : iterator( 0 )
  {
    /*------------------------------------------------
    Register the singleton instance with the peripheral tracker
    ------------------------------------------------*/
    auto peripheralType                  = Chimera::Peripheral::Type::GPIO;
    iterator                             = Thor::Driver::Mapping::PeriphTypeToIterator.find( peripheralType )->second;
    periphSingletonInstances[ iterator ] = reinterpret_cast<Peripheral *>( this );
  }

  GPIOPeriph::~GPIOPeriph()
  {
    free( periphSingletonInstances[ iterator ] );
    periphSingletonInstances[ iterator ] = nullptr;
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
      if ( periphSingletonInstances[ iterator->second ] == nullptr )
      {
        periphSingletonInstances[ iterator->second ] = new GPIOPeriph();
      }

      return periphSingletonInstances[ iterator->second ];
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
    uint32_t tmp   = *prrConfig.reg;

    /*------------------------------------------------
    Begin the reset operation
    ------------------------------------------------*/
    tmp |= prrConfig.mask;
    *prrConfig.reg = tmp;

    /*------------------------------------------------
    Waste a few cycles to allow the reset to complete
    ------------------------------------------------*/
    // TODO

    /*------------------------------------------------
    Remove the reset flag as it is not cleared automatically by hardware
    ------------------------------------------------*/
    tmp &= ~prrConfig.mask;
    *prrConfig.reg = tmp;

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
    *ceConfig.reg |= ceConfig.mask;

    return result;
  }

  Chimera::Status_t GPIOPeriph::disableClock( const size_t instance )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    Read-modify-write
    ------------------------------------------------*/
    auto ceConfig = ClockConfig_GPIO[ iterator ];
    *ceConfig.reg &= ~ceConfig.mask;

    return result;
  }

  Chimera::Status_t GPIOPeriph::enableClockLowPower( const size_t instance )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    Read-modify-write
    ------------------------------------------------*/
    auto celpConfig = ClockConfigLP_GPIO[ iterator ];
    *celpConfig.reg |= celpConfig.mask;

    return result;
  }

  Chimera::Status_t GPIOPeriph::disableClockLowPower( const size_t instance )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    Read-modify-write
    ------------------------------------------------*/
    auto celpConfig = ClockConfigLP_GPIO[ iterator ];
    *celpConfig.reg &= ~celpConfig.mask;

    return result;
  }
}    // namespace Thor::Driver::RCC
