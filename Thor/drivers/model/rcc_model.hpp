/********************************************************************************
 *   File Name:
 *    rcc_model.hpp
 *
 *   Description:
 *    STM32 RCC interface modeling for the Thor driver
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_RCC_MODEL_HPP
#define THOR_DRIVER_RCC_MODEL_HPP

/* Chimera Includes */
#include <Chimera/types/common_types.hpp>
#include <Chimera/types/peripheral_types.hpp>

namespace Thor::Driver::RCC
{
  class Peripheral
  {
  public:
    virtual ~Peripheral() = default;
    
    /**
     *  Initializes the peripheral to a default configuration
     *  
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t init() = 0;

    /**
     *  Resets the peripheral using RCC reset registers
     *
     *  @param[in]  instance    The peripheral instance number
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t reset( const size_t instance ) = 0;

    /**
     *  Returns the type of peripheral that is being controlled
     *
     *  @param[in]  instance    The peripheral instance number
     *  @return Chimera::Peripheral::Type
     */
    virtual Chimera::Peripheral::Type getType() = 0;

    /**
     *  Enables the peripheral clock
     *
     *  @param[in]  instance    The peripheral instance number
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t enableClock( const size_t instance ) = 0;

    /**
     *  Disables the peripheral clock
     *
     *  @param[in]  instance    The peripheral instance number
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t disableClock( const size_t instance ) = 0;

    /**
     *  Enables the peripheral clock in low power mode
     *
     *  @param[in]  instance    The peripheral instance number
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t enableClockLowPower( const size_t instance ) = 0;

    /**
     *  Disables the peripheral clock in low power mode
     *
     *  @param[in]  instance    The peripheral instance number
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t disableClockLowPower( const size_t instance ) = 0;
  };
}    // namespace Thor::Driver::RCC

#endif /* !THOR_DRIVER_RCC_MODEL_HPP */