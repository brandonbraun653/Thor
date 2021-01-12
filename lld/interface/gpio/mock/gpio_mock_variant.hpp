/********************************************************************************
 *  File Name:
 *    gpio_mock_variant.hpp
 *
 *  Description:
 *    Mock variant of the GPIO hardware
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_GPIO_MOCK_VARIANT_HPP
#define THOR_LLD_GPIO_MOCK_VARIANT_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/interface/gpio/gpio_intf.hpp>
#include <Thor/lld/interface/gpio/gpio_types.hpp>

namespace Thor::LLD::GPIO
{
  /*-------------------------------------------------------------------------------
  Literals
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_GPIO_PERIPHS = 15;


  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct RegisterMap
  {
    uint32_t MODER;   /**< GPIO port mode register */
    uint32_t OTYPER;  /**< GPIO port output type register */
    uint32_t OSPEEDR; /**< GPIO port output speed register */
    uint32_t PUPDR;   /**< GPIO port pull-up/pull-down register */
    uint32_t IDR;     /**< GPIO port input data register */
    uint32_t ODR;     /**< GPIO port output data register */
    uint32_t BSRR;    /**< GPIO port bit set/reset register */
    uint32_t LCKR;    /**< GPIO port configuration lock register */
    uint64_t AFR;     /**< GPIO alternate function registers */
    uint32_t BRR;     /**< GPIO port bit reset register */
  };


  /*-------------------------------------------------------------------------------
  External Variables
  -------------------------------------------------------------------------------*/
  extern std::array<RegisterMap*, NUM_GPIO_PERIPHS> PeripheralRegisterMaps;


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  void initializeMapping();


}  // namespace Thor::LLD::GPIO

#endif  /* !THOR_LLD_GPIO_MOCK_VARIANT_HPP */
