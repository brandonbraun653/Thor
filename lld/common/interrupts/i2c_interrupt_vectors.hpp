/******************************************************************************
 *  File Name:
 *    i2c_interrupt_vectors.hpp
 *
 *  Description:
 *    Provides ISR routine function prototypes that are common between all STM32
 *    families.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_I2C_INTERRUPT_VECTORS_HPP
#define THOR_I2C_INTERRUPT_VECTORS_HPP

#if defined( __cplusplus )
extern "C"
{
#endif

  void I2C1_EV_IRQHandler();
  void I2C1_ER_IRQHandler();
  void I2C2_EV_IRQHandler();
  void I2C2_ER_IRQHandler();
  void I2C3_EV_IRQHandler();
  void I2C3_ER_IRQHandler();

#if defined( __cplusplus )
}
#endif

#endif /* !THOR_I2C_INTERRUPT_VECTORS_HPP */
