/********************************************************************************
 *  File Name:
 *    can_interrupt_vectors.hpp
 *
 *  Description:
 *    Provides ISR routine function prototypes that are common between all STM32
 *    families.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_CAN_INTERRUPT_VECTORS_HPP
#define THOR_CAN_INTERRUPT_VECTORS_HPP

#if defined( __cplusplus )
extern "C"
{
#endif

  void CAN1_TX_IRQHandler();
  void CAN1_FIFO0_IRQHandler();
  void CAN1_FIFO1_IRQHandler();
  void CAN1_ERR_STS_CHG_IRQHandler();

#if defined( __cplusplus )
}
#endif

#endif /* !THOR_CAN_INTERRUPT_VECTORS_HPP */
