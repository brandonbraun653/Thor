/******************************************************************************
 *  File Name:
 *    usb_interrupt_vectors.hpp
 *
 *  Description:
 *    Provides ISR routine function prototypes that are common between all STM32
 *    families. These actually need to be implemented in whichever driver is
 *    used for Thor.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_USB_INTERRUPT_VECTORS_HPP
#define THOR_USB_INTERRUPT_VECTORS_HPP

#if defined( __cplusplus )
extern "C"
{
#endif

  void OTG_FS_IRQHandler();

#if defined( __cplusplus )
}
#endif

#endif /* !THOR_USB_INTERRUPT_VECTORS_HPP */