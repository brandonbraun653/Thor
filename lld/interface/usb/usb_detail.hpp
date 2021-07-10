/********************************************************************************
 *  File Name:
 *    usb_detail.hpp
 *
 *  Description:
 *    Common header for Thor USB that configures the driver based on which
 *    chip family is being compiled against.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_LLD_USB_INTF_DETAIL_HPP
#define THOR_LLD_USB_INTF_DETAIL_HPP

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/usb/mock/usb_mock.hpp>
#include <Thor/lld/interface/usb/mock/usb_mock_variant.hpp>
#elif defined( TARGET_LLD_TEST )
#include <Thor/lld/interface/usb/sim/usb_sim_variant.hpp>
#include <Thor/lld/interface/usb/sim/usb_sim_types.hpp>
#elif defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/usb/hw_usb_prj.hpp>
#elif defined( TARGET_STM32L4 )
#include <Thor/lld/stm32l4x/usb/hw_usb_prj.hpp>
#include <Thor/lld/stm32l4x/usb/hw_usb_types.hpp>
#else
#pragma message( "usb_detail.hpp: Unknown target for LLD" )
#endif

#endif /* !THOR_LLD_USB_INTF_DETAIL_HPP */
