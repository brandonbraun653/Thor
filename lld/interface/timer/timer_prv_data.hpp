/********************************************************************************
 *  File Name:
 *    timer_prv_data.hpp
 *
 *  Description:
 *    Declaration of data that must be defined by the LLD implementation or is
 *    shared among all possible drivers.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_DATA_HPP
#define THOR_LLD_TIMER_DATA_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/common/macros.hpp>
#include <Thor/lld/common/types.hpp>

namespace Thor::LLD::TIMER
{
  /*-------------------------------------------------------------------------------
  Peripheral Instances:
    Memory mapped structs that allow direct access to the registers of a peripheral
  -------------------------------------------------------------------------------*/



  /*-------------------------------------------------------------------------------
  Configuration Maps:
    These convert high level configuration options into low level register config
    options. The idea is to allow the user to specify some general options, then
    convert that over to what the peripheral understands during config/init steps.
  -------------------------------------------------------------------------------*/
  namespace ConfigMap
  {
  }    // namespace ConfigMap


  /*-------------------------------------------------------------------------------
  Peripheral Resources:
    These objects define critical resources used in the low level driver. The goal
    is to minimize memory consumption, so these arrays only hold enough information
    for the currently configured number of peripherals. They are intended to be
    accessed directly via the _ResourceIndex_ attribute of the ConfigMap namespace.
  -------------------------------------------------------------------------------*/
  namespace Resource
  {
  }
}    // namespace Thor::LLD::TIMER

#endif /* !THOR_LLD_TIMER_DATA_HPP */
