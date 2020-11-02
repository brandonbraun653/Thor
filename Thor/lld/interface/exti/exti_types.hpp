/********************************************************************************
 *  File Name:
 *    exti_types.hpp
 *
 *  Description:
 *    Common LLD EXTI Types
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_LLD_EXTI_DRIVER_TYPES_HPP
#define THOR_LLD_EXTI_DRIVER_TYPES_HPP

/* STL Includes */
#include <cstdint>

namespace Thor::LLD::EXTI
{
  /*-------------------------------------------------------------------------------
  Foward Declarations
  -------------------------------------------------------------------------------*/
  struct RegisterMap;


  /*-------------------------------------------------------------------------------
  Enumerations
  -------------------------------------------------------------------------------*/
  enum class LineType : uint8_t
  {
    INTERRUPT,
    EVENT,
    WAKEUP,

    NUM_OPTIONS,
    UNKNOWN
  };


  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct LineConfig
  {
    LineConfig() : supported( false ), NVIC_IRQNumber( -100 ), lineType( LineType::UNKNOWN )
    {
    }

    LineConfig( bool s, int n, LineType l )
    {
      supported      = s;
      NVIC_IRQNumber = n;
      lineType       = l;
    }

    bool supported;
    int NVIC_IRQNumber;
    LineType lineType;

  };

}    // namespace Thor::LLD::EXTI

#endif /* !THOR_LLD_EXTI_DRIVER_TYPES_HPP */
