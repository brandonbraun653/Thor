/********************************************************************************
 *  File Name:
 *    hld_usart_chimera.hpp
 *
 *	 Description:
 *    Chimera hooks for implementing USART
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_USART_CHIMERA_HOOKS_HPP
#define THOR_USART_CHIMERA_HOOKS_HPP

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/usart>

namespace Chimera::USART::Backend
{
  Chimera::Status_t initialize();

  Chimera::Status_t reset();

  Chimera::USART::USART_sPtr create_shared_ptr();

  Chimera::USART::USART_uPtr create_unique_ptr();
}    // namespace Chimera::USART::Backend

#endif /* !THOR_USART_CHIMERA_HOOKS_HPP */
