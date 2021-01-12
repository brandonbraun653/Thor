/********************************************************************************
 *  File Name:
 *    hld_clock_driver.hpp
 *
 *  Description:
 *    Interface to the HLD clock driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HLD_CLOCK_HPP
#define THOR_HLD_CLOCK_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/clock>

namespace Thor::Clock
{
  /**
   *  @copydoc Chimera::Clock::initialize()
   */
  Chimera::Status_t initialize();

  /**
   *  @copydoc Chimera::Clock::periphEnable( const Chimera::Peripheral::Type periph )
   */
  Chimera::Status_t periphEnable( const Chimera::Peripheral::Type periph );

  /**
   *  @copydoc Chimera::Clock::periphDisable( const Chimera::Peripheral::Type periph )
   */
  Chimera::Status_t periphDisable( const Chimera::Peripheral::Type periph );

  /**
   *  @copydoc Chimera::Clock::enableClock( const Chimera::Clock::Bus bus )
   */
  Chimera::Status_t enableClock( const Chimera::Clock::Bus bus );

  /**
   *  @copydoc Chimera::Clock::disableClock( const Chimera::Clock::Bus bus )
   */
  Chimera::Status_t disableClock( const Chimera::Clock::Bus bus );

  /**
   *  @copydoc isEnabled( const Chimera::Clock::Bus bus )
   */
  bool isEnabled( const Chimera::Clock::Bus bus );

  /**
   *  @copydoc Chimera::Clock::getFrequency( const Chimera::Clock::Bus bus )
   */
  size_t getFrequency( const Chimera::Clock::Bus bus );

  /**
   *  @copydoc Chimera::Clock::setFrequency( const Chimera::Clock::Bus bus, const size_t freq )
   */
  Chimera::Status_t setFrequency( const Chimera::Clock::Bus bus, const size_t freq );
}    // namespace Thor::HLD::Clock

#endif /* !THOR_HLD_CLOCK_HPP */
