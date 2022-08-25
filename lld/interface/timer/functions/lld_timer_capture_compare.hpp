/******************************************************************************
 *  File Name:
 *    lld_timer_capture_compare.hpp
 *
 *  Description:
 *    Capture compare aspects of a timer driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_CAPTURE_COMPARE_HPP
#define THOR_LLD_TIMER_CAPTURE_COMPARE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/gpio>
#include <Chimera/timer>
#include <Thor/lld/interface/timer/timer_types.hpp>
#include <cstdint>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum CCMode : uint32_t
  {
    CCM_OUTPUT,
    CCM_INPUT_MAP_TI2,
    CCM_INPUT_MAP_TI1,
    CCM_INPUT_MAP_TRC,
    CCM_INVALID
  };

  enum CCPolarity : uint32_t
  {
    CCP_OUT_ACTIVE_HIGH,
    CCP_OUT_ACTIVE_LOW,
    CCP_IN_RISING_EDGE,
    CCP_IN_FALLING_EDGE,
    CCP_IN_BOTH_EDGE,
    CCP_INVALID
  };

  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  /* clang-format off */
  static constexpr std::array<uint32_t, EnumValue( Chimera::Timer::Output::NUM_OPTIONS )> EnableFlags = {
    CCER_CC1E,  /* OUTPUT_1P */
    CCER_CC1NE, /* OUTPUT_1N */
    CCER_CC2E,
    CCER_CC2NE,
    CCER_CC3E,
    CCER_CC3NE,
    CCER_CC4E,
    0,          /* OUTPUT_4N */
    CCER_CC5E,
    0,          /* OUTPUT_5N */
    CCER_CC6E,
    0           /* OUTPUT_6N */
  };

  static constexpr std::array<uint32_t, EnumValue( Chimera::Timer::Output::NUM_OPTIONS )> PolarityFlags = {
    CCER_CC1P,  /* OUTPUT_1P */
    CCER_CC1NP, /* OUTPUT_1N */
    CCER_CC2P,
    CCER_CC2NP,
    CCER_CC3P,
    CCER_CC3NP,
    CCER_CC4P,
    CCER_CC4NP,
    CCER_CC5P,
    0,          /* OUTPUT_5N */
    CCER_CC6P,
    0           /* OUTPUT_6N */
  };

  static constexpr std::array<uint32_t, EnumValue( Chimera::Timer::Output::NUM_OPTIONS )> IdleFlags = {
    CR2_OIS1,   /* OUTPUT_1P */
    CR2_OIS1N,  /* OUTPUT_1N */
    CR2_OIS2,
    CR2_OIS2N,
    CR2_OIS3,
    CR2_OIS3N,
    CR2_OIS4,
    0,          /* OUTPUT_4N */
    CR2_OIS5,
    0,          /* OUTPUT_5N */
    CR2_OIS6,
    0           /* OUTPUT_6N */
  };
  /* clang-format on */

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  template<typename... Args>
  static constexpr uint32_t EnableFlagGenerator( Args&&... args )
  {
    //return ( EnableFlags[ EnumValue( args ) ], | ... );
    return 0;
  }


  static inline void enableCCPreload( Handle_rPtr timer )
  {
    CCPC::set( timer->registers, CR2_CCPC );
  }

  static inline void disableCCPreload( Handle_rPtr timer )
  {
    CCPC::clear( timer->registers, CR2_CCPC );
  }

  /**
   * @brief Sets the IO idling logic state
   *
   * @param timer   Which timer to act on
   * @param ch      Which output channel to configure
   * @param state   Desired idling state
   */
  void setOutputIdleState( Handle_rPtr timer, const Chimera::Timer::Output ch, const Chimera::GPIO::State state );

  /**
   * @brief Sets the idle state of multiple channels at once
   *
   * @param timer   Which timer to act on
   * @param bf      Bit field of timer output channels
   * @param state   Desired idling state
   */
  void setOutputIdleStateBulk( Handle_rPtr timer, const uint32_t bf, const Chimera::GPIO::State state );

  /**
   * @brief Disables a capture compare channel output
   *
   * @param timer   Which timer to act on
   * @param ch      Which output channel to disable
   */
  void disableCCOutput( Handle_rPtr timer, const Chimera::Timer::Output ch );

  /**
   * @brief Disables capture compare channel outputs in bulk
   *
   * @param timer   Which timer to act on
   * @param bf      Bitfield of the channels to act on
   */
  void disableCCOutputBulk( Handle_rPtr timer, const uint32_t bf );

  /**
   * @brief Enables a capture compare channel output
   *
   * @param timer   Which timer to act on
   * @param ch      Which output channel to enable
   */
  void enableCCOutput( Handle_rPtr timer, const Chimera::Timer::Output ch );

  /**
   * @brief Enables capture compare channel outputs in bulk
   *
   * @param timer   Which timer to act on
   * @param bf      Bitfield of the channels to act on
   */
  void enableCCOutputBulk( Handle_rPtr timer, const uint32_t bf );

  /**
   * @brief Sets the capture/compare polarity of a channel output
   *
   * @param timer   Which timer to act on
   * @param ch      The output channel to act on
   * @param pol     What polarity is being set
   */
  void setCCOutputPolarity( Handle_rPtr timer, const Chimera::Timer::Output ch, const CCPolarity pol );

  /**
   * @brief Sets the capture/compare polarity of a multiple channel outputs at once
   *
   * @param timer   Which timer to act on
   * @param bf      A bitfield of the output channels to act on
   * @param pol     What polarity is being set
   */
  void setCCOutputPolarityBulk( Handle_rPtr timer, const uint32_t bf, const CCPolarity pol );

  /**
   * @brief Sets the capture/compare mode behavior for a timer channel
   *
   * @param timer   Which timer to act on
   * @param ch      The channel to act on
   * @param mode    Mode being set
   * @return Chimera::Status_t
   */
  Chimera::Status_t setCCMode( Handle_rPtr timer, const Chimera::Timer::Channel ch, const CCMode mode );

  /**
   * @brief Gets the capture/compare mode set on the channel
   *
   * @param timer   Which timer to act on
   * @param ch      Which channel to inspect
   * @return CCMode
   */
  CCMode getCCMode( Handle_rPtr timer, const Chimera::Timer::Channel ch );

}  // namespace Thor::LLD::TIMER

#endif  /* !THOR_LLD_TIMER_CAPTURE_COMPARE_HPP */
