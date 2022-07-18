/******************************************************************************
 *  File Name:
 *    lld_timer_output_compare.cpp
 *
 *  Description:
 *    Output compare functionality driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/timer>
#include <Thor/lld/interface/inc/timer>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void useOCPreload( Handle_rPtr timer, const Chimera::Timer::Channel ch, const bool use )
  {
    const uint32_t value = use ? 1 : 0;

    switch ( ch )
    {
      case Chimera::Timer::Channel::CHANNEL_1:
        OC1PE::set( timer->registers, value << CCMR1_OC1PE_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_2:
        OC2PE::set( timer->registers, value << CCMR1_OC2PE_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_3:
        OC3PE::set( timer->registers, value << CCMR2_OC3PE_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_4:
        OC4PE::set( timer->registers, value << CCMR2_OC4PE_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_5:
        OC5PE::set( timer->registers, value << CCMR3_OC5PE_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_6:
        OC6PE::set( timer->registers, value << CCMR3_OC6PE_Pos );
        break;

      default:
        // Do nothing
        break;
    };
  }


  Chimera::Status_t setOCMode( Handle_rPtr timer, const Chimera::Timer::Channel ch, const OCMode mode )
  {
    switch ( ch )
    {
      case Chimera::Timer::Channel::CHANNEL_1:
        OC1M::set( timer->registers, mode << CCMR1_OC1M_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_2:
        OC2M::set( timer->registers, mode << CCMR1_OC2M_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_3:
        OC3M::set( timer->registers, mode << CCMR2_OC3M_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_4:
        OC4M::set( timer->registers, mode << CCMR2_OC4M_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_5:
        OC5M::set( timer->registers, mode << CCMR3_OC5M_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_6:
        OC6M::set( timer->registers, mode << CCMR3_OC6M_Pos );
        break;

      default:
        return Chimera::Status::NOT_SUPPORTED;
    };

    return Chimera::Status::OK;
  }


  OCMode getOCMode( Handle_rPtr timer, const Chimera::Timer::Channel ch )
  {
    switch ( ch )
    {
      case Chimera::Timer::Channel::CHANNEL_1:
        return static_cast<OCMode>( OC1PE::get( timer->registers) >> CCMR1_OC1PE_Pos );

      case Chimera::Timer::Channel::CHANNEL_2:
        return static_cast<OCMode>( OC2PE::get( timer->registers) >> CCMR1_OC2PE_Pos );

      case Chimera::Timer::Channel::CHANNEL_3:
        return static_cast<OCMode>( OC3PE::get( timer->registers) >> CCMR2_OC3PE_Pos );

      case Chimera::Timer::Channel::CHANNEL_4:
        return static_cast<OCMode>( OC4PE::get( timer->registers) >> CCMR2_OC4PE_Pos );

      case Chimera::Timer::Channel::CHANNEL_5:
        return static_cast<OCMode>( OC5PE::get( timer->registers) >> CCMR3_OC5PE_Pos );

      case Chimera::Timer::Channel::CHANNEL_6:
        return static_cast<OCMode>( OC6PE::get( timer->registers) >> CCMR3_OC6PE_Pos );

      default:
        return OC_MODE_INVALID;
    };
  }


  Chimera::Status_t setOCReference( Handle_rPtr timer, const Chimera::Timer::Channel ch, const uint32_t ref )
  {
    switch ( ch )
    {
      case Chimera::Timer::Channel::CHANNEL_1:
        CC1::set( timer->registers, ref );
        break;

      case Chimera::Timer::Channel::CHANNEL_2:
        CC2::set( timer->registers, ref );
        break;

      case Chimera::Timer::Channel::CHANNEL_3:
        CC3::set( timer->registers, ref );
        break;

      case Chimera::Timer::Channel::CHANNEL_4:
        CC4::set( timer->registers, ref );
        break;

      case Chimera::Timer::Channel::CHANNEL_5:
        CC5::set( timer->registers, ref );
        break;

      case Chimera::Timer::Channel::CHANNEL_6:
        CC6::set( timer->registers, ref );
        break;

      default:
        return Chimera::Status::NOT_SUPPORTED;
    };

    return Chimera::Status::OK;
  }


  uint32_t getOCReference( Handle_rPtr timer, const Chimera::Timer::Channel ch )
  {
    switch ( ch )
    {
      case Chimera::Timer::Channel::CHANNEL_1:
        return CC1::get( timer->registers );

      case Chimera::Timer::Channel::CHANNEL_2:
        return CC2::get( timer->registers );

      case Chimera::Timer::Channel::CHANNEL_3:
        return CC3::get( timer->registers );

      case Chimera::Timer::Channel::CHANNEL_4:
        return CC4::get( timer->registers );

      case Chimera::Timer::Channel::CHANNEL_5:
        return CC5::get( timer->registers );

      case Chimera::Timer::Channel::CHANNEL_6:
        return CC6::get( timer->registers );

      default:
        return 0;
    };
  }

}  // namespace Thor::LLD::TIMER
