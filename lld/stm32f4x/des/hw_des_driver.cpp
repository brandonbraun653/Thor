/********************************************************************************
 *  File Name:
 *    hw_des_driver.cpp
 *
 *  Description:
 *    DES driver implementation
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <cstring>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/system>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/des/des_types.hpp>
#include <Thor/lld/stm32f4x/des/hw_des_mapping.hpp>
#include <Thor/lld/stm32f4x/des/hw_des_prj.hpp>
#include <Thor/lld/stm32f4x/des/hw_des_types.hpp>


#if defined( TARGET_STM32F4 ) && defined( THOR_DES ) && defined( STM32F446xx )

namespace Thor::LLD::DES
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  void initialize()
  {
    // Nothing to do
  }


  void getUniqueId( UniqueID &id )
  {
    Reg32_t uidBytes1 = UIDField1::get( UDIDR );
    Reg32_t uidBytes2 = UIDField2::get( UDIDR );
    Reg32_t uidBytes3 = UIDField3::get( UDIDR );

    id.fill( 0 );
    memcpy( &id[ 0 ], &uidBytes1, sizeof( Reg32_t ) );
    memcpy( &id[ 4 ], &uidBytes2, sizeof( Reg32_t ) );
    memcpy( &id[ 8 ], &uidBytes3, sizeof( Reg32_t ) );
  }


  size_t getFlashSize()
  {
    return static_cast<size_t>( FS::get( FSDR ) );
  }


  Chimera::System::Packaging getICPackaging()
  {
    Reg32_t pkg = PD::get( PDR );

    switch ( pkg )
    {
      case PDR_LQFP64:
        return Chimera::System::Packaging::LQFP64;
        break;

      case PDR_LQFP100:
        return Chimera::System::Packaging::LQFP100;
        break;

      case PDR_WLCSP81:
        return Chimera::System::Packaging::WLCSP36;
        break;

      case PDR_LQFP_UFBGA_144:
        return Chimera::System::Packaging::LQFP144;
        break;

      default:
        return Chimera::System::Packaging::UNKNOWN;
    }
  }
}    // namespace Thor::LLD::DES

#endif /* TARGET_STM32F4 && THOR_LLD_DES && STM32F446xx */
