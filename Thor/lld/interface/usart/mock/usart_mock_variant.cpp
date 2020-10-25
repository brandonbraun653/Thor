/********************************************************************************
 *  File Name:
 *    usart_mock_variant.cpp
 *
 *  Description:
 *    Thor USART mock variant
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/usart/usart_types.hpp>
#include <Thor/lld/interface/usart/mock/usart_mock_variant.hpp>

#if defined( THOR_LLD_USART_MOCK )

namespace Thor::LLD::USART
{
  /*-------------------------------------------------------------------------------
  Variables
  -------------------------------------------------------------------------------*/
  static RegisterMap USART1_PERIPH;
  static RegisterMap USART2_PERIPH;
  static RegisterMap USART3_PERIPH;
  static RegisterMap USART4_PERIPH;

  // Chimera::Container::LightFlatMap<Chimera::Serial::Channel, RegisterMap *> ChannelToInstance{
  //   { Chimera::Serial::Channel::SERIAL1, &USART1_PERIPH },
  //   { Chimera::Serial::Channel::SERIAL2, &USART2_PERIPH },
  //   { Chimera::Serial::Channel::SERIAL3, &USART3_PERIPH },
  //   { Chimera::Serial::Channel::SERIAL4, &USART4_PERIPH }
  // };


  // Thor::LLD::RIndexMap InstanceToResourceIndex{
  //   { reinterpret_cast<std::uintptr_t>( &USART1_PERIPH ), USART1_RESOURCE_INDEX },
  //   { reinterpret_cast<std::uintptr_t>( &USART2_PERIPH ), USART2_RESOURCE_INDEX },
  //   { reinterpret_cast<std::uintptr_t>( &USART3_PERIPH ), USART3_RESOURCE_INDEX },
  //   { reinterpret_cast<std::uintptr_t>( &USART4_PERIPH ), USART4_RESOURCE_INDEX }
  // };

  // std::array<RegisterMap*, NUM_USART_PERIPHS> PeripheralRegisterMaps;


  // /* clang-format off */
  // std::array<uint32_t, static_cast<size_t>( Chimera::Serial::CharWid::NUM_OPTIONS )> CharWidToRegConfig = {
  //   Configuration::WordLength::LEN_8BIT,
  //   Configuration::WordLength::LEN_9BIT
  // };

  // std::array<uint32_t, static_cast<size_t>( Chimera::Serial::Parity::NUM_OPTIONS )> ParityToRegConfig = {
  //   Configuration::Parity::NONE,
  //   Configuration::Parity::EVEN,
  //   Configuration::Parity::ODD
  // };

  // std::array<uint32_t, static_cast<size_t>( Chimera::Serial::StopBits::NUM_OPTIONS )> StopBitsToRegConfig = {
  //   Configuration::Stop::BIT_1,
  //   Configuration::Stop::BIT_1_5,
  //   Configuration::Stop::BIT_2
  // };
  // /* clang-format on */

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  void initializeRegisters()
  {
    // for( size_t x=0; x<PeripheralRegisterMaps.size(); x++)
    // {
    //   if( PeripheralRegisterMaps[x])
    //   {
    //     delete PeripheralRegisterMaps[ x ];
    //   }

    //   PeripheralRegisterMaps[x] = new RegisterMap();
    // }
  }


  void initializeMapping()
  {

  }
}  // namespace Thor::LLD::USART

#endif /* THOR_LLD_USART_MOCK */
