/********************************************************************************
 *  File Name:
 *    spi_mock_variant.cpp
 *
 *  Description:
 *    Thor SPI mock variant
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Thor Includes */
#include <Thor/lld/interface/spi/mock/spi_mock_variant.hpp>

namespace Thor::LLD::SPI
{
  /*-------------------------------------------------------------------------------
  Variables
  -------------------------------------------------------------------------------*/
  static RegisterMap SPI1_PERIPH;
  static RegisterMap SPI2_PERIPH;
  static RegisterMap SPI3_PERIPH;
  static RegisterMap SPI4_PERIPH;

  Chimera::Container::LightFlatMap<Chimera::SPI::Channel, RegisterMap *> ChannelToInstance{
    { Chimera::SPI::Channel::SPI1, &SPI1_PERIPH },
    { Chimera::SPI::Channel::SPI2, &SPI2_PERIPH },
    { Chimera::SPI::Channel::SPI3, &SPI3_PERIPH },
    { Chimera::SPI::Channel::SPI4, &SPI4_PERIPH }
  };


  Thor::LLD::RIndexMap InstanceToResourceIndex{
    { reinterpret_cast<std::uintptr_t>( &SPI1_PERIPH ), SPI1_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( &SPI2_PERIPH ), SPI2_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( &SPI3_PERIPH ), SPI3_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( &SPI4_PERIPH ), SPI4_RESOURCE_INDEX }
  };

  std::array<RegisterMap*, NUM_SPI_PERIPHS> PeripheralRegisterMaps;


  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  const std::array<Chimera::SPI::Channel, NUM_SPI_PERIPHS> supportedChannels = {
#if defined ( STM32_SPI1_PERIPH_AVAILABLE )
    Chimera::SPI::Channel::SPI1,
#else
    Chimera::SPI::Channel::NOT_SUPPORTED,
#endif

#if defined ( STM32_SPI2_PERIPH_AVAILABLE )
    Chimera::SPI::Channel::SPI2,
#else
    Chimera::SPI::Channel::NOT_SUPPORTED,
#endif

#if defined ( STM32_SPI3_PERIPH_AVAILABLE )
    Chimera::SPI::Channel::SPI3,
#else
    Chimera::SPI::Channel::NOT_SUPPORTED,
#endif

#if defined ( STM32_SPI4_PERIPH_AVAILABLE )
    Chimera::SPI::Channel::SPI4,
#else
    Chimera::SPI::Channel::NOT_SUPPORTED,
#endif
  };

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  void initializeRegisters()
  {
    for( size_t x=0; x<PeripheralRegisterMaps.size(); x++)
    {
      if( PeripheralRegisterMaps[x])
      {
        delete PeripheralRegisterMaps[ x ];
      }

      PeripheralRegisterMaps[x] = new RegisterMap();
    }
  }


  void initializeMapping()
  {

  }
}  // namespace Thor::LLD::SPI
