/********************************************************************************
 *  File Name:
 *    gpio_mock_variant.cpp
 *
 *  Description:
 *    Thor GPIO mock variant
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/gpio/mock/gpio_mock_variant.hpp>

#if defined( THOR_LLD_GPIO_MOCK )

namespace Thor::LLD::GPIO
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/

  {
  }


  void initializeMapping()
  {

  }
}  // namespace Thor::LLD::GPIO

#endif /* THOR_LLD_GPIO_MOCK */
