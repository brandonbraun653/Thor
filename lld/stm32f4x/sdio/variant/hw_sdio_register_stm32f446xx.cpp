/******************************************************************************
 *  File Name:
 *    hw_sdio_register_stm32f446xx.cpp
 *
 *  Description:
 *    Chip specific data for the SDIO peripheral on the STM32F446xx chips
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/lld/interface/sdio/sdio_detail.hpp>

namespace Thor::LLD::SDIO
{
	/*---------------------------------------------------------------------------
	Peripheral Instances
	---------------------------------------------------------------------------*/
#if defined( STM32_SDIO1_PERIPH_AVAILABLE )
	RegisterMap *SDIO1_PERIPH = reinterpret_cast<RegisterMap *>( SDIO1_BASE_ADDR );
#endif
}  // namespace Thor::LLD::SDIO
