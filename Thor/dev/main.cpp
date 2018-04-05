#include <stm32f7xx_hal.h>
#include <stm32_hal_legacy.h>

#include <Chimera/spi.hpp>


int main(void)
{
	HAL_Init();

	Chimera::SPIClass test_spi;

	Chimera::SPI::Setup setup;

	test_spi.init(2, setup);

	for (;;)
	{
		
	}
}
