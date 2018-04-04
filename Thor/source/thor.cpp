#include <Thor/include/thor.h>
#include <Thor/include/exti.h>

void ThorInit()
{
	/* Set the clock and peripheral settings to max performance */
	ThorSystemClockConfig();

	/* Set up the EXTI handler for passing messages from
	 * from high priority to low priority interrupts. */
	#ifdef USING_FREERTOS
	setupEXTI0_Interrupt();
	#endif

}