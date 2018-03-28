#include "../include/exceptions.h"

void BasicErrorHandler(std::string err_msg)
{
	/* If you got here, look at the message and trace back to the root problem */
	volatile std::string message = err_msg;
	for (;;)
		;
}


void HardFault_Handler()
{
	/* Well you broke SOMETHING Jim Bob */
	
	while (1)
	{
	}
}