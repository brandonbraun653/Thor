#ifndef THOR_H_
#define THOR_H_

#include <Thor/include/config.hpp>
#include <Thor/include/core.hpp>
#include <Thor/include/definitions.hpp>

extern void ThorInit();

/** @namespace Thor */
namespace Thor
{
	void delayMilliseconds(uint32_t ms);
	void delayMicroseconds(uint32_t us);
}

#endif /* THOR_H_ */
