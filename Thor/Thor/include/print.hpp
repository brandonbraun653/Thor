#ifndef PRINT_H_
#define PRINT_H_

#include <stdlib.h>
#include <cmath>
#include <string>

#include <Thor/include/config.hpp>


extern std::string float2String(float number);

#if USE_SERIAL_DEBUG_OUTPUT
extern void setupSTDIO();
#endif


#endif /* !PRINT_H_ */