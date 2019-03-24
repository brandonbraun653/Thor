#ifndef PRINT_H_
#define PRINT_H_

#include <stdlib.h>
#include <cmath>
#include <string>

#include <Thor/config.hpp>


extern std::string float2String( float number );

#if USE_SERIAL_DEBUG_OUTPUT && !defined( USING_VISUALGDB_PROFILER )
extern void setupSTDIO();
#endif


#endif /* !PRINT_H_ */
