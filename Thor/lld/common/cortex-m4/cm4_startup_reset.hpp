/********************************************************************************
 *  File Name:
 *    cm4_startup_reset.hpp
 *
 *  Description:
 *    Export the startup vector table so the linker can't throw it away
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( __cplusplus )
extern "C"
{
#endif

extern void *StartupVectorTable;
#define THOR_SYSTEM_ISR_VECTOR_SYMBOL( StartupVectorTable )

#if defined( __cplusplus )
}
#endif
