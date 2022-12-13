/******************************************************************************
 *  File Name:
 *    macros.hpp
 *
 *  Description:
 *    Macros that are shared across the LLD
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_MACROS_HPP
#define THOR_LLD_MACROS_HPP

/*-----------------------------------------------------------------------------
Macros
-----------------------------------------------------------------------------*/
/*-------------------------------------------------
Enables/disables the const data attribute. This lets
the developer place configuration info in either
flash memory or RAM. Usually an embedded system will
have more flash than RAM, so it makes sense to store
it there. For simulators, it's desireable to have it
in RAM for controlling initialization and behavioral
testing (ie simulate if configs are bad).
-------------------------------------------------*/
#if !defined( LLD_CONST )
#if defined( EMBEDDED )
#define LLD_CONST const
#else
#define LLD_CONST
#endif /* EMBEDDED */
#endif /* !LLD_CONST */

/*-----------------------------------------------------------------------------
Function Attributes
-----------------------------------------------------------------------------*/
#define THOR_RAM_FUNC __attribute__ ((long_call, section (".thor_ram_code")))

#endif  /* !THOR_LLD_MACROS_HPP */
