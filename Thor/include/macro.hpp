#pragma once
#ifndef THOR_MACRO_HPP
#define THOR_MACRO_HPP
#include <Thor/include/system.hpp>

#define DISABLE_WRITE_BUFFERING (*SCB_REG_ACTLR |= ACTLR_DISDEFWBUF_Msk)
#define ENABLE_WRITE_BUFFERING	(*SCB_REG_ACTLR &= ~ACTLR_DISDEFWBUF_Msk)

#endif /* !THOR_MACRO_HPP */