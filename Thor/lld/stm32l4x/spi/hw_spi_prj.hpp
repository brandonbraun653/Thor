/********************************************************************************
 *  File Name:
 *    hw_spi_prj.hpp
 *
 *  Description:
 *    Pulls in target specific definitions and resources used in the actual driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_SPI_PROJECT_HPP
#define THOR_HW_SPI_PROJECT_HPP

#if defined( STM32L432xx )
#include <Thor/lld/stm32l4x/spi/variant/hw_spi_register_stm32l4xxxx.hpp>
#endif

#endif /* !THOR_HW_SPI_PROJECT_HPP */
