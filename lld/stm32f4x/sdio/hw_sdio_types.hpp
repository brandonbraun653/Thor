/******************************************************************************
 *  File Name:
 *    hw_sdio_types.hpp
 *
 *  Description:
 *    Type descriptors for SDIO
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_SDIO_TYPES_HPP
#define THOR_HW_SDIO_TYPES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>

namespace Thor::LLD::SDIO
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct RegisterMap
  {
    volatile uint32_t       POWER;           /**< SDIO power control register,    Address offset: 0x00 */
    volatile uint32_t       CLKCR;           /**< SDI clock control register,     Address offset: 0x04 */
    volatile uint32_t       ARG;             /**< SDIO argument register,         Address offset: 0x08 */
    volatile uint32_t       CMD;             /**< SDIO command register,          Address offset: 0x0C */
    volatile const uint32_t RESPCMD;         /**< SDIO command response register, Address offset: 0x10 */
    volatile const uint32_t RESP1;           /**< SDIO response 1 register,       Address offset: 0x14 */
    volatile const uint32_t RESP2;           /**< SDIO response 2 register,       Address offset: 0x18 */
    volatile const uint32_t RESP3;           /**< SDIO response 3 register,       Address offset: 0x1C */
    volatile const uint32_t RESP4;           /**< SDIO response 4 register,       Address offset: 0x20 */
    volatile uint32_t       DTIMER;          /**< SDIO data timer register,       Address offset: 0x24 */
    volatile uint32_t       DLEN;            /**< SDIO data length register,      Address offset: 0x28 */
    volatile uint32_t       DCTRL;           /**< SDIO data control register,     Address offset: 0x2C */
    volatile const uint32_t DCOUNT;          /**< SDIO data counter register,     Address offset: 0x30 */
    volatile const uint32_t STA;             /**< SDIO status register,           Address offset: 0x34 */
    volatile uint32_t       ICR;             /**< SDIO interrupt clear register,  Address offset: 0x38 */
    volatile uint32_t       MASK;            /**< SDIO mask register,             Address offset: 0x3C */
    uint32_t                RESERVED0[ 2 ];  /**< Reserved, 0x40-0x44                                  */
    volatile const uint32_t FIFOCNT;         /**< SDIO FIFO counter register,     Address offset: 0x48 */
    uint32_t                RESERVED1[ 13 ]; /**< Reserved, 0x4C-0x7C                                  */
    volatile uint32_t       FIFO;            /**< SDIO data FIFO register,        Address offset: 0x80 */
  };
}    // namespace Thor::LLD::SDIO

#endif /* !THOR_HW_SDIO_TYPES_HPP */
