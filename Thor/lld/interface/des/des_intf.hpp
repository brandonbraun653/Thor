/********************************************************************************
 *  File Name:
 *    des_intf.hpp
 *
 *  Description:
 *    Public interface for the Device Electronic Signature module
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_DES_INTERFACE_HPP
#define THOR_LLD_DES_INTERFACE_HPP

/* Chimera Includes */
#include <Chimera/system>

/* Thor Includes */
#include <Thor/lld/interface/des/des_types.hpp>

namespace Thor::LLD::DES
{
  /**
   *  Initializes the DES low level driver
   *
   *  @return void
   */
  void initialize();

  /**
   *  Gets the unique identifier programmed into the chip at
   *  the ST factory.
   *
   *  @param[in]  id    The id data to be updated
   *  @return void
   */
  void getUniqueId( UniqueID &id );

  /**
   *  Gets the flash size in kB as reported by the chip
   *
   *  @return size_t
   */
  size_t getFlashSize();

  /**
   *  Gets the physical IC packaging style used
   *
   *  @return Chimera::System::Packaging
   */
  Chimera::System::Packaging getICPackaging();

}    // namespace Thor::LLD::DES

#endif /* !THOR_LLD_DES_INTERFACE_HPP */
