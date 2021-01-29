/********************************************************************************
 *  File Name:
 *    hw_uart_driver.cpp
 *
 *  Description:
 *    Common driver interface implementation
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/assert>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/uart>
#include <Thor/lld/interface/uart/common_driver/uart_common_intf.hpp>

namespace Thor::LLD::UART
{
  /*-------------------------------------------------------------------------------
  Private Functions
  -------------------------------------------------------------------------------*/
  void prjEnableTransmitter( RegisterMap *const periph )
  {
    if( !UE::get( periph ) )
    {
      UE::set( periph, CR1_UE );
    }

    TE::set( periph, CR1_TE );
  }


  void prjDisableTransmitter( RegisterMap *const periph )
  {
    TE::clear( periph, CR1_TE );
  }


  void prjEnableReceiver( RegisterMap *const periph )
  {
    if( !UE::get( periph ) )
    {
      UE::set( periph, CR1_UE );
    }

    RE::set( periph, CR1_RE );
  }


  void prjDisableReceiver( RegisterMap *const periph )
  {
    RE::clear( periph, CR1_RE );
  }


  void prjWriteDataRegister( RegisterMap *const periph, const uint16_t data )
  {
    periph->DR = data & DR_DR_Msk;
  }


  uint16_t prjReadDataRegister( RegisterMap *const periph )
  {
    return periph->DR & DR_DR_Msk;
  }


  void prjEnableISRSignal( RegisterMap *const periph, const ISRSignal signal )
  {
    switch( signal )
    {
      case ISRSignal::PARITY_ERROR:
        PEIE::set( periph, CR1_PEIE );
        break;

      case ISRSignal::TRANSMIT_DATA_REG_EMPTY:
        TXEIE::set( periph, CR1_TXEIE );
        break;

      case ISRSignal::TRANSMIT_COMPLETE:
        TCIE::set( periph, CR1_TCIE );
        break;

      case ISRSignal::RECEIVED_DATA_READY:
        RXNEIE::set( periph, CR1_RXNEIE );
        break;

      case ISRSignal::LINE_IDLE:
        IDLEIE::set( periph, CR1_IDLEIE );
        break;

      default:
        RT_HARD_ASSERT( false );
        break;
    };
  }


  void prjDisableISRSignal( RegisterMap *const periph, const ISRSignal signal )
  {
    switch( signal )
    {
      case ISRSignal::PARITY_ERROR:
        PEIE::clear( periph, CR1_PEIE );
        break;

      case ISRSignal::TRANSMIT_DATA_REG_EMPTY:
        TXEIE::clear( periph, CR1_TXEIE );
        break;

      case ISRSignal::TRANSMIT_COMPLETE:
        TCIE::clear( periph, CR1_TCIE );
        break;

      case ISRSignal::RECEIVED_DATA_READY:
        RXNEIE::clear( periph, CR1_RXNEIE );
        break;

      case ISRSignal::LINE_IDLE:
        IDLEIE::clear( periph, CR1_IDLEIE );
        break;

      default:
        RT_HARD_ASSERT( false );
        break;
    };
  }


  bool prjGetISRSignal( RegisterMap *const periph, const ISRSignal signal )
  {
    switch( signal )
    {
      case ISRSignal::PARITY_ERROR:
        return PE::get( periph );
        break;

      case ISRSignal::TRANSMIT_DATA_REG_EMPTY:
        return TXE::get( periph );
        break;

      case ISRSignal::TRANSMIT_COMPLETE:
        return TC::get( periph );
        break;

      case ISRSignal::RECEIVED_DATA_READY:
        return RXNE::get( periph );
        break;

      case ISRSignal::LINE_IDLE:
        return IDLE::get( periph );
        break;

      default:
        RT_HARD_ASSERT( false );
        return false;
        break;
    };
  }


  void prjSetISRSignal( RegisterMap *const periph, const ISRSignal signal )
  {
    switch( signal )
    {
      case ISRSignal::PARITY_ERROR:
      case ISRSignal::TRANSMIT_DATA_REG_EMPTY:
      case ISRSignal::TRANSMIT_COMPLETE:
      case ISRSignal::RECEIVED_DATA_READY:
      case ISRSignal::LINE_IDLE:
        // Do nothing. These signals cannot be manually set.
        break;

      default:
        RT_HARD_ASSERT( false );
        break;
    };
  }


  void prjClrISRSignal( RegisterMap *const periph, const ISRSignal signal )
  {
    switch( signal )
    {
      case ISRSignal::PARITY_ERROR:
      case ISRSignal::LINE_IDLE:
        // Can only be cleared by reading SR, then read/write access to DR
        return;
        break;

      case ISRSignal::RECEIVED_DATA_READY:
      case ISRSignal::TRANSMIT_DATA_REG_EMPTY:
        // Can only be cleared via read/write to data register
        return;
        break;

      case ISRSignal::TRANSMIT_COMPLETE:
        TC::set( periph, 0 );
        break;

      default:
        RT_HARD_ASSERT( false );
        break;
    };
  }

}    // namespace Thor::LLD::UART
