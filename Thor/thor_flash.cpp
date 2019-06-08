/********************************************************************************
 * File Name:
 *	  thor_flash.cpp
 *
 * Description:
 *	  Interface to the internal flash of the Thor MCU
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <cstring>

/* Thor Includes */
#include <Thor/flash.hpp>


namespace Thor
{
  namespace Memory
  {
    InternalFlash::InternalFlash()
    {
      memset( &eraseInit, 0u, sizeof( FLASH_EraseInitTypeDef ) );
    }

    InternalFlash::~InternalFlash()
    {
    }

    Chimera::Status_t InternalFlash::write( const size_t address, const uint8_t *const data, const size_t length )
    {
      Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

      if ( !data )
      {
        error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
      }
      else if ( HAL_FLASH_Unlock() != HAL_OK )
      {
        error = Chimera::CommonStatusCodes::LOCKED;
      }
      else
      {
        /*------------------------------------------------
        Initialize the write operation 
        TODO: (for now fix at byte access)
        ------------------------------------------------*/
        uint32_t writeAddress = address;
        uint64_t writeData    = 0u;
        uint32_t access       = FLASH_TYPEPROGRAM_BYTE;
        uint32_t pgmWidth     = getPgmByteWidth( access );
        uint32_t bytesLeft    = length;
        uint32_t dataOffset   = 0u;

        do
        {
          /*------------------------------------------------
          Pull out the next piece of data
          TODO: (will need to adjust for access level later)
          ------------------------------------------------*/
          memcpy( &writeData, ( data + dataOffset ), pgmWidth );

          /*------------------------------------------------
          Do the actual write to flash
          ------------------------------------------------*/
          if ( HAL_FLASH_Program(access, writeAddress, writeData) != HAL_OK ) 
          {
            error = convertHALError( HAL_FLASH_GetError() );
            break;
          }
          
          /*------------------------------------------------
          Update the programming iterators 
          ------------------------------------------------*/
          writeAddress  += pgmWidth;
          dataOffset    += pgmWidth;
          bytesLeft     -= pgmWidth;

        } while ( bytesLeft > 0 );

        HAL_FLASH_Lock();
      }

      return error;
    }

    Chimera::Status_t InternalFlash::read( const size_t address, uint8_t *const data, const size_t length )
    {
      Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

      if ( !data )
      {
        error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
      }
      else
      {
        // NOTE: Depending on how large the read is & how well it's aligned, DMA may actually be faster...
        // TODO: Detect alignment to help speed up larger reads
        volatile uint8_t *flash_address = reinterpret_cast<uint8_t *>( address );
        
        for ( uint32_t x = 0; x < length; x++ ) 
        {
          *( data + x ) = *( flash_address + x );
        }
      }

      return error;
    }

    Chimera::Status_t InternalFlash::erase( const size_t address, const size_t length )
    {
      /** 
       *  This function deviates from the Chimera spec in that the hardware makes it unwieldly 
       *  to try and perform such fine-grained erasing operations. Most of the flash banks inside 
       *  the STM32 MCUs are not equally sized small chunks like is typically the case with 
       *  external flash.
       *  
       *  For example, on the STM32F446xx, there are 4 banks of 16k, 1 bank of 64k, and 3 
       *  banks of 128k. Each of those banks is considered a "sector". Erasing a partial
       *  range inside a larger sector would require copying the entire sector to SRAM, erasing
       *  the physical sector, erasing the copy in the specified range, then writing it all back. 
       *  There simply is not enough time/memory to do that efficiently. 
       * 
       *  Instead, this function will erase the sectors that the address range touches
       *  and leave the consequences of that up to the programmer...
       */
      Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

      if ( HAL_FLASH_Unlock() != HAL_OK )
      {
        error = Chimera::CommonStatusCodes::BUSY;
      }
      else
      {
        uint32_t sectorError = 0u;

        uint32_t startSector = addressToSector( address );
        uint32_t endSector   = addressToSector( ( address + length ) );

        eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
        eraseInit.Sector    = startSector;
        eraseInit.NbSectors = ( endSector - startSector ) + 1u;
        eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;


        if ( HAL_FLASHEx_Erase( &eraseInit, &sectorError ) != HAL_OK ) 
        {
          error = Chimera::CommonStatusCodes::FAIL;
        }

        HAL_FLASH_Lock();
      }

      return error;
    }

    Chimera::Status_t InternalFlash::writeCompleteCallback( const Chimera::Function::void_func_uint32_t func )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    Chimera::Status_t InternalFlash::readCompleteCallback( const Chimera::Function::void_func_uint32_t func )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    Chimera::Status_t InternalFlash::eraseCompleteCallback( const Chimera::Function::void_func_uint32_t func )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    bool InternalFlash::isInitialized()
    {
      /* We are always initialized as there is no setup sequence */
      return true;
    }

    Chimera::Status_t InternalFlash::convertHALError( const uint32_t error )
    {
      Chimera::Status_t convertedError = Chimera::CommonStatusCodes::OK;

      switch ( error )
      {
        case HAL_FLASH_ERROR_RD:
          convertedError = Chimera::Modules::Memory::Status::ERR_READ_PROTECT;
          break;
          
#if defined( STM32F4 )
        case HAL_FLASH_ERROR_PGS:
          convertedError = Chimera::Modules::Memory::Status::ERR_PGM_SEQUENCE;
          break;
#endif

        case HAL_FLASH_ERROR_PGP:
          convertedError = Chimera::Modules::Memory::Status::ERR_PGM_PARALLEL;
          break;
          
        case HAL_FLASH_ERROR_PGA:
          convertedError = Chimera::Modules::Memory::Status::ERR_PGM_ALIGNMENT;
          break;

        case HAL_FLASH_ERROR_WRP:
          convertedError = Chimera::Modules::Memory::Status::ERR_WRITE_PROTECT;
          break;

        case HAL_FLASH_ERROR_OPERATION:
          convertedError = Chimera::CommonStatusCodes::FAIL;
          break;
          
        case HAL_FLASH_ERROR_NONE:
          convertedError = Chimera::CommonStatusCodes::OK;
          break;
          
        default:
          convertedError = Chimera::CommonStatusCodes::UNKNOWN_ERROR;
      }

      return convertedError;
    }

    uint32_t InternalFlash::getPgmByteWidth( const uint32_t TypeProgram )
    {
      /*------------------------------------------------
      Depends upon the FLASH_TYPEPROGRAM_XXX #def linearly associating
      BYTE        = 0u
      HALFWORD    = 1u
      WORD        = 2u
      DOUBLEWORD  = 3u

      This shouldn't change, but double check stm32fXxx_hal_flash.h to be sure.
      ------------------------------------------------*/
      return ( 1u << TypeProgram );
    }

    uint32_t InternalFlash::addressToSector( const uint32_t address )
    {
      uint32_t sector = 0u;
      
#if defined( FLASH_MEMORY_MAPPING_INITIALIZED )
      
      for ( uint8_t x = 0; x < sectorStartBoundary.size(); x++ )
      {
        if ( address >= sectorStartBoundary[ x ] )
        {
          sector = x;
        }
        else
        {
          break;
        }
      }
#else
#warning Unable to compute addressToSector for current device
#endif

      return sector;
    }
  }
}
