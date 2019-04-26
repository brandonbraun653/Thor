/********************************************************************************
 *   File Name:
 *    dma_definitions.hpp
 *
 *   Description:
 *    Thor DMA Definitions
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DMA_DEFS_HPP
#define THOR_DMA_DEFS_HPP

/* C++ Includes */
#include <cstdint>

/* Thor Includes */
#include <Thor/headers.hpp>

namespace Thor::DMA
{
/* Useful Macros for Generating DMA Register Addresses*/
#define DMA_OFFSET_LISR 0x00U
#define DMA_OFFSET_HISR 0x04U
#define DMA_OFFSET_LIFCR 0x08U
#define DMA_OFFSET_HIFCR 0x0CU
#define DMA_OFFSET_SxCR( STREAM_NUMBER ) ( 0x10U + 0x18U * ( uint32_t )STREAM_NUMBER )
#define DMA_OFFSET_SxNDTR( STREAM_NUMBER ) ( 0x14U + 0x18U * ( uint32_t )STREAM_NUMBER )
#define DMA_OFFSET_SxPAR( STREAM_NUMBER ) ( 0x18U + 0x18U * ( uint32_t )STREAM_NUMBER )
#define DMA_OFFSET_SxM0AR( STREAM_NUMBER ) ( 0x1CU + 0x18U * ( uint32_t )STREAM_NUMBER )
#define DMA_OFFSET_SxM1AR( STREAM_NUMBER ) ( 0x20U + 0x18U * ( uint32_t )STREAM_NUMBER )

// The data sheet has conflicting address definitions for this register. The register map
// does not match manual calculations using the equation below...unsure which is right
#define DMA_OFFSET_SxFCR( STREAM_NUMBER ) ( 0x24U + 0x24U * ( uint32_t )STREAM_NUMBER )

/*------------------------------------------------
DMA1 Register Address Accessors
------------------------------------------------*/
#define DMA1_LISR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_LISR ) )
#define DMA1_HISR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_HISR ) )
#define DMA1_LIFCR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_LIFCR ) )
#define DMA1_HIFCR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_HIFCR ) )
#define DMA1_S0CR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 0 ) ) )
#define DMA1_S1CR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 1 ) ) ) /* DMA1_SxCR */
#define DMA1_S2CR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 2 ) ) )
#define DMA1_S3CR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 3 ) ) )
#define DMA1_S4CR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 4 ) ) )
#define DMA1_S5CR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 5 ) ) )
#define DMA1_S6CR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 6 ) ) )
#define DMA1_S7CR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 7 ) ) )
#define DMA1_S0NDTR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 0 ) ) )
#define DMA1_S1NDTR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 1 ) ) ) /* DMA1_SxNDTR */
#define DMA1_S2NDTR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 2 ) ) )
#define DMA1_S3NDTR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 3 ) ) )
#define DMA1_S4NDTR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 4 ) ) )
#define DMA1_S5NDTR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 5 ) ) )
#define DMA1_S6NDTR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 6 ) ) )
#define DMA1_S7NDTR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 7 ) ) )
#define DMA1_S0PAR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 0 ) ) )
#define DMA1_S1PAR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 1 ) ) ) /* DMA1_SxPAR */
#define DMA1_S2PAR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 2 ) ) )
#define DMA1_S3PAR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 3 ) ) )
#define DMA1_S4PAR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 4 ) ) )
#define DMA1_S5PAR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 5 ) ) )
#define DMA1_S6PAR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 6 ) ) )
#define DMA1_S7PAR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 7 ) ) )
#define DMA1_S0M0AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 0 ) ) )
#define DMA1_S1M0AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 1 ) ) ) /* DMA1_SxM0AR */
#define DMA1_S2M0AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 2 ) ) )
#define DMA1_S3M0AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 3 ) ) )
#define DMA1_S4M0AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 4 ) ) )
#define DMA1_S5M0AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 5 ) ) )
#define DMA1_S6M0AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 6 ) ) )
#define DMA1_S7M0AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 7 ) ) )
#define DMA1_S0M1AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 0 ) ) )
#define DMA1_S1M1AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 1 ) ) ) /* DMA1_SxM1AR*/
#define DMA1_S2M1AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 2 ) ) )
#define DMA1_S3M1AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 3 ) ) )
#define DMA1_S4M1AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 4 ) ) )
#define DMA1_S5M1AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 5 ) ) )
#define DMA1_S6M1AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 6 ) ) )
#define DMA1_S7M1AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 7 ) ) )
#define DMA1_S0FCR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 0 ) ) )
#define DMA1_S1FCR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 1 ) ) ) /* DMA1_SxFCR */
#define DMA1_S2FCR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 2 ) ) )
#define DMA1_S3FCR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 3 ) ) )
#define DMA1_S4FCR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 4 ) ) )
#define DMA1_S5FCR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 5 ) ) )
#define DMA1_S6FCR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 6 ) ) )
#define DMA1_S7FCR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 7 ) ) )

/*------------------------------------------------
DMA2 Register Address Accessors
------------------------------------------------*/
#define DMA2_LISR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_LISR ) )
#define DMA2_HISR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_HISR ) )
#define DMA2_LIFCR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_LIFCR ) )
#define DMA2_HIFCR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_HIFCR ) )
#define DMA2_S0CR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 0 ) ) )
#define DMA2_S1CR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 1 ) ) ) /* DMA2_SxCR */
#define DMA2_S2CR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 2 ) ) )
#define DMA2_S3CR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 3 ) ) )
#define DMA2_S4CR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 4 ) ) )
#define DMA2_S5CR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 5 ) ) )
#define DMA2_S6CR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 6 ) ) )
#define DMA2_S7CR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 7 ) ) )
#define DMA2_S0NDTR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 0 ) ) )
#define DMA2_S1NDTR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 1 ) ) ) /* DMA2_SxNDTR */
#define DMA2_S2NDTR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 2 ) ) )
#define DMA2_S3NDTR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 3 ) ) )
#define DMA2_S4NDTR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 4 ) ) )
#define DMA2_S5NDTR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 5 ) ) )
#define DMA2_S6NDTR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 6 ) ) )
#define DMA2_S7NDTR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 7 ) ) )
#define DMA2_S0PAR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 0 ) ) )
#define DMA2_S1PAR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 1 ) ) ) /* DMA2_SxPAR */
#define DMA2_S2PAR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 2 ) ) )
#define DMA2_S3PAR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 3 ) ) )
#define DMA2_S4PAR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 4 ) ) )
#define DMA2_S5PAR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 5 ) ) )
#define DMA2_S6PAR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 6 ) ) )
#define DMA2_S7PAR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 7 ) ) )
#define DMA2_S0M0AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 0 ) ) )
#define DMA2_S1M0AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 1 ) ) ) /* DMA2_SxM0AR */
#define DMA2_S2M0AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 2 ) ) )
#define DMA2_S3M0AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 3 ) ) )
#define DMA2_S4M0AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 4 ) ) )
#define DMA2_S5M0AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 5 ) ) )
#define DMA2_S6M0AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 6 ) ) )
#define DMA2_S7M0AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 7 ) ) )
#define DMA2_S0M1AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 0 ) ) )
#define DMA2_S1M1AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 1 ) ) ) /* DMA2_SxM1AR*/
#define DMA2_S2M1AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 2 ) ) )
#define DMA2_S3M1AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 3 ) ) )
#define DMA2_S4M1AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 4 ) ) )
#define DMA2_S5M1AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 5 ) ) )
#define DMA2_S6M1AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 6 ) ) )
#define DMA2_S7M1AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 7 ) ) )
#define DMA2_S0FCR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 0 ) ) )
#define DMA2_S1FCR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 1 ) ) ) /* DMA2_SxFCR */
#define DMA2_S2FCR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 2 ) ) )
#define DMA2_S3FCR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 3 ) ) )
#define DMA2_S4FCR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 4 ) ) )
#define DMA2_S5FCR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 5 ) ) )
#define DMA2_S6FCR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 6 ) ) )
#define DMA2_S7FCR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 7 ) ) )

  /*------------------------------------------------
  DMAx_SxCR Register
  ------------------------------------------------*/
  static constexpr uint32_t DMAx_SxCR_CHSEL_MSK = 0x07;
  static constexpr uint32_t DMAx_SxCR_CHSEL_POS = 25u;
  static constexpr uint32_t DMAx_SxCR_CHSEL     = DMAx_SxCR_CHSEL_MSK << DMAx_SxCR_CHSEL_POS;

}    // namespace Thor::DMA

#endif /* !THOR_DMA_DEFS_HPP */