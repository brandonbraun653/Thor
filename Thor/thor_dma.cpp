/********************************************************************************
* File Name:
*   thor_dma.cpp
*     
* Description:
*   Implements the Thor DMA driver
* 
* 2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

/* C++ Includes */
#include <array>

/* Thor Includes */
#include <Thor/thor.hpp>
#include <Thor/dma.hpp>


static const std::array<std::array<uint8_t, 7>, 7> dma1RequestMapping =
{ {
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0 }
}};
