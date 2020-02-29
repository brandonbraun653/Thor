# Thor

Thor is my personal attempt at learning how to architect writing a Hardware Abstraction layer for STM32 series MCUs. It is broken up 
into a High Level Driver (HLD) and a Low Level Driver (LLD). The high level driver handles common functionality that you would expect
from modern hardware peripheral drivers, like thread awareness and asynchronous operation. The low level driver is a chip specific,
bare metal driver that controls the peripheral register level. Using a predetermined interface, the HLD and LLD work together to allow
swapping processors without too much hassle. Simply provide a LLD implementation and you are good to go.

The HLD is built on top of the Chimera interface specification (my personal spec), which is a labor of love and always a work in progress,
so if you have suggestions for improvements I would love to hear them.

## Required Repositories ##
[Chimera](https://github.com/brandonbraun653/Chimera)
Provides the HLD interface specification and common defintions.

## Optional Repositories ##
[FreeRTOS](https://github.com/brandonbraun653/FreeRTOS)
Slightly modified FreeRTOS 10.0.0 to integrate better with my build system. Functionally the same as the released FreeRTOS.

## Build System ##
- Boost
- MSBuild
- VisualGDB
