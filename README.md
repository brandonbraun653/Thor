# Thor

Thor is a translation layer that wraps around the well known STM32 HAL and exposes it using the [Chimera](https://github.com/brandonbraun653/Chimera) interface. This
allows all of my projects (perhaps yours too?) to be able to utilize the entire STM32 lineup without too much hassle.

## Required Repositories ##
1. [CMSIS](https://github.com/ARM-software/CMSIS_5)
2. [Chimera](https://github.com/brandonbraun653/Chimera)

## Semi-Optional ##
Choose at least one of these back-end STM32 HAL drivers to compile with.

1. [STM32F7](https://github.com/brandonbraun653/STM32HAL_F7)
2. [STM32F4](https://github.com/brandonbraun653/STM32HAL_F4)
3. More coming later...
