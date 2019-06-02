## Folder Structure
---------------------
At the root directory, all of the top level interfaces for various hardware peripherals can be accessed through their associated header file. If the user wants a GPIO, SPI, Serial, etc object, this is where they can find the classes that allow their instantiation. Nearly all the peripheral drivers are derived directly from the Chimera HAL interface specifications, so these drivers should function as drop-in replacements to any existing software that uses Chimera as the embedded backend.

#### Config
-----------
This directory provides various configuration files used when compiling/linking to external libraries. For example, if using Thor as the backend driver for Chimera, the requisite chimeraPort.hpp file that informs Chimera of various configuration details is included inside of config/chimera.

#### Defaults
-------------
These are some of the default configuration options needed to initialize the low level registers for the various STM32 hardware peripherals, which takes a load off the user as they don't have to remember 50 configuration settings just to get a driver working. This takes up some RO data in the final compiled binary (estimated ~10k), but for the moment it works out ok. Most STM32 chips are not cramped for space. Eventually an attempt will be made to compress it as small as possible, but that simply isn't the highest priority right now.

#### Definitions
----------------
This is the location for common data that describes what kind of hardware is available, minimum and maximum operating conditions, etc. 


#### Drivers
------------
Currently in development, this is where all of the custom (ie not STM32HAL) peripheral drivers are located. While the STM32HAL gets you up to speed very quickly, it is bulky and does not take advantage of modern C++ features. While this is not necessarily bad (a lot of great code is in C), I personally would like to have faster, lighter, and asynchronous aware drivers written with more advanced C++ systems in mind. Plus it's a great exercise in library design, so keep on the lookout for updates to this folder.

#### Types
----------
Location for all the driver data structures.