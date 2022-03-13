# Timer Driver Interface
This peripheral driver is a bit unique compared to other driver interfaces due to ST having shared the core Timer framework across a majority of
devices the Thor project could ever care about. This leads to a super driver at the LLD interface level with only a few small details that may
differ being defined in the chip family driver sub-folders.

## References
- AN4013: STM32 cross-series timer overview
- AN4776: General-purpose timer cookbook for STM32 microcontrollers