# ====================================================
# Add the projects for all the supported drivers
# ====================================================
add_subdirectory("des")
add_subdirectory("flash")
add_subdirectory("gpio")
add_subdirectory("interrupt")
add_subdirectory("power")
add_subdirectory("rcc")
add_subdirectory("spi")
add_subdirectory("startup")
add_subdirectory("system")
add_subdirectory("timer")
add_subdirectory("usart")

# ====================================================
# All LLD Drivers
# ====================================================
set(LLD thor_lld_stm32l4)
add_library(${LLD} INTERFACE)
target_link_libraries(${LLD} INTERFACE
  thor_lld_stm32l4_des
  thor_lld_stm32l4_flash
  thor_lld_stm32l4_gpi
  thor_lld_stm32l4_interrupt
  thor_lld_stm32l4_power
  thor_lld_stm32l4_rcc
  thor_lld_stm32l4_spi
  thor_lld_stm32l4_startup
  thor_lld_stm32l4_syste
  thor_lld_stm32l4_timer
  thor_lld_stm32l4_usart
)
export(TARGETS ${LLD} FILE "${PROJECT_BINARY_DIR}/Thor/lld/${LLD}.cmake")