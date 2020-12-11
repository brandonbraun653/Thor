# ====================================================
# Add the projects for all the supported drivers
# ====================================================
add_subdirectory("adc")
add_subdirectory("can")
add_subdirectory("crs")
add_subdirectory("des")
add_subdirectory("exti")
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
add_subdirectory("usb")

# ====================================================
# All LLD Drivers
# ====================================================
set(LLD thor_lld_stm32l4)
add_library(${LLD} INTERFACE)
target_link_libraries(${LLD} INTERFACE
  thor_lld_stm32l4_adc
  thor_lld_stm32l4_can
  thor_lld_stm32l4_crs
  thor_lld_stm32l4_des
  thor_lld_stm32l4_exti
  thor_lld_stm32l4_flash
  thor_lld_stm32l4_gpio
  thor_lld_stm32l4_interrupt
  thor_lld_stm32l4_power
  thor_lld_stm32l4_rcc
  thor_lld_stm32l4_spi
  thor_lld_stm32l4_startup
  thor_lld_stm32l4_system
  thor_lld_stm32l4_timer
  thor_lld_stm32l4_usart
  thor_lld_stm32l4_usb
)
export(TARGETS ${LLD} FILE "${PROJECT_BINARY_DIR}/Thor/lld/${LLD}.cmake")

# ====================================================
# Custom Compiler Options:
# By default always build release variant
# ====================================================
set(BUILD_OPTIONS thor_lld_stm32l4x_build_options)
add_library(${BUILD_OPTIONS} INTERFACE)
target_link_libraries(${BUILD_OPTIONS} INTERFACE
  prj_build_target
  #prj_build_target_rel
  #prj_build_target_dbg
  prj_device_target
)
export(TARGETS ${BUILD_OPTIONS} FILE "${PROJECT_BINARY_DIR}/Thor/${BUILD_OPTIONS}.cmake")