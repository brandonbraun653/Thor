# ====================================================
# Add the projects for all the supported drivers
# ====================================================
add_subdirectory("can")
add_subdirectory("clock")
add_subdirectory("common")
add_subdirectory("crc")
add_subdirectory("dma")
add_subdirectory("exti")
add_subdirectory("flash")
add_subdirectory("gpio")
add_subdirectory("interrupt")
add_subdirectory("power")
add_subdirectory("pwm")
add_subdirectory("spi")
add_subdirectory("sram")
add_subdirectory("system")
add_subdirectory("timer")
add_subdirectory("uart")
add_subdirectory("usart")
add_subdirectory("watchdog")

# ====================================================
# All HLD Drivers
# ====================================================
set(HLD thor_hld)
add_library(${HLD} INTERFACE)
target_link_libraries(${HLD} INTERFACE
  thor_hld_can
  thor_hld_clock
  thor_hld_dma
  thor_hld_exti
  thor_hld_gpio
  thor_hld_pwm
  thor_hld_spi
  thor_hld_system
  thor_hld_timer
  thor_hld_uart
  thor_hld_usart
  thor_hld_watchdog
)
export(TARGETS ${HLD} FILE "${PROJECT_BINARY_DIR}/Thor/hld/${HLD}.cmake")


# ====================================================
# Custom Compiler Options:
# By default always build release variant
# ====================================================
set(BUILD_OPTIONS thor_hld_build_options)
add_library(${BUILD_OPTIONS} INTERFACE)
target_link_libraries(${BUILD_OPTIONS} INTERFACE
  prj_build_target
  #prj_build_target_rel
  #prj_build_target_dbg
  prj_device_target
)
export(TARGETS ${BUILD_OPTIONS} FILE "${PROJECT_BINARY_DIR}/Thor/${BUILD_OPTIONS}.cmake")