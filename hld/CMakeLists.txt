include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

# ====================================================
# Add the projects for all the supported drivers
# ====================================================
add_subdirectory("adc")
add_subdirectory("can")
add_subdirectory("clock")
add_subdirectory("common")
add_subdirectory("dma")
add_subdirectory("exti")
add_subdirectory("gpio")
add_subdirectory("i2c")
add_subdirectory("interrupt")
add_subdirectory("sdio")
add_subdirectory("serial")
add_subdirectory("spi")
add_subdirectory("system")
add_subdirectory("timer")
add_subdirectory("uart")
add_subdirectory("usart")
add_subdirectory("watchdog")

# ====================================================
# All HLD Drivers
# ====================================================
gen_intf_lib_variants(
  TARGET
    thor_hld_core

  LIBRARIES
    thor_hld_adc
    thor_hld_can
    thor_hld_clock
    thor_hld_dma
    thor_hld_exti
    thor_hld_gpio
    thor_hld_i2c
    thor_hld_interrupt
    thor_hld_sdio
    thor_hld_serial
    thor_hld_spi
    thor_hld_system
    thor_hld_timer
    thor_hld_uart
    thor_hld_usart
    thor_hld_watchdog

  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/hld"
)
