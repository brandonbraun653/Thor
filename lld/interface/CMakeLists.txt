include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

# ====================================================
# Add the projects for all the supported drivers
# ====================================================
add_subdirectory("adc")
add_subdirectory("can")
add_subdirectory("dma")
add_subdirectory("exti")
add_subdirectory("gpio")
add_subdirectory("i2c")
add_subdirectory("interrupt")
add_subdirectory("rcc")
add_subdirectory("spi")
add_subdirectory("system")
add_subdirectory("timer")
add_subdirectory("usart")
add_subdirectory("watchdog")

# ====================================================
# All Interface Low Level Drivers (Hardware Target)
# ====================================================
gen_intf_lib_variants(
  TARGET
    thor_lld_intf

  LIBRARIES
    thor_lld_intf_adc
    thor_lld_intf_can
    thor_lld_intf_dma
    thor_lld_intf_exti
    thor_lld_intf_gpio
    thor_lld_intf_i2c
    thor_lld_intf_interrupt
    thor_lld_intf_rcc
    thor_lld_intf_spi
    thor_lld_intf_sys
    thor_lld_intf_timer
    thor_lld_intf_usart
    thor_lld_intf_usb
    thor_lld_intf_watchdog

  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld_intf"
)
