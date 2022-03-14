include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

# ====================================================
# Add the projects for all the supported drivers
# ====================================================
add_subdirectory("adc")
add_subdirectory("can")
add_subdirectory("crs")
add_subdirectory("des")
add_subdirectory("dma")
add_subdirectory("exti")
add_subdirectory("flash")
add_subdirectory("gpio")
add_subdirectory("interrupt")
add_subdirectory("iwdg")
add_subdirectory("power")
add_subdirectory("rcc")
add_subdirectory("spi")
add_subdirectory("startup")
add_subdirectory("system")
add_subdirectory("timer")
add_subdirectory("usart")
add_subdirectory("wwdg")

# ====================================================
# All LLD Drivers
# ====================================================
gen_intf_lib_variants(
  TARGET
    thor_lld_stm32l4
  LIBRARIES
    thor_lld_stm32l4_adc
    thor_lld_stm32l4_can
    thor_lld_stm32l4_crs
    thor_lld_stm32l4_des
    thor_lld_stm32l4_dma
    thor_lld_stm32l4_exti
    thor_lld_stm32l4_flash
    thor_lld_stm32l4_gpio
    thor_lld_stm32l4_interrupt
    thor_lld_stm32l4_iwdg
    thor_lld_stm32l4_power
    thor_lld_stm32l4_rcc
    thor_lld_stm32l4_spi
    thor_lld_stm32l4_startup
    thor_lld_stm32l4_system
    #thor_lld_stm32l4_timer
    thor_lld_stm32l4_usart
    thor_lld_stm32l4_wwdg
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld"
)
