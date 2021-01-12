include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

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
add_subdirectory("iwdg")
add_subdirectory("power")
add_subdirectory("rcc")
add_subdirectory("spi")
add_subdirectory("startup")
add_subdirectory("system")
add_subdirectory("timer")
add_subdirectory("usart")
add_subdirectory("usb")
add_subdirectory("wwdg")

# ====================================================
# All LLD Drivers
# ====================================================
set(all_libraries
  thor_lld_stm32l4_adc
  thor_lld_stm32l4_can
  thor_lld_stm32l4_crs
  thor_lld_stm32l4_des
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
  thor_lld_stm32l4_timer
  thor_lld_stm32l4_usart
  thor_lld_stm32l4_usb
  thor_lld_stm32l4_wwdg
)

# ====================================================
# This function generates the following targets:
#   thor_lld_stm32l4
#   thor_lld_stm32l4_rel
#   thor_lld_stm32l4_rel_dbg
#   thor_lld_stm32l4_dbg
#
# Each target builds the appropriate variant of the
# libraries listed above in "all_libraries".
# ====================================================
function(build_thor_lld_stm32l4_core variant)

  # Add the variant flag to each library target
  set(all_libraries_tmp${variant} "")
  foreach(lib ${all_libraries})
    list(APPEND all_libraries_tmp${variant} ${lib}${variant})
  endforeach()

  # Create interface library for the appropriate variant
  set(LLD${variant} thor_lld_stm32l4${variant})
  add_library("${LLD${variant}}" INTERFACE)
  target_link_libraries("${LLD${variant}}" INTERFACE "${all_libraries_tmp${variant}}")
  export(TARGETS "${LLD${variant}}" FILE "${PROJECT_BINARY_DIR}/Thor/lld/${LLD${variant}}.cmake")

endfunction()
add_target_variants(build_thor_lld_stm32l4_core)
