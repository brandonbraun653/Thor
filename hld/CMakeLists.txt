include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

# ====================================================
# Add the projects for all the supported drivers
# ====================================================
add_subdirectory("adc")
add_subdirectory("can")
add_subdirectory("clock")
add_subdirectory("common")
add_subdirectory("crc")
add_subdirectory("dma")
add_subdirectory("exti")
add_subdirectory("flash")
add_subdirectory("gpio")
add_subdirectory("interrupt")
add_subdirectory("pwm")
add_subdirectory("spi")
add_subdirectory("sram")
add_subdirectory("system")
add_subdirectory("timer")
add_subdirectory("uart")
add_subdirectory("usart")
add_subdirectory("usb")
add_subdirectory("watchdog")

# ====================================================
# All HLD Drivers
# ====================================================
set(all_libraries
  thor_hld_adc
  thor_hld_can
  thor_hld_clock
  thor_hld_dma
  thor_hld_exti
  thor_hld_gpio
  thor_hld_interrupt
  thor_hld_pwm
  thor_hld_spi
  thor_hld_system
  thor_hld_timer
  thor_hld_uart
  thor_hld_usart
  thor_hld_usb
  thor_hld_watchdog
)

# ====================================================
# This function generates the following targets:
#   thor_hld_core
#   thor_hld_core_rel
#   thor_hld_core_rel_dbg
#   thor_hld_core_dbg
#
# Each target builds the appropriate variant of the
# libraries listed above in "all_libraries".
# ====================================================
function(build_thor_core variant)

  # Add the variant flag to each library target
  set(all_libraries_tmp${variant} "")
  foreach(lib ${all_libraries})
    list(APPEND all_libraries_tmp${variant} ${lib}${variant})
  endforeach()

  # Create interface library for the appropriate variant
  set(THOR_HLD${variant} thor_hld_core${variant})
  add_library("${THOR_HLD${variant}}" INTERFACE)
  target_link_libraries("${THOR_HLD${variant}}" INTERFACE "${all_libraries_tmp${variant}}")
  export(TARGETS "${THOR_HLD${variant}}" FILE "${PROJECT_BINARY_DIR}/Thor/hld/${THOR_HLD${variant}}.cmake")

endfunction()
add_target_variants(build_thor_core)


# ====================================================
# Custom Compiler Options:
# By default always build release variant
# ====================================================
set(BUILD_OPTIONS thor_hld_build_options)
add_library(${BUILD_OPTIONS} INTERFACE)
target_link_libraries(${BUILD_OPTIONS} INTERFACE
  prj_build_target
  prj_device_target
)
export(TARGETS ${BUILD_OPTIONS} FILE "${PROJECT_BINARY_DIR}/Thor/${BUILD_OPTIONS}.cmake")