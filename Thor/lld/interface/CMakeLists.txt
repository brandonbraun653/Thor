include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

# ====================================================
# Add the projects for all the supported drivers
# ====================================================
add_subdirectory("adc")
add_subdirectory("can")
add_subdirectory("common")
add_subdirectory("crc")
add_subdirectory("des")
add_subdirectory("dma")
add_subdirectory("exti")
add_subdirectory("flash")
add_subdirectory("gpio")
add_subdirectory("interrupt")
add_subdirectory("power")
add_subdirectory("rcc")
add_subdirectory("spi")
add_subdirectory("sram")
add_subdirectory("system")
add_subdirectory("timer")
add_subdirectory("uart")
add_subdirectory("usart")
add_subdirectory("usb")
add_subdirectory("watchdog")

# ====================================================
# All Interface Low Level Drivers (Hardware Target)
# ====================================================
set(all_libraries
  thor_lld_intf_adc
  thor_lld_intf_can
  thor_lld_intf_common
  thor_lld_intf_exti
  thor_lld_intf_gpio
  thor_lld_intf_spi
  thor_lld_intf_sys
  thor_lld_intf_usart
  thor_lld_intf_usb
)

# ====================================================
# This function generates the following targets:
#   thor_lld_intf
#   thor_lld_intf_rel
#   thor_lld_intf_rel_dbg
#   thor_lld_intf_dbg
#
# Each target builds the appropriate variant of the
# libraries listed above in "all_libraries".
# ====================================================
function(build_thor_lld_intf_core variant)

  # Add the variant flag to each library target
  set(all_libraries_tmp${variant} "")
  foreach(lib ${all_libraries})
    list(APPEND all_libraries_tmp${variant} ${lib}${variant})
  endforeach()

  # Create interface library for the appropriate variant
  set(LLD_INTF${variant} thor_lld_intf${variant})
  add_library("${LLD_INTF${variant}}" INTERFACE)
  target_link_libraries("${LLD_INTF${variant}}" INTERFACE "${all_libraries_tmp${variant}}")
  export(TARGETS "${LLD_INTF${variant}}" FILE "${PROJECT_BINARY_DIR}/Thor/lld_intf/${LLD_INTF${variant}}.cmake")

endfunction()
add_target_variants(build_thor_lld_intf_core)


# ====================================================
# All Simulated Low Level Drivers
# ====================================================
set(LLD_INTF_SIM thor_lld_sim)
add_library(${LLD_INTF_SIM} INTERFACE)
target_link_libraries(${LLD_INTF_SIM} INTERFACE
  thor_lld_sim_spi
  thor_lld_sim_gpio
  thor_lld_sim_usart
)
export(TARGETS ${LLD_INTF_SIM} FILE "${PROJECT_BINARY_DIR}/Thor/lld_intf/${LLD_INTF_SIM}.cmake")

# ====================================================
# All Mocked Low Level Drivers
# ====================================================
set(LLD_INTF_MOCK thor_lld_mock)
add_library(${LLD_INTF_MOCK} INTERFACE)
target_link_libraries(${LLD_INTF_MOCK} INTERFACE
  thor_lld_mock_can
  thor_lld_mock_interrupt
)
export(TARGETS ${LLD_INTF_MOCK} FILE "${PROJECT_BINARY_DIR}/Thor/lld_intf/${LLD_INTF_MOCK}.cmake")
