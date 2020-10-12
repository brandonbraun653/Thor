# ====================================================
# Add the projects for all the supported drivers
# ====================================================
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
add_subdirectory("watchdog")

# ====================================================
# All Low Level Interface Drivers
# ====================================================
set(LLD_INTF thor_lld_intf)
add_library(${LLD_INTF} INTERFACE)
target_link_libraries(${LLD_INTF} INTERFACE
  thor_lld_intf_can
  thor_lld_intf_common
  thor_lld_intf_gpio
  thor_lld_intf_spi
  thor_lld_intf_usart
)
export(TARGETS ${LLD_INTF} FILE "${PROJECT_BINARY_DIR}/Thor/lld_intf/${LLD_INTF}.cmake")


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