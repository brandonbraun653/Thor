include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_static_lib_variants(
  TARGET
    thor_lld_stm32f4_wwdg
  SOURCES
    hw_wwdg_data.cpp
    ../../interface/watchdog/common_driver/watchdog_wwdg_common_driver.cpp
  PRV_LIBRARIES
    chimera_intf_inc
    thor_intf_inc

    lld_common_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld"
)
