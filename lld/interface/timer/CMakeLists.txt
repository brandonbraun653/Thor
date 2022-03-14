include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_static_lib_variants(
  TARGET
    thor_lld_intf_timer
  SOURCES
    driver/lld_timer_advanced.cpp
    driver/lld_timer_basic.cpp
    driver/lld_timer_data.cpp
    driver/lld_timer_intf.cpp
    driver/lld_timer_general.cpp
  PRV_LIBRARIES
    chimera_intf_inc
    thor_intf_inc
    lld_common_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld_intf"
)
