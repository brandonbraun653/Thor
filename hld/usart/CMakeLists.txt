include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_static_lib_variants(
  TARGET
    thor_hld_usart
  SOURCES
    hld_usart_driver.cpp
  PRV_LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    thor_intf_inc
    lld_common_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/hld"
)
