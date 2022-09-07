include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_intf_lib_variants(
  TARGET
    thor_lld_stm32f4_can
  LIBRARIES
    thor_lld_intf_bx_can
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld"
)
