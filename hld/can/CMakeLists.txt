include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_static_lib_variants(
  TARGET
    thor_hld_can
  SOURCES
    hld_can_chimera.cpp
    hld_can_driver.cpp
  PRV_LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    lld_common_inc
    project_inc
    thor_intf_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/hld"
)
