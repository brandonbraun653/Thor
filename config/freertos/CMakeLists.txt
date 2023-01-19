include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_static_lib_variants(
  TARGET
    thor_cfg_freertos
  SOURCES
    thor_freertos_hooks_prj.cpp
  PRV_LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    thor_intf_inc
    project_intf_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld"
)
