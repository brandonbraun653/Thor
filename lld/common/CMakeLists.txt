include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

# ====================================================
# Add sub-projects
# ====================================================
add_subdirectory("cortex-m4")

gen_static_lib_variants(
  TARGET
    thor_lld_cmn
  SOURCES
    sys_call_stubs.cpp
  PRV_LIBRARIES
    chimera_intf_inc
    thor_intf_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld/common"
)
