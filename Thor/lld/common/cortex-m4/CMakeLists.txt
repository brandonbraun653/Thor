include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_static_lib_variants(
  TARGET
    thor_cmn_cm4
  SOURCES
    cm4_exceptions.cpp
    cm4_interrupts.cpp
    cm4_startup_reset.cpp
    cm4_system_time.cpp
    cm4_utilities.cpp
  PRV_LIBRARIES
    chimera_intf_inc
    thor_intf_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld/common"
)
