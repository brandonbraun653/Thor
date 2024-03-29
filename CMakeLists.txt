# ====================================================
# Configure the Thor project appropriately
# ====================================================
include("${COMMON_TOOL_ROOT}/cmake/options/thor.cmake")

# ====================================================
# Import the Config, HLD, and LLD projects
# ====================================================
add_subdirectory("config")
add_subdirectory("hld")
add_subdirectory("lld")

# ====================================================
# Various exports
# ====================================================

# Public include location
gen_intf_lib(
  TARGET
    thor_intf_inc
  INTF_DEFINES
    BOOST_NO_EXCEPTIONS
  INTF_INCLUDES
    "./../"
    "include"
  INTF_LIBRARIES
    project_intf_inc
    chimera_intf_inc
    aurora_intf_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor"
)
