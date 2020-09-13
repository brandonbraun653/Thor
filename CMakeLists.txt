# ====================================================
# Import the Config, HLD, and LLD projects
# ====================================================
add_subdirectory("Thor/config")
add_subdirectory("Thor/hld")
add_subdirectory("Thor/lld")

# ====================================================
# Various exports
# ====================================================

# Public include location
add_library(thor_inc INTERFACE)
target_include_directories(thor_inc INTERFACE ".")
target_compile_definitions(thor_inc INTERFACE BOOST_NO_EXCEPTIONS)
export(TARGETS thor_inc FILE "${PROJECT_BINARY_DIR}/Thor/thor-inc.cmake")
