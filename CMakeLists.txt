# ====================================================
# Configure the Thor project appropriately
# ====================================================
include("${COMMON_TOOL_ROOT}/cmake/options/thor.cmake")

# ====================================================
# Import and Configure FreeRTOS (currently ignores FreeRTOS cache variables)
# ====================================================
if(Toolchain::REQUIRES_FREERTOS_THREADS)
  # Thor Config Options
  if(NOT CUSTOM_FREERTOS_CFG)
    message(STATUS "Using Thor default FreeRTOS configuration")
    add_library(freertos_cfg INTERFACE)
    target_include_directories(freertos_cfg INTERFACE "Thor/config/freertos")
    export(TARGETS freertos_cfg FILE "${PROJECT_BINARY_DIR}/ProjectConfig/freertos-cfg.cmake")
  endif()

  # Heap Selection
  add_library(freertos_heap INTERFACE)
  target_link_libraries(freertos_heap INTERFACE freertos_heap4)
  export(TARGETS freertos_heap FILE "${PROJECT_BINARY_DIR}/ProjectConfig/freertos-heap.cmake")

  # Port Selection
  if(${THOR_FAMILY} MATCHES "^[LF]4$")
    add_library(freertos_port INTERFACE)
    target_link_libraries(freertos_port INTERFACE freertos_cm4f)
    export(TARGETS freertos_port FILE "${PROJECT_BINARY_DIR}/ProjectConfig/freertos-port-cm4f.cmake")
  else()
    message(FATAL_ERROR "Need to select the proper freertos port lib to link against")
  endif()
endif()

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
