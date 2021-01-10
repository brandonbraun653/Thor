include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

# ====================================================
# Common
# ====================================================
set(LINK_LIBS
  thor_inc          # Thor public headers
  chimera_intf_inc       # Chimera public headers
)

# ====================================================
# Driver Library
# ====================================================
function(build_library variant)
  set(CMN thor_cmn_cm4${variant})
  add_library(${CMN} STATIC
    cm4_interrupts.cpp
    cm4_exceptions.cpp
    cm4_system_time.cpp
    cm4_utilities.cpp
  )
  target_link_libraries(${CMN} PRIVATE ${LINK_LIBS} ${VARIANT} prj_device_target prj_build_target${variant})
  export(TARGETS ${CMN} FILE "${PROJECT_BINARY_DIR}/Thor/lld/common/${CMN}.cmake")
endfunction()

add_target_variants(build_library)
