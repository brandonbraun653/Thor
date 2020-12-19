include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

# ====================================================
# Common
# ====================================================
set(LINK_LIBS
  thor_inc          # Thor public headers
  chimera_inc       # Chimera public headers
  lld_common_inc    # 3rd Party headers
)

# ====================================================
# Interface Library
# ====================================================
function(build_library variant)
  set(INTF thor_lld_intf_common${variant})
  add_library(${INTF} STATIC
    peripheral_interface.cpp
  )
  target_link_libraries(${INTF} PRIVATE ${LINK_LIBS} prj_device_target prj_build_target${variant})
  export(TARGETS ${INTF} FILE "${PROJECT_BINARY_DIR}/Thor/lld_intf/${INTF}.cmake")
endfunction()

add_target_variants(build_library)
