include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

# ====================================================
# Common
# ====================================================
set(LINK_LIBS
  aurora_intf_inc        # Aurora public headers
  type_safe_inc     # Type safe public headers
  thor_inc          # Thor public headers
  chimera_intf_inc       # Chimera public headers
)

# ====================================================
# Interface Library
# ====================================================
function(build_library variant)
  set(CFG thor_cfg_freertos${variant})
  add_library(${CFG} STATIC
    thor_freertos_hooks_prj.cpp
  )
  target_link_libraries(${CFG} PRIVATE ${LINK_LIBS} prj_device_target prj_build_target${variant})
  export(TARGETS ${CFG} FILE "${PROJECT_BINARY_DIR}/Thor/cfg/${CFG}.cmake")
endfunction()

add_target_variants(build_library)
