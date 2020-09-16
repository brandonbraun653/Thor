# ====================================================
# Common
# ====================================================
set(LINK_LIBS
  aurora_inc        # Aurora public headers
  type_safe_inc     # Type safe public headers
  thor_inc          # Thor public headers
  chimera_inc       # Chimera public headers
  prj_device_target # Compiler options for target device
)

# ====================================================
# Interface Library
# ====================================================
set(CFG thor_cfg_freertos)
add_library(${CFG} STATIC
  thor_freertos_hooks_prj.cpp
)
target_link_libraries(${CFG} PRIVATE ${LINK_LIBS})
export(TARGETS ${CFG} FILE "${PROJECT_BINARY_DIR}/Thor/cfg/${CFG}.cmake")
