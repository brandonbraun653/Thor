# ====================================================
# Common
# ====================================================
set(LINK_LIBS
  thor_inc          # Thor public headers
  chimera_inc       # Chimera public headers
  aurora_inc        # Aurora public headers
  Boost::boost      # Boost headers
  lld_common_inc   # 3rd Party headers
  prj_device_target # Compiler options for target device
)

# ====================================================
# Driver Library (+ Variant)
# ====================================================
set(CORE thor_lld_stm32l4_system)
add_library(${CORE} STATIC
  sys_common_init.cpp
)
target_link_libraries(${CORE} PRIVATE ${LINK_LIBS})
export(TARGETS ${CORE} FILE "${PROJECT_BINARY_DIR}/Thor/lld/${CORE}.cmake")
