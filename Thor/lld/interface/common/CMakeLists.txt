# ====================================================
# Common
# ====================================================
set(LINK_LIBS
  thor_inc          # Thor public headers
  chimera_inc       # Chimera public headers
  lld_common_inc   # 3rd Party headers
  thor_lld_intf_build_options
)

# ====================================================
# Interface Library
# ====================================================
set(INTF thor_lld_intf_common)
add_library(${INTF} STATIC
  peripheral_interface.cpp
)
target_link_libraries(${INTF} PRIVATE ${LINK_LIBS})
export(TARGETS ${INTF} FILE "${PROJECT_BINARY_DIR}/Thor/lld_intf/${INTF}.cmake")
