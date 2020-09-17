# ====================================================
# Common
# ====================================================
set(LINK_LIBS
  thor_inc          # Thor public headers
  chimera_inc       # Chimera public headers
  Boost::boost      # Boost headers
  lld_common_inc   # 3rd Party headers
  prj_device_target # Compiler options for target device
)

# ====================================================
# Simulated Low Level Driver
# ====================================================
set(SIM_LLD thor_lld_sim_usart)
add_library(${SIM_LLD} STATIC
  sim/usart_sim_data.cpp
  sim/usart_sim_driver.cpp
  sim/usart_sim_variant.cpp
)
target_link_libraries(${SIM_LLD} PRIVATE ${LINK_LIBS})
export(TARGETS ${SIM_LLD} FILE "${PROJECT_BINARY_DIR}/Thor/lld_intf/${SIM_LLD}.cmake")

# ====================================================
# Interface Library
# ====================================================
set(INTF thor_lld_intf_usart)
add_library(${INTF} STATIC
  usart_intf.cpp
)
target_link_libraries(${INTF} PRIVATE ${LINK_LIBS})
export(TARGETS ${INTF} FILE "${PROJECT_BINARY_DIR}/Thor/lld_intf/${INTF}.cmake")
