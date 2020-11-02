# ====================================================
# Common
# ====================================================
set(LINK_LIBS
  thor_inc          # Thor public headers
  chimera_inc       # Chimera public headers
  Boost::boost      # Boost headers
  lld_common_inc   # 3rd Party headers
  thor_lld_intf_build_options
)

# ====================================================
# Simulated Low Level Driver
# ====================================================
set(SIM_LLD thor_lld_sim_exti)
add_library(${SIM_LLD} STATIC
  sim/exti_sim_data.cpp
  sim/exti_sim_driver.cpp
  sim/exti_sim_variant.cpp
)
target_link_libraries(${SIM_LLD} PRIVATE ${LINK_LIBS})
export(TARGETS ${SIM_LLD} FILE "${PROJECT_BINARY_DIR}/Thor/lld_intf/${SIM_LLD}.cmake")


# ====================================================
# Mocked Low Level Driver
# ====================================================
set(MOCK_LLD thor_lld_mock_exti)
add_library(${MOCK_LLD} STATIC
  mock/exti_mock.cpp
  mock/exti_mock_variant.cpp
)
target_link_libraries(${MOCK_LLD} PRIVATE ${LINK_LIBS} gmock_inc)
export(TARGETS ${MOCK_LLD} FILE "${PROJECT_BINARY_DIR}/Thor/lld_intf/${MOCK_LLD}.cmake")

# ====================================================
# Interface Library
# ====================================================
set(INTF thor_lld_intf_exti)
add_library(${INTF} STATIC
  exti_intf.cpp
)
target_link_libraries(${INTF} PRIVATE ${LINK_LIBS})
export(TARGETS ${INTF} FILE "${PROJECT_BINARY_DIR}/Thor/lld_intf/${INTF}.cmake")
