include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

# ====================================================
# Common
# ====================================================
set(LINK_LIBS
  thor_inc          # Thor public headers
  chimera_intf_inc       # Chimera public headers
  Boost::boost      # Boost headers
  lld_common_inc    # 3rd Party headers
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
# Mocked Low Level Driver
# ====================================================
set(MOCK_LLD thor_lld_mock_usart)
add_library(${MOCK_LLD} STATIC
  mock/usart_mock.cpp
  mock/usart_mock_variant.cpp
)
target_link_libraries(${MOCK_LLD} PRIVATE ${LINK_LIBS} gmock_inc)
export(TARGETS ${MOCK_LLD} FILE "${PROJECT_BINARY_DIR}/Thor/lld_intf/${MOCK_LLD}.cmake")

# ====================================================
# Interface Library
# ====================================================
function(build_library variant)
  set(INTF thor_lld_intf_usart${variant})
  add_library(${INTF} STATIC
    usart_intf.cpp
  )
  target_link_libraries(${INTF} PRIVATE ${LINK_LIBS} prj_device_target prj_build_target${variant})
  export(TARGETS ${INTF} FILE "${PROJECT_BINARY_DIR}/Thor/lld_intf/${INTF}.cmake")
endfunction()

add_target_variants(build_library)
