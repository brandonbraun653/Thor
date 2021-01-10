# ====================================================
# Common
# ====================================================
set(LINK_LIBS
  thor_inc          # Thor public headers
  chimera_intf_inc       # Chimera public headers
  Boost::boost      # Boost headers
  lld_common_inc    # 3rd Party headers
  thor_lld_intf_build_options
)

# ====================================================
# Mocked Low Level Driver
# ====================================================
set(MOCK_LLD thor_lld_mock_interrupt)
add_library(${MOCK_LLD} STATIC
  mock/interrupt_mock.cpp
)
target_link_libraries(${MOCK_LLD} PRIVATE ${LINK_LIBS} gmock_inc)
export(TARGETS ${MOCK_LLD} FILE "${PROJECT_BINARY_DIR}/Thor/lld_mock/${MOCK_LLD}.cmake")

