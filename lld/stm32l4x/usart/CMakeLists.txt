include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_static_lib_variants(
  TARGET
    thor_lld_stm32l4_usart
  SOURCES
    hw_usart_driver.cpp
    hw_usart_data.cpp
    ${THOR_ROOT}/lld/interface/usart/common_driver/usart_common_driver.cpp
  PRV_LIBRARIES
    chimera_intf_inc
    thor_intf_inc
    Boost::boost
    lld_common_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld"
)
