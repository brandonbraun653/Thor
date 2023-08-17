include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")


if(${THOR_CHIP} MATCHES "^F446$")
  set(VARIANT_SRC variant/hw_des_register_stm32f446xx.cpp)
elseif(${THOR_CHIP} MATCHES "^SIM$")
  message(WARNING "Missing STM32F4 simulator driver variant for: des")
else()
  message(FATAL_ERROR "Unsupported STM32F4 driver variant for: des")
endif()


gen_static_lib_variants(
  TARGET
    thor_lld_stm32f4_des
  SOURCES
    ${VARIANT_SRC}
    hw_des_driver.cpp
  PRV_LIBRARIES
    chimera_intf_inc
    thor_intf_inc
    lld_common_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld"
)
