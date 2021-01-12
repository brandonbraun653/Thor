include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

if(${THOR_CHIP} MATCHES "^F446$")
  set(VARIANT_SRC variant/hw_iwdg_register_stm32f446re.cpp)
else()
  message(FATAL_ERROR "Unsupported STM32F4 driver variant for: iwdg")
endif()


gen_static_lib_variants(
  TARGET
    thor_lld_stm32f4_iwdg
  SOURCES
    ${VARIANT_SRC}
    hw_iwdg_driver.cpp
    hw_iwdg_data.cpp
  PRV_LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    thor_intf_inc
    Boost::boost
    lld_common_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld"
)
