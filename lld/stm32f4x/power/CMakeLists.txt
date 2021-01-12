include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

# if(${THOR_CHIP} MATCHES "^F446$")
#   set(VARIANT_SRC variant/hw_power_register_stm32f446re.cpp)
# else()
#   message(FATAL_ERROR "Unsupported STM32F4 driver variant for: power")
# endif()


gen_static_lib_variants(
  TARGET
    thor_lld_stm32f4_power
  SOURCES
    # ${VARIANT_SRC}
    hw_power_driver.cpp
  PRV_LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    thor_intf_inc
    Boost::boost
    lld_common_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld"
)
