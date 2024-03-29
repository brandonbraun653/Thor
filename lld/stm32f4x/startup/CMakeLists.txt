include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")


if(${THOR_CHIP} MATCHES "^F446$")
  set(VARIANT_SRC hw_startup_stm32f446xx.c)
elseif(${THOR_CHIP} MATCHES "^SIM$")
  message(WARNING "Missing STM32F4 simulator driver variant for: startup")
  set(VARIANT_SRC hw_startup_stm32f446xx.c)
else()
  message(FATAL_ERROR "Unsupported STM32F4 driver variant for: startup")
endif()


gen_static_lib_variants(
  TARGET
    thor_lld_stm32f4_startup
  SOURCES
    ${VARIANT_SRC}
  PRV_LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    thor_intf_inc

    lld_common_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld"
)
