include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

if(${THOR_CHIP} MATCHES "^F446$" AND ${THOR_IMPL} MATCHES "HW")
    set(VARIANT_SRC variant/hw_sdio_register_stm32f446xx.cpp)
elseif(${THOR_IMPL} MATCHES "^SIM$")
    set(VARIANT_SRC variant/hw_sdio_register_sim.cpp)
else()
    message(FATAL_ERROR "Unsupported STM32L4 driver variant for: sdio")
endif()

gen_static_lib_variants(
  TARGET
    thor_lld_stm32f4_sdio
  SOURCES
    ../../interface/sdio/sdio_common_driver.cpp
    hw_sdio_data.cpp
    ${VARIANT_SRC}
  PRV_LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    thor_intf_inc
    lld_common_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld"
)
