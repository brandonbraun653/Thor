include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_static_lib_variants(
  TARGET
    thor_lld_stm32f4_spi
  SOURCES
    ./../../../lld/interface/spi/common_driver/spi_common_driver.cpp
    hw_spi_data.cpp
    hw_spi_driver.cpp
  PRV_LIBRARIES
    chimera_intf_inc
    thor_intf_inc
    lld_common_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld"
)
