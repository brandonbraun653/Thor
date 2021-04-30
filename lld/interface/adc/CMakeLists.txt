include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_static_lib_variants(
  TARGET
    thor_lld_intf_adc
  SOURCES
    adc_intf.cpp
    adc_common_driver.cpp
  PRV_LIBRARIES
    chimera_intf_inc
    aurora_intf_inc
    thor_intf_inc
    lld_common_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld_intf"
)
