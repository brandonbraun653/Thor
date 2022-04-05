include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_static_lib_variants(
  TARGET
    thor_lld_intf_i2c
  SOURCES
    common_driver/i2c_cmn_data.cpp
    common_driver/i2c_cmn_intf.cpp
    common_driver/i2c_cmn_isr.cpp
  PRV_LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    thor_intf_inc
    lld_common_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld_intf"
)
