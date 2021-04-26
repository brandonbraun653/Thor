include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_static_lib_variants(
  TARGET
    thor_lld_stm32f4_rcc
  SOURCES
    hw_rcc_data.cpp
    hw_rcc_driver.cpp
    hw_rcc_periph_ctrl.cpp
    hw_rcc_pll.cpp
    hw_rcc_prv.cpp
    hw_rcc_sys_ctrl.cpp
  PRV_LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    thor_intf_inc

    lld_common_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld"
)

