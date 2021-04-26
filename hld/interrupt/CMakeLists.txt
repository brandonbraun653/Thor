include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_static_lib_variants(
  TARGET
    thor_hld_interrupt
  SOURCES
    hld_interrupt_chimera.cpp
    hld_interrupt_driver.cpp
  PRV_LIBRARIES

    aurora_intf_inc
    chimera_intf_inc
    lld_common_inc
    thor_intf_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/hld"
)

