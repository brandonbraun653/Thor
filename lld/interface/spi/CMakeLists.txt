include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_static_lib_variants(
  TARGET
    thor_lld_intf_spi
  SOURCES
    spi_intf.cpp
  PRV_LIBRARIES
    chimera_intf_inc
    thor_intf_inc

    lld_common_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld_intf"
)
