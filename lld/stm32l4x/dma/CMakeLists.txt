include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

gen_static_lib_variants(
  TARGET
    thor_lld_stm32l4_dma
  SOURCES
    hw_dma_data.cpp
    hw_dma_irq.cpp
    hw_dma_stream.cpp
  PRV_LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    thor_intf_inc
    lld_common_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld"
)
