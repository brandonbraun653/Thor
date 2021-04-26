include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

# ====================================================
# Common
# ====================================================
set(LINK_LIBS
  thor_intf_inc          # Thor public headers
  chimera_intf_inc       # Chimera public headers
  aurora_intf_inc        # Aurora public headers
        # Boost headers
  lld_common_inc    # 3rd Party headers
)

# ====================================================
# Driver Library (+ Variant)
# ====================================================
function(build_library variant)
  set(CORE thor_lld_stm32l4_adc${variant})
  add_library(${CORE} STATIC
    hw_adc_driver.cpp
    hw_adc_data.cpp
  )
  target_link_libraries(${CORE} PRIVATE ${LINK_LIBS} prj_device_target prj_build_target${variant})
  export(TARGETS ${CORE} FILE "${PROJECT_BINARY_DIR}/Thor/lld/${CORE}.cmake")
endfunction()

add_target_variants(build_library)
