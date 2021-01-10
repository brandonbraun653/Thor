include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

# ====================================================
# Common
# ====================================================
set(LINK_LIBS
  thor_inc          # Thor public headers
  chimera_intf_inc       # Chimera public headers
  aurora_intf_inc        # Aurora public headers
  Boost::boost      # Boost headers
  lld_common_inc    # 3rd Party headers
)

# ====================================================
# Variant Library
# ====================================================
if(${THOR_CHIP} MATCHES "^L432$")
  set(VARIANT_SRC variant/hw_timer_register_stm32l4xxxx.cpp)
else()
  message(FATAL_ERROR "Unsupported STM32L4 driver variant for: timer")
endif()

# ====================================================
# Driver Library (+ Variant)
# ====================================================
function(build_library variant)
  set(CORE thor_lld_stm32l4_timer${variant})
  add_library(${CORE} STATIC
    hw_timer_driver_stm32l4.cpp
    hw_timer_driver_stm32l4_general.cpp
    hw_timer_mapping.cpp
    ${VARIANT_SRC}
  )
  target_link_libraries(${CORE} PRIVATE ${LINK_LIBS} prj_device_target prj_build_target${variant})
  export(TARGETS ${CORE} FILE "${PROJECT_BINARY_DIR}/Thor/lld/${CORE}.cmake")
endfunction()

add_target_variants(build_library)
