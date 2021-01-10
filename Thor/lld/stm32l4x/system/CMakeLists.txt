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
  set(VARIANT_SRC variant/hw_sys_register_stm32l432kc.cpp)
else()
  message(FATAL_ERROR "Unsupported STM32L4 driver variant for: sys")
endif()


# ====================================================
# Driver Library (+ Variant)
# ====================================================
function(build_library variant)
  set(CORE thor_lld_stm32l4_system${variant})
  add_library(${CORE} STATIC
    hw_sys_startup.cpp
    hw_sys_driver.cpp
    hw_sys_data.cpp
    ${VARIANT_SRC}
  )
  target_link_libraries(${CORE} PRIVATE ${LINK_LIBS} prj_device_target prj_build_target${variant})
  export(TARGETS ${CORE} FILE "${PROJECT_BINARY_DIR}/Thor/lld/${CORE}.cmake")
endfunction()

add_target_variants(build_library)
