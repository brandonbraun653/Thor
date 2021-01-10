include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

# # ====================================================
# Common
# ====================================================
set(LINK_LIBS
  aurora_intf_inc        # Aurora public headers
  Boost::boost      # Boost public headers
  thor_inc          # Thor public headers
  chimera_intf_inc       # Chimera public headers
  lld_common_inc    # 3rd Party headers
)

# ====================================================
# Interface Library
# ====================================================
function(build_library variant)
  set(HLD thor_hld_watchdog${variant})
  add_library(${HLD} STATIC
    hld_watchdog_chimera.cpp
    hld_watchdog_driver.cpp
  )
  target_link_libraries(${HLD} PRIVATE ${LINK_LIBS} prj_device_target prj_build_target${variant})
  export(TARGETS ${HLD} FILE "${PROJECT_BINARY_DIR}/Thor/hld/${HLD}.cmake")
endfunction()

add_target_variants(build_library)
