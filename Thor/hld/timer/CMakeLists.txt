# ====================================================
# Common
# ====================================================
set(LINK_LIBS
  aurora_inc        # Aurora public headers
  Boost::boost      # Boost public headers
  thor_inc          # Thor public headers
  chimera_inc       # Chimera public headers
  lld_common_inc   # 3rd Party headers
  thor_hld_build_options
)

# ====================================================
# Interface Library
# ====================================================
set(HLD thor_hld_timer)
add_library(${HLD} STATIC
  hld_timer_chimera.cpp
  hld_timer_driver.cpp
  hld_timer_driver_advanced.cpp
  hld_timer_driver_basic.cpp
  hld_timer_driver_general.cpp
  hld_timer_driver_low_power.cpp
)
target_link_libraries(${HLD} PRIVATE ${LINK_LIBS})
export(TARGETS ${HLD} FILE "${PROJECT_BINARY_DIR}/Thor/hld/${HLD}.cmake")
