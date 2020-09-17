# ====================================================
# Common
# ====================================================
set(LINK_LIBS
  aurora_inc        # Aurora public headers
  Boost::boost      # Boost public headers
  thor_inc          # Thor public headers
  chimera_inc       # Chimera public headers
  lld_common_inc   # 3rd Party headers
  prj_device_target # Compiler options for target device
)

# ====================================================
# Interface Library
# ====================================================
set(HLD thor_hld_gpio)
add_library(${HLD} STATIC
  hld_gpio_chimera.cpp
  hld_gpio_driver.cpp
)
target_link_libraries(${HLD} PRIVATE ${LINK_LIBS})
export(TARGETS ${HLD} FILE "${PROJECT_BINARY_DIR}/Thor/hld/${HLD}.cmake")
