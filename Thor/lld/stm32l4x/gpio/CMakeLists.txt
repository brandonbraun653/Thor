# ====================================================
# Core Library
# ====================================================
add_library(thor_lld_stm32l4_gpio STATIC
  hw_gpio_data.cpp
  hw_gpio_driver.cpp
)
target_link_libraries(thor_lld_stm32l4_gpio PRIVATE thor_inc chimera_inc lld_ext_inc_dep prj_device_target)

export(TARGETS thor_lld_stm32l4_gpio FILE "${PROJECT_BINARY_DIR}/Thor/lld/stm32l4-gpio.cmake")
