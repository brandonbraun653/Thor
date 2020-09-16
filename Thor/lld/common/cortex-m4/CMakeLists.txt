# ====================================================
# Common
# ====================================================
set(LINK_LIBS
  thor_inc          # Thor public headers
  chimera_inc       # Chimera public headers
  prj_device_target # Compiler options for target device
)

# ====================================================
# Driver Library
# ====================================================
set(CMN thor_cmn_cm4)
add_library(${CMN} STATIC
  cm4_interrupts.cpp
)
target_link_libraries(${CMN} PRIVATE ${LINK_LIBS} ${VARIANT})
export(TARGETS ${CMN} FILE "${PROJECT_BINARY_DIR}/Thor/lld/common/${CMN}.cmake")
