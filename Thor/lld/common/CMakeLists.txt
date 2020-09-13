# ====================================================
# Various exports
# ====================================================

# LLD Common Include External Dependencies
add_library(lld_common_inc_dep INTERFACE)
target_link_libraries(lld_common_inc_dep INTERFACE type_safe_inc)
export(TARGETS lld_common_inc_dep FILE "${PROJECT_BINARY_DIR}/Thor/lld/common/inc-dep.cmake")