# --------------------------------------
# Add compiler options to a target
# --------------------------------------
function(SET_THOR_COMPILER_OPTIONS TARGET)
    # Note: Not sure if -fexceptions should be public. Do upstreams need to compile with it if thor does???
    target_compile_options(${TARGET} PUBLIC -fexceptions)
    target_compile_options(${TARGET} PUBLIC $<$<CONFIG:DEBUG>:-ggdb -Og>)
    target_compile_options(${TARGET} PUBLIC $<$<CONFIG:RELEASE>:-O3>)
    target_compile_options(${TARGET} PRIVATE --std=gnu++11)

    # For the target MCU, allow the architecture specific options be derived transitively from
    # the find_package() command. Each target exports their compiler options.
endfunction()

# --------------------------------------
# Add public compiler definitions to a target
# --------------------------------------
function(SET_THOR_COMPILER_DEFINITIONS TARGET)
    # Common to all builds
    target_compile_definitions(${TARGET} PUBLIC 
        -DUSING_FREERTOS 
        -DUSE_FREERTOS 
        -DEIGEN_INITIALIZE_MATRICES_BY_ZERO
    )

    # If we are exposing Chimera bindings, enable them through the preprocessor
    get_property(USING_CHIMERA TARGET ${TARGET} PROPERTY FEATURE_CHIMERA)
    if(USING_CHIMERA STREQUAL "ON")
        message(STATUS "THOR: Enabling Chimera bindings")
        target_compile_definitions(${TARGET} PUBLIC -DUSING_CHIMERA)
    endif()

    # Build type specific 
    target_compile_definitions(${TARGET} PUBLIC $<$<CONFIG:DEBUG>: -DDEBUG=1 -DDEBUG_DEFAULT_INTERRUPT_HANDLERS>)
    target_compile_definitions(${TARGET} PUBLIC $<$<CONFIG:RELEASE>: -DNDEBUG=1 -DRELEASE=1 -DEIGEN_NO_DEBUG>)
endfunction()