INCLUDE(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

if(WIN32)
    if(EXISTS "$ENV{CMAKE_GNU_ARM_EMBEDDED_TOOLCHAIN}")
        file(TO_CMAKE_PATH "$ENV{CMAKE_GNU_ARM_EMBEDDED_TOOLCHAIN}" WIN32_TOOLCHAIN_DIR)
    else()
        message(FATAL_ERROR "Could not find environment variable [CMAKE_GNU_ARM_EMBEDDED_TOOLCHAIN]. Please set this to the GNU ARM bin directory.")
    endif()

    CMAKE_FORCE_C_COMPILER("${WIN32_TOOLCHAIN_DIR}/arm-none-eabi-gcc.exe" GNU)
    CMAKE_FORCE_CXX_COMPILER("${WIN32_TOOLCHAIN_DIR}/arm-none-eabi-g++.exe" GNU)
endif(WIN32)

if(UNIX)
    set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
    set(CMAKE_CXX_COMPILER "arm-none-eabi-g++")
endif(UNIX)

set(CMAKE_EXE_LINKER_FLAGS "--specs=nano.specs" CACHE INTERNAL "")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

ADD_CUSTOM_TARGET(debug
  COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Debug ${CMAKE_SOURCE_DIR}
  COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target all
  COMMENT "Creating the executable in the debug mode.")

ADD_CUSTOM_TARGET(release
  COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Release ${CMAKE_SOURCE_DIR}
  COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target all
  COMMENT "Creating the executable in the release mode.")