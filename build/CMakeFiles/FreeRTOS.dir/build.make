# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.12

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = G:\ProgramFiles\CMake\bin\cmake.exe

# The command to remove a file.
RM = G:\ProgramFiles\CMake\bin\cmake.exe -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = G:\git\Microcontrollers\Thor_STM32

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = G:\git\Microcontrollers\Thor_STM32\build

# Include any dependencies generated for this target.
include CMakeFiles/FreeRTOS.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/FreeRTOS.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/FreeRTOS.dir/flags.make

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/GCC/ARM_CM4F/port.c.obj: CMakeFiles/FreeRTOS.dir/flags.make
CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/GCC/ARM_CM4F/port.c.obj: ../Thor/lib/FreeRTOS/portable/GCC/ARM_CM4F/port.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=G:\git\Microcontrollers\Thor_STM32\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/GCC/ARM_CM4F/port.c.obj"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\portable\GCC\ARM_CM4F\port.c.obj   -c G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\portable\GCC\ARM_CM4F\port.c

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/GCC/ARM_CM4F/port.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/GCC/ARM_CM4F/port.c.i"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\portable\GCC\ARM_CM4F\port.c > CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\portable\GCC\ARM_CM4F\port.c.i

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/GCC/ARM_CM4F/port.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/GCC/ARM_CM4F/port.c.s"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\portable\GCC\ARM_CM4F\port.c -o CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\portable\GCC\ARM_CM4F\port.c.s

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/Common/mpu_wrappers.c.obj: CMakeFiles/FreeRTOS.dir/flags.make
CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/Common/mpu_wrappers.c.obj: ../Thor/lib/FreeRTOS/portable/Common/mpu_wrappers.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=G:\git\Microcontrollers\Thor_STM32\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/Common/mpu_wrappers.c.obj"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\portable\Common\mpu_wrappers.c.obj   -c G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\portable\Common\mpu_wrappers.c

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/Common/mpu_wrappers.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/Common/mpu_wrappers.c.i"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\portable\Common\mpu_wrappers.c > CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\portable\Common\mpu_wrappers.c.i

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/Common/mpu_wrappers.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/Common/mpu_wrappers.c.s"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\portable\Common\mpu_wrappers.c -o CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\portable\Common\mpu_wrappers.c.s

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/MemMang/heap_4.c.obj: CMakeFiles/FreeRTOS.dir/flags.make
CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/MemMang/heap_4.c.obj: ../Thor/lib/FreeRTOS/portable/MemMang/heap_4.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=G:\git\Microcontrollers\Thor_STM32\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/MemMang/heap_4.c.obj"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\portable\MemMang\heap_4.c.obj   -c G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\portable\MemMang\heap_4.c

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/MemMang/heap_4.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/MemMang/heap_4.c.i"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\portable\MemMang\heap_4.c > CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\portable\MemMang\heap_4.c.i

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/MemMang/heap_4.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/MemMang/heap_4.c.s"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\portable\MemMang\heap_4.c -o CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\portable\MemMang\heap_4.c.s

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/croutine.c.obj: CMakeFiles/FreeRTOS.dir/flags.make
CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/croutine.c.obj: ../Thor/lib/FreeRTOS/croutine.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=G:\git\Microcontrollers\Thor_STM32\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/croutine.c.obj"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\croutine.c.obj   -c G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\croutine.c

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/croutine.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/croutine.c.i"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\croutine.c > CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\croutine.c.i

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/croutine.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/croutine.c.s"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\croutine.c -o CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\croutine.c.s

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/event_groups.c.obj: CMakeFiles/FreeRTOS.dir/flags.make
CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/event_groups.c.obj: ../Thor/lib/FreeRTOS/event_groups.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=G:\git\Microcontrollers\Thor_STM32\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/event_groups.c.obj"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\event_groups.c.obj   -c G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\event_groups.c

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/event_groups.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/event_groups.c.i"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\event_groups.c > CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\event_groups.c.i

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/event_groups.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/event_groups.c.s"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\event_groups.c -o CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\event_groups.c.s

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/list.c.obj: CMakeFiles/FreeRTOS.dir/flags.make
CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/list.c.obj: ../Thor/lib/FreeRTOS/list.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=G:\git\Microcontrollers\Thor_STM32\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/list.c.obj"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\list.c.obj   -c G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\list.c

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/list.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/list.c.i"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\list.c > CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\list.c.i

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/list.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/list.c.s"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\list.c -o CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\list.c.s

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/queue.c.obj: CMakeFiles/FreeRTOS.dir/flags.make
CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/queue.c.obj: ../Thor/lib/FreeRTOS/queue.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=G:\git\Microcontrollers\Thor_STM32\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/queue.c.obj"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\queue.c.obj   -c G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\queue.c

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/queue.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/queue.c.i"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\queue.c > CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\queue.c.i

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/queue.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/queue.c.s"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\queue.c -o CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\queue.c.s

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/stream_buffer.c.obj: CMakeFiles/FreeRTOS.dir/flags.make
CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/stream_buffer.c.obj: ../Thor/lib/FreeRTOS/stream_buffer.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=G:\git\Microcontrollers\Thor_STM32\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/stream_buffer.c.obj"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\stream_buffer.c.obj   -c G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\stream_buffer.c

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/stream_buffer.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/stream_buffer.c.i"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\stream_buffer.c > CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\stream_buffer.c.i

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/stream_buffer.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/stream_buffer.c.s"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\stream_buffer.c -o CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\stream_buffer.c.s

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/tasks.c.obj: CMakeFiles/FreeRTOS.dir/flags.make
CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/tasks.c.obj: ../Thor/lib/FreeRTOS/tasks.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=G:\git\Microcontrollers\Thor_STM32\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building C object CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/tasks.c.obj"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\tasks.c.obj   -c G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\tasks.c

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/tasks.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/tasks.c.i"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\tasks.c > CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\tasks.c.i

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/tasks.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/tasks.c.s"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\tasks.c -o CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\tasks.c.s

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/timers.c.obj: CMakeFiles/FreeRTOS.dir/flags.make
CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/timers.c.obj: ../Thor/lib/FreeRTOS/timers.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=G:\git\Microcontrollers\Thor_STM32\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building C object CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/timers.c.obj"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\timers.c.obj   -c G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\timers.c

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/timers.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/timers.c.i"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\timers.c > CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\timers.c.i

CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/timers.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/timers.c.s"
	G:\SysGCC\GNUToolsARMEmbedded\7_2017_q4_major\bin\arm-none-eabi-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S G:\git\Microcontrollers\Thor_STM32\Thor\lib\FreeRTOS\timers.c -o CMakeFiles\FreeRTOS.dir\Thor\lib\FreeRTOS\timers.c.s

# Object files for target FreeRTOS
FreeRTOS_OBJECTS = \
"CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/GCC/ARM_CM4F/port.c.obj" \
"CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/Common/mpu_wrappers.c.obj" \
"CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/MemMang/heap_4.c.obj" \
"CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/croutine.c.obj" \
"CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/event_groups.c.obj" \
"CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/list.c.obj" \
"CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/queue.c.obj" \
"CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/stream_buffer.c.obj" \
"CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/tasks.c.obj" \
"CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/timers.c.obj"

# External object files for target FreeRTOS
FreeRTOS_EXTERNAL_OBJECTS =

libFreeRTOS_rel.a: CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/GCC/ARM_CM4F/port.c.obj
libFreeRTOS_rel.a: CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/Common/mpu_wrappers.c.obj
libFreeRTOS_rel.a: CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/portable/MemMang/heap_4.c.obj
libFreeRTOS_rel.a: CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/croutine.c.obj
libFreeRTOS_rel.a: CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/event_groups.c.obj
libFreeRTOS_rel.a: CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/list.c.obj
libFreeRTOS_rel.a: CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/queue.c.obj
libFreeRTOS_rel.a: CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/stream_buffer.c.obj
libFreeRTOS_rel.a: CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/tasks.c.obj
libFreeRTOS_rel.a: CMakeFiles/FreeRTOS.dir/Thor/lib/FreeRTOS/timers.c.obj
libFreeRTOS_rel.a: CMakeFiles/FreeRTOS.dir/build.make
libFreeRTOS_rel.a: CMakeFiles/FreeRTOS.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=G:\git\Microcontrollers\Thor_STM32\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Linking C static library libFreeRTOS_rel.a"
	$(CMAKE_COMMAND) -P CMakeFiles\FreeRTOS.dir\cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\FreeRTOS.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/FreeRTOS.dir/build: libFreeRTOS_rel.a

.PHONY : CMakeFiles/FreeRTOS.dir/build

CMakeFiles/FreeRTOS.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\FreeRTOS.dir\cmake_clean.cmake
.PHONY : CMakeFiles/FreeRTOS.dir/clean

CMakeFiles/FreeRTOS.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" G:\git\Microcontrollers\Thor_STM32 G:\git\Microcontrollers\Thor_STM32 G:\git\Microcontrollers\Thor_STM32\build G:\git\Microcontrollers\Thor_STM32\build G:\git\Microcontrollers\Thor_STM32\build\CMakeFiles\FreeRTOS.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/FreeRTOS.dir/depend
