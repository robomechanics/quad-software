# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/nathan/SpiritSDK/SDK/examples_RML

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nathan/SpiritSDK/SDK/examples_RML/build

# Include any dependencies generated for this target.
include CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/flags.make

CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/MAVLinkSpeedTest/main.cpp.obj: CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/flags.make
CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/MAVLinkSpeedTest/main.cpp.obj: ../MAVLinkSpeedTest/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nathan/SpiritSDK/SDK/examples_RML/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/MAVLinkSpeedTest/main.cpp.obj"
	/home/nathan/ghost_robotics/arm_toolchain/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/MAVLinkSpeedTest/main.cpp.obj -c /home/nathan/SpiritSDK/SDK/examples_RML/MAVLinkSpeedTest/main.cpp

CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/MAVLinkSpeedTest/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/MAVLinkSpeedTest/main.cpp.i"
	/home/nathan/ghost_robotics/arm_toolchain/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nathan/SpiritSDK/SDK/examples_RML/MAVLinkSpeedTest/main.cpp > CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/MAVLinkSpeedTest/main.cpp.i

CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/MAVLinkSpeedTest/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/MAVLinkSpeedTest/main.cpp.s"
	/home/nathan/ghost_robotics/arm_toolchain/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nathan/SpiritSDK/SDK/examples_RML/MAVLinkSpeedTest/main.cpp -o CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/MAVLinkSpeedTest/main.cpp.s

# Object files for target MAVLinkSpeedTest_mb_VISION_0x37
MAVLinkSpeedTest_mb_VISION_0x37_OBJECTS = \
"CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/MAVLinkSpeedTest/main.cpp.obj"

# External object files for target MAVLinkSpeedTest_mb_VISION_0x37
MAVLinkSpeedTest_mb_VISION_0x37_EXTERNAL_OBJECTS =

MAVLinkSpeedTest_mb_VISION_0x37: CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/MAVLinkSpeedTest/main.cpp.obj
MAVLinkSpeedTest_mb_VISION_0x37: CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/build.make
MAVLinkSpeedTest_mb_VISION_0x37: CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nathan/SpiritSDK/SDK/examples_RML/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable MAVLinkSpeedTest_mb_VISION_0x37"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/link.txt --verbose=$(VERBOSE)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold Post-build
	arm-none-eabi-size MAVLinkSpeedTest_mb_VISION_0x37
	arm-none-eabi-objcopy -O ihex MAVLinkSpeedTest_mb_VISION_0x37 MAVLinkSpeedTest_mb_VISION_0x37.bin
	arm-none-eabi-objcopy -O binary -S MAVLinkSpeedTest_mb_VISION_0x37 MAVLinkSpeedTest_mb_VISION_0x37.bin

# Rule to build all files generated by this target.
CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/build: MAVLinkSpeedTest_mb_VISION_0x37

.PHONY : CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/build

CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/cmake_clean.cmake
.PHONY : CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/clean

CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/depend:
	cd /home/nathan/SpiritSDK/SDK/examples_RML/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nathan/SpiritSDK/SDK/examples_RML /home/nathan/SpiritSDK/SDK/examples_RML /home/nathan/SpiritSDK/SDK/examples_RML/build /home/nathan/SpiritSDK/SDK/examples_RML/build /home/nathan/SpiritSDK/SDK/examples_RML/build/CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/MAVLinkSpeedTest_mb_VISION_0x37.dir/depend

