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
include CMakeFiles/FirstStep_mb_VISION_0x40.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/FirstStep_mb_VISION_0x40.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/FirstStep_mb_VISION_0x40.dir/flags.make

CMakeFiles/FirstStep_mb_VISION_0x40.dir/FirstStep/main.cpp.obj: CMakeFiles/FirstStep_mb_VISION_0x40.dir/flags.make
CMakeFiles/FirstStep_mb_VISION_0x40.dir/FirstStep/main.cpp.obj: ../FirstStep/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nathan/SpiritSDK/SDK/examples_RML/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/FirstStep_mb_VISION_0x40.dir/FirstStep/main.cpp.obj"
	/home/nathan/ghost_robotics/arm_toolchain/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FirstStep_mb_VISION_0x40.dir/FirstStep/main.cpp.obj -c /home/nathan/SpiritSDK/SDK/examples_RML/FirstStep/main.cpp

CMakeFiles/FirstStep_mb_VISION_0x40.dir/FirstStep/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FirstStep_mb_VISION_0x40.dir/FirstStep/main.cpp.i"
	/home/nathan/ghost_robotics/arm_toolchain/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nathan/SpiritSDK/SDK/examples_RML/FirstStep/main.cpp > CMakeFiles/FirstStep_mb_VISION_0x40.dir/FirstStep/main.cpp.i

CMakeFiles/FirstStep_mb_VISION_0x40.dir/FirstStep/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FirstStep_mb_VISION_0x40.dir/FirstStep/main.cpp.s"
	/home/nathan/ghost_robotics/arm_toolchain/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nathan/SpiritSDK/SDK/examples_RML/FirstStep/main.cpp -o CMakeFiles/FirstStep_mb_VISION_0x40.dir/FirstStep/main.cpp.s

# Object files for target FirstStep_mb_VISION_0x40
FirstStep_mb_VISION_0x40_OBJECTS = \
"CMakeFiles/FirstStep_mb_VISION_0x40.dir/FirstStep/main.cpp.obj"

# External object files for target FirstStep_mb_VISION_0x40
FirstStep_mb_VISION_0x40_EXTERNAL_OBJECTS =

FirstStep_mb_VISION_0x40: CMakeFiles/FirstStep_mb_VISION_0x40.dir/FirstStep/main.cpp.obj
FirstStep_mb_VISION_0x40: CMakeFiles/FirstStep_mb_VISION_0x40.dir/build.make
FirstStep_mb_VISION_0x40: CMakeFiles/FirstStep_mb_VISION_0x40.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nathan/SpiritSDK/SDK/examples_RML/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable FirstStep_mb_VISION_0x40"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FirstStep_mb_VISION_0x40.dir/link.txt --verbose=$(VERBOSE)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold Post-build
	arm-none-eabi-size FirstStep_mb_VISION_0x40
	arm-none-eabi-objcopy -O ihex FirstStep_mb_VISION_0x40 FirstStep_mb_VISION_0x40.bin
	arm-none-eabi-objcopy -O binary -S FirstStep_mb_VISION_0x40 FirstStep_mb_VISION_0x40.bin

# Rule to build all files generated by this target.
CMakeFiles/FirstStep_mb_VISION_0x40.dir/build: FirstStep_mb_VISION_0x40

.PHONY : CMakeFiles/FirstStep_mb_VISION_0x40.dir/build

CMakeFiles/FirstStep_mb_VISION_0x40.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/FirstStep_mb_VISION_0x40.dir/cmake_clean.cmake
.PHONY : CMakeFiles/FirstStep_mb_VISION_0x40.dir/clean

CMakeFiles/FirstStep_mb_VISION_0x40.dir/depend:
	cd /home/nathan/SpiritSDK/SDK/examples_RML/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nathan/SpiritSDK/SDK/examples_RML /home/nathan/SpiritSDK/SDK/examples_RML /home/nathan/SpiritSDK/SDK/examples_RML/build /home/nathan/SpiritSDK/SDK/examples_RML/build /home/nathan/SpiritSDK/SDK/examples_RML/build/CMakeFiles/FirstStep_mb_VISION_0x40.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/FirstStep_mb_VISION_0x40.dir/depend

