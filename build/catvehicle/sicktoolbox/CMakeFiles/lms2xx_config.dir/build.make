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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/dfq/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dfq/catkin_ws/build

# Include any dependencies generated for this target.
include catvehicle/sicktoolbox/CMakeFiles/lms2xx_config.dir/depend.make

# Include the progress variables for this target.
include catvehicle/sicktoolbox/CMakeFiles/lms2xx_config.dir/progress.make

# Include the compile flags for this target's objects.
include catvehicle/sicktoolbox/CMakeFiles/lms2xx_config.dir/flags.make

catvehicle/sicktoolbox/CMakeFiles/lms2xx_config.dir/c++/examples/lms2xx/lms2xx_config/src/main.cc.o: catvehicle/sicktoolbox/CMakeFiles/lms2xx_config.dir/flags.make
catvehicle/sicktoolbox/CMakeFiles/lms2xx_config.dir/c++/examples/lms2xx/lms2xx_config/src/main.cc.o: /home/dfq/catkin_ws/src/catvehicle/sicktoolbox/c++/examples/lms2xx/lms2xx_config/src/main.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dfq/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object catvehicle/sicktoolbox/CMakeFiles/lms2xx_config.dir/c++/examples/lms2xx/lms2xx_config/src/main.cc.o"
	cd /home/dfq/catkin_ws/build/catvehicle/sicktoolbox && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lms2xx_config.dir/c++/examples/lms2xx/lms2xx_config/src/main.cc.o -c /home/dfq/catkin_ws/src/catvehicle/sicktoolbox/c++/examples/lms2xx/lms2xx_config/src/main.cc

catvehicle/sicktoolbox/CMakeFiles/lms2xx_config.dir/c++/examples/lms2xx/lms2xx_config/src/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lms2xx_config.dir/c++/examples/lms2xx/lms2xx_config/src/main.cc.i"
	cd /home/dfq/catkin_ws/build/catvehicle/sicktoolbox && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dfq/catkin_ws/src/catvehicle/sicktoolbox/c++/examples/lms2xx/lms2xx_config/src/main.cc > CMakeFiles/lms2xx_config.dir/c++/examples/lms2xx/lms2xx_config/src/main.cc.i

catvehicle/sicktoolbox/CMakeFiles/lms2xx_config.dir/c++/examples/lms2xx/lms2xx_config/src/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lms2xx_config.dir/c++/examples/lms2xx/lms2xx_config/src/main.cc.s"
	cd /home/dfq/catkin_ws/build/catvehicle/sicktoolbox && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dfq/catkin_ws/src/catvehicle/sicktoolbox/c++/examples/lms2xx/lms2xx_config/src/main.cc -o CMakeFiles/lms2xx_config.dir/c++/examples/lms2xx/lms2xx_config/src/main.cc.s

# Object files for target lms2xx_config
lms2xx_config_OBJECTS = \
"CMakeFiles/lms2xx_config.dir/c++/examples/lms2xx/lms2xx_config/src/main.cc.o"

# External object files for target lms2xx_config
lms2xx_config_EXTERNAL_OBJECTS =

/home/dfq/catkin_ws/devel/lib/sicktoolbox/lms2xx_config: catvehicle/sicktoolbox/CMakeFiles/lms2xx_config.dir/c++/examples/lms2xx/lms2xx_config/src/main.cc.o
/home/dfq/catkin_ws/devel/lib/sicktoolbox/lms2xx_config: catvehicle/sicktoolbox/CMakeFiles/lms2xx_config.dir/build.make
/home/dfq/catkin_ws/devel/lib/sicktoolbox/lms2xx_config: /home/dfq/catkin_ws/devel/lib/libSickLMS2xx.so
/home/dfq/catkin_ws/devel/lib/sicktoolbox/lms2xx_config: catvehicle/sicktoolbox/CMakeFiles/lms2xx_config.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dfq/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/dfq/catkin_ws/devel/lib/sicktoolbox/lms2xx_config"
	cd /home/dfq/catkin_ws/build/catvehicle/sicktoolbox && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lms2xx_config.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
catvehicle/sicktoolbox/CMakeFiles/lms2xx_config.dir/build: /home/dfq/catkin_ws/devel/lib/sicktoolbox/lms2xx_config

.PHONY : catvehicle/sicktoolbox/CMakeFiles/lms2xx_config.dir/build

catvehicle/sicktoolbox/CMakeFiles/lms2xx_config.dir/clean:
	cd /home/dfq/catkin_ws/build/catvehicle/sicktoolbox && $(CMAKE_COMMAND) -P CMakeFiles/lms2xx_config.dir/cmake_clean.cmake
.PHONY : catvehicle/sicktoolbox/CMakeFiles/lms2xx_config.dir/clean

catvehicle/sicktoolbox/CMakeFiles/lms2xx_config.dir/depend:
	cd /home/dfq/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dfq/catkin_ws/src /home/dfq/catkin_ws/src/catvehicle/sicktoolbox /home/dfq/catkin_ws/build /home/dfq/catkin_ws/build/catvehicle/sicktoolbox /home/dfq/catkin_ws/build/catvehicle/sicktoolbox/CMakeFiles/lms2xx_config.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : catvehicle/sicktoolbox/CMakeFiles/lms2xx_config.dir/depend

