# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/intel/workspace/test_template/cmake_empty_project

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/intel/workspace/test_template/cmake_empty_project/build

# Include any dependencies generated for this target.
include CMakeFiles/test_script.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_script.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_script.dir/flags.make

CMakeFiles/test_script.dir/src/main.cpp.o: CMakeFiles/test_script.dir/flags.make
CMakeFiles/test_script.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/intel/workspace/test_template/cmake_empty_project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_script.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_script.dir/src/main.cpp.o -c /home/intel/workspace/test_template/cmake_empty_project/src/main.cpp

CMakeFiles/test_script.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_script.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/intel/workspace/test_template/cmake_empty_project/src/main.cpp > CMakeFiles/test_script.dir/src/main.cpp.i

CMakeFiles/test_script.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_script.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/intel/workspace/test_template/cmake_empty_project/src/main.cpp -o CMakeFiles/test_script.dir/src/main.cpp.s

CMakeFiles/test_script.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/test_script.dir/src/main.cpp.o.requires

CMakeFiles/test_script.dir/src/main.cpp.o.provides: CMakeFiles/test_script.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_script.dir/build.make CMakeFiles/test_script.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/test_script.dir/src/main.cpp.o.provides

CMakeFiles/test_script.dir/src/main.cpp.o.provides.build: CMakeFiles/test_script.dir/src/main.cpp.o


# Object files for target test_script
test_script_OBJECTS = \
"CMakeFiles/test_script.dir/src/main.cpp.o"

# External object files for target test_script
test_script_EXTERNAL_OBJECTS =

test_script: CMakeFiles/test_script.dir/src/main.cpp.o
test_script: CMakeFiles/test_script.dir/build.make
test_script: libtest_library.a
test_script: CMakeFiles/test_script.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/intel/workspace/test_template/cmake_empty_project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_script"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_script.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_script.dir/build: test_script

.PHONY : CMakeFiles/test_script.dir/build

CMakeFiles/test_script.dir/requires: CMakeFiles/test_script.dir/src/main.cpp.o.requires

.PHONY : CMakeFiles/test_script.dir/requires

CMakeFiles/test_script.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_script.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_script.dir/clean

CMakeFiles/test_script.dir/depend:
	cd /home/intel/workspace/test_template/cmake_empty_project/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/intel/workspace/test_template/cmake_empty_project /home/intel/workspace/test_template/cmake_empty_project /home/intel/workspace/test_template/cmake_empty_project/build /home/intel/workspace/test_template/cmake_empty_project/build /home/intel/workspace/test_template/cmake_empty_project/build/CMakeFiles/test_script.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_script.dir/depend
