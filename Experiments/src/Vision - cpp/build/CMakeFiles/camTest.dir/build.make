# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = "/home/nascimento/Projects/Task-Space-Control-Vision/Task-Space-Control-Vision/Experiments/src/Vision - cpp"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/nascimento/Projects/Task-Space-Control-Vision/Task-Space-Control-Vision/Experiments/src/Vision - cpp/build"

# Include any dependencies generated for this target.
include CMakeFiles/camTest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/camTest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/camTest.dir/flags.make

CMakeFiles/camTest.dir/camTest.c.o: CMakeFiles/camTest.dir/flags.make
CMakeFiles/camTest.dir/camTest.c.o: ../camTest.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/nascimento/Projects/Task-Space-Control-Vision/Task-Space-Control-Vision/Experiments/src/Vision - cpp/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/camTest.dir/camTest.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/camTest.dir/camTest.c.o   -c "/home/nascimento/Projects/Task-Space-Control-Vision/Task-Space-Control-Vision/Experiments/src/Vision - cpp/camTest.c"

CMakeFiles/camTest.dir/camTest.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/camTest.dir/camTest.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/home/nascimento/Projects/Task-Space-Control-Vision/Task-Space-Control-Vision/Experiments/src/Vision - cpp/camTest.c" > CMakeFiles/camTest.dir/camTest.c.i

CMakeFiles/camTest.dir/camTest.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/camTest.dir/camTest.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/home/nascimento/Projects/Task-Space-Control-Vision/Task-Space-Control-Vision/Experiments/src/Vision - cpp/camTest.c" -o CMakeFiles/camTest.dir/camTest.c.s

CMakeFiles/camTest.dir/camTest.c.o.requires:

.PHONY : CMakeFiles/camTest.dir/camTest.c.o.requires

CMakeFiles/camTest.dir/camTest.c.o.provides: CMakeFiles/camTest.dir/camTest.c.o.requires
	$(MAKE) -f CMakeFiles/camTest.dir/build.make CMakeFiles/camTest.dir/camTest.c.o.provides.build
.PHONY : CMakeFiles/camTest.dir/camTest.c.o.provides

CMakeFiles/camTest.dir/camTest.c.o.provides.build: CMakeFiles/camTest.dir/camTest.c.o


# Object files for target camTest
camTest_OBJECTS = \
"CMakeFiles/camTest.dir/camTest.c.o"

# External object files for target camTest
camTest_EXTERNAL_OBJECTS =

camTest: CMakeFiles/camTest.dir/camTest.c.o
camTest: CMakeFiles/camTest.dir/build.make
camTest: CMakeFiles/camTest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/nascimento/Projects/Task-Space-Control-Vision/Task-Space-Control-Vision/Experiments/src/Vision - cpp/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable camTest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/camTest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/camTest.dir/build: camTest

.PHONY : CMakeFiles/camTest.dir/build

CMakeFiles/camTest.dir/requires: CMakeFiles/camTest.dir/camTest.c.o.requires

.PHONY : CMakeFiles/camTest.dir/requires

CMakeFiles/camTest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/camTest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/camTest.dir/clean

CMakeFiles/camTest.dir/depend:
	cd "/home/nascimento/Projects/Task-Space-Control-Vision/Task-Space-Control-Vision/Experiments/src/Vision - cpp/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/nascimento/Projects/Task-Space-Control-Vision/Task-Space-Control-Vision/Experiments/src/Vision - cpp" "/home/nascimento/Projects/Task-Space-Control-Vision/Task-Space-Control-Vision/Experiments/src/Vision - cpp" "/home/nascimento/Projects/Task-Space-Control-Vision/Task-Space-Control-Vision/Experiments/src/Vision - cpp/build" "/home/nascimento/Projects/Task-Space-Control-Vision/Task-Space-Control-Vision/Experiments/src/Vision - cpp/build" "/home/nascimento/Projects/Task-Space-Control-Vision/Task-Space-Control-Vision/Experiments/src/Vision - cpp/build/CMakeFiles/camTest.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/camTest.dir/depend

