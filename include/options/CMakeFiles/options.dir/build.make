# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/tom/csm-git

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tom/csm-git

# Include any dependencies generated for this target.
include sm/lib/options/CMakeFiles/options.dir/depend.make

# Include the progress variables for this target.
include sm/lib/options/CMakeFiles/options.dir/progress.make

# Include the compile flags for this target's objects.
include sm/lib/options/CMakeFiles/options.dir/flags.make

sm/lib/options/CMakeFiles/options.dir/options.o: sm/lib/options/CMakeFiles/options.dir/flags.make
sm/lib/options/CMakeFiles/options.dir/options.o: sm/lib/options/options.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tom/csm-git/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object sm/lib/options/CMakeFiles/options.dir/options.o"
	cd /home/tom/csm-git/sm/lib/options && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/options.dir/options.o   -c /home/tom/csm-git/sm/lib/options/options.c

sm/lib/options/CMakeFiles/options.dir/options.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/options.dir/options.i"
	cd /home/tom/csm-git/sm/lib/options && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -E /home/tom/csm-git/sm/lib/options/options.c > CMakeFiles/options.dir/options.i

sm/lib/options/CMakeFiles/options.dir/options.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/options.dir/options.s"
	cd /home/tom/csm-git/sm/lib/options && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -S /home/tom/csm-git/sm/lib/options/options.c -o CMakeFiles/options.dir/options.s

sm/lib/options/CMakeFiles/options.dir/options.o.requires:
.PHONY : sm/lib/options/CMakeFiles/options.dir/options.o.requires

sm/lib/options/CMakeFiles/options.dir/options.o.provides: sm/lib/options/CMakeFiles/options.dir/options.o.requires
	$(MAKE) -f sm/lib/options/CMakeFiles/options.dir/build.make sm/lib/options/CMakeFiles/options.dir/options.o.provides.build
.PHONY : sm/lib/options/CMakeFiles/options.dir/options.o.provides

sm/lib/options/CMakeFiles/options.dir/options.o.provides.build: sm/lib/options/CMakeFiles/options.dir/options.o

sm/lib/options/CMakeFiles/options.dir/options_interface.o: sm/lib/options/CMakeFiles/options.dir/flags.make
sm/lib/options/CMakeFiles/options.dir/options_interface.o: sm/lib/options/options_interface.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tom/csm-git/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object sm/lib/options/CMakeFiles/options.dir/options_interface.o"
	cd /home/tom/csm-git/sm/lib/options && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/options.dir/options_interface.o   -c /home/tom/csm-git/sm/lib/options/options_interface.c

sm/lib/options/CMakeFiles/options.dir/options_interface.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/options.dir/options_interface.i"
	cd /home/tom/csm-git/sm/lib/options && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -E /home/tom/csm-git/sm/lib/options/options_interface.c > CMakeFiles/options.dir/options_interface.i

sm/lib/options/CMakeFiles/options.dir/options_interface.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/options.dir/options_interface.s"
	cd /home/tom/csm-git/sm/lib/options && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -S /home/tom/csm-git/sm/lib/options/options_interface.c -o CMakeFiles/options.dir/options_interface.s

sm/lib/options/CMakeFiles/options.dir/options_interface.o.requires:
.PHONY : sm/lib/options/CMakeFiles/options.dir/options_interface.o.requires

sm/lib/options/CMakeFiles/options.dir/options_interface.o.provides: sm/lib/options/CMakeFiles/options.dir/options_interface.o.requires
	$(MAKE) -f sm/lib/options/CMakeFiles/options.dir/build.make sm/lib/options/CMakeFiles/options.dir/options_interface.o.provides.build
.PHONY : sm/lib/options/CMakeFiles/options.dir/options_interface.o.provides

sm/lib/options/CMakeFiles/options.dir/options_interface.o.provides.build: sm/lib/options/CMakeFiles/options.dir/options_interface.o

# Object files for target options
options_OBJECTS = \
"CMakeFiles/options.dir/options.o" \
"CMakeFiles/options.dir/options_interface.o"

# External object files for target options
options_EXTERNAL_OBJECTS =

sm/lib/options/liboptions.a: sm/lib/options/CMakeFiles/options.dir/options.o
sm/lib/options/liboptions.a: sm/lib/options/CMakeFiles/options.dir/options_interface.o
sm/lib/options/liboptions.a: sm/lib/options/CMakeFiles/options.dir/build.make
sm/lib/options/liboptions.a: sm/lib/options/CMakeFiles/options.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking C static library liboptions.a"
	cd /home/tom/csm-git/sm/lib/options && $(CMAKE_COMMAND) -P CMakeFiles/options.dir/cmake_clean_target.cmake
	cd /home/tom/csm-git/sm/lib/options && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/options.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
sm/lib/options/CMakeFiles/options.dir/build: sm/lib/options/liboptions.a
.PHONY : sm/lib/options/CMakeFiles/options.dir/build

sm/lib/options/CMakeFiles/options.dir/requires: sm/lib/options/CMakeFiles/options.dir/options.o.requires
sm/lib/options/CMakeFiles/options.dir/requires: sm/lib/options/CMakeFiles/options.dir/options_interface.o.requires
.PHONY : sm/lib/options/CMakeFiles/options.dir/requires

sm/lib/options/CMakeFiles/options.dir/clean:
	cd /home/tom/csm-git/sm/lib/options && $(CMAKE_COMMAND) -P CMakeFiles/options.dir/cmake_clean.cmake
.PHONY : sm/lib/options/CMakeFiles/options.dir/clean

sm/lib/options/CMakeFiles/options.dir/depend:
	cd /home/tom/csm-git && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tom/csm-git /home/tom/csm-git/sm/lib/options /home/tom/csm-git /home/tom/csm-git/sm/lib/options /home/tom/csm-git/sm/lib/options/CMakeFiles/options.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : sm/lib/options/CMakeFiles/options.dir/depend
