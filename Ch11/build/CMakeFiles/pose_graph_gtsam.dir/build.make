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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.10.1/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.10.1/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/lipenghua/GitHub/SLAMbook/Ch11

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/lipenghua/GitHub/SLAMbook/Ch11/build

# Include any dependencies generated for this target.
include CMakeFiles/pose_graph_gtsam.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pose_graph_gtsam.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pose_graph_gtsam.dir/flags.make

CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.o: CMakeFiles/pose_graph_gtsam.dir/flags.make
CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.o: ../pose_graph_gtsam.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/lipenghua/GitHub/SLAMbook/Ch11/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.o -c /Users/lipenghua/GitHub/SLAMbook/Ch11/pose_graph_gtsam.cpp

CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/lipenghua/GitHub/SLAMbook/Ch11/pose_graph_gtsam.cpp > CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.i

CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/lipenghua/GitHub/SLAMbook/Ch11/pose_graph_gtsam.cpp -o CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.s

CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.o.requires:

.PHONY : CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.o.requires

CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.o.provides: CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.o.requires
	$(MAKE) -f CMakeFiles/pose_graph_gtsam.dir/build.make CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.o.provides.build
.PHONY : CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.o.provides

CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.o.provides.build: CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.o


# Object files for target pose_graph_gtsam
pose_graph_gtsam_OBJECTS = \
"CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.o"

# External object files for target pose_graph_gtsam
pose_graph_gtsam_EXTERNAL_OBJECTS =

pose_graph_gtsam: CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.o
pose_graph_gtsam: CMakeFiles/pose_graph_gtsam.dir/build.make
pose_graph_gtsam: /usr/local/lib/libcholmod.dylib
pose_graph_gtsam: /usr/local/lib/libamd.dylib
pose_graph_gtsam: /usr/local/lib/libcolamd.dylib
pose_graph_gtsam: /usr/local/lib/libcamd.dylib
pose_graph_gtsam: /usr/local/lib/libccolamd.dylib
pose_graph_gtsam: /usr/local/lib/libmetis.dylib
pose_graph_gtsam: /usr/local/lib/libsuitesparseconfig.dylib
pose_graph_gtsam: /usr/local/lib/libgtsam.4.0.0.dylib
pose_graph_gtsam: /usr/local/lib/libboost_serialization-mt.dylib
pose_graph_gtsam: /usr/local/lib/libboost_system-mt.dylib
pose_graph_gtsam: /usr/local/lib/libboost_filesystem-mt.dylib
pose_graph_gtsam: /usr/local/lib/libboost_thread-mt.dylib
pose_graph_gtsam: /usr/local/lib/libboost_date_time-mt.dylib
pose_graph_gtsam: /usr/local/lib/libboost_timer-mt.dylib
pose_graph_gtsam: /usr/local/lib/libboost_chrono-mt.dylib
pose_graph_gtsam: /usr/local/lib/libtbb.dylib
pose_graph_gtsam: /usr/local/lib/libtbbmalloc.dylib
pose_graph_gtsam: /usr/local/lib/libmetis.dylib
pose_graph_gtsam: CMakeFiles/pose_graph_gtsam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/lipenghua/GitHub/SLAMbook/Ch11/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable pose_graph_gtsam"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pose_graph_gtsam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pose_graph_gtsam.dir/build: pose_graph_gtsam

.PHONY : CMakeFiles/pose_graph_gtsam.dir/build

CMakeFiles/pose_graph_gtsam.dir/requires: CMakeFiles/pose_graph_gtsam.dir/pose_graph_gtsam.cpp.o.requires

.PHONY : CMakeFiles/pose_graph_gtsam.dir/requires

CMakeFiles/pose_graph_gtsam.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pose_graph_gtsam.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pose_graph_gtsam.dir/clean

CMakeFiles/pose_graph_gtsam.dir/depend:
	cd /Users/lipenghua/GitHub/SLAMbook/Ch11/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/lipenghua/GitHub/SLAMbook/Ch11 /Users/lipenghua/GitHub/SLAMbook/Ch11 /Users/lipenghua/GitHub/SLAMbook/Ch11/build /Users/lipenghua/GitHub/SLAMbook/Ch11/build /Users/lipenghua/GitHub/SLAMbook/Ch11/build/CMakeFiles/pose_graph_gtsam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pose_graph_gtsam.dir/depend

