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
CMAKE_SOURCE_DIR = /Users/lipenghua/GitHub/SLAMbook/Ch9

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/lipenghua/GitHub/SLAMbook/Ch9/build

# Include any dependencies generated for this target.
include src/CMakeFiles/myslam.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/myslam.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/myslam.dir/flags.make

src/CMakeFiles/myslam.dir/frame.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/frame.cpp.o: ../src/frame.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/lipenghua/GitHub/SLAMbook/Ch9/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/myslam.dir/frame.cpp.o"
	cd /Users/lipenghua/GitHub/SLAMbook/Ch9/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/frame.cpp.o -c /Users/lipenghua/GitHub/SLAMbook/Ch9/src/frame.cpp

src/CMakeFiles/myslam.dir/frame.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/frame.cpp.i"
	cd /Users/lipenghua/GitHub/SLAMbook/Ch9/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/lipenghua/GitHub/SLAMbook/Ch9/src/frame.cpp > CMakeFiles/myslam.dir/frame.cpp.i

src/CMakeFiles/myslam.dir/frame.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/frame.cpp.s"
	cd /Users/lipenghua/GitHub/SLAMbook/Ch9/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/lipenghua/GitHub/SLAMbook/Ch9/src/frame.cpp -o CMakeFiles/myslam.dir/frame.cpp.s

src/CMakeFiles/myslam.dir/frame.cpp.o.requires:

.PHONY : src/CMakeFiles/myslam.dir/frame.cpp.o.requires

src/CMakeFiles/myslam.dir/frame.cpp.o.provides: src/CMakeFiles/myslam.dir/frame.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/myslam.dir/build.make src/CMakeFiles/myslam.dir/frame.cpp.o.provides.build
.PHONY : src/CMakeFiles/myslam.dir/frame.cpp.o.provides

src/CMakeFiles/myslam.dir/frame.cpp.o.provides.build: src/CMakeFiles/myslam.dir/frame.cpp.o


src/CMakeFiles/myslam.dir/mappoint.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/mappoint.cpp.o: ../src/mappoint.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/lipenghua/GitHub/SLAMbook/Ch9/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/myslam.dir/mappoint.cpp.o"
	cd /Users/lipenghua/GitHub/SLAMbook/Ch9/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/mappoint.cpp.o -c /Users/lipenghua/GitHub/SLAMbook/Ch9/src/mappoint.cpp

src/CMakeFiles/myslam.dir/mappoint.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/mappoint.cpp.i"
	cd /Users/lipenghua/GitHub/SLAMbook/Ch9/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/lipenghua/GitHub/SLAMbook/Ch9/src/mappoint.cpp > CMakeFiles/myslam.dir/mappoint.cpp.i

src/CMakeFiles/myslam.dir/mappoint.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/mappoint.cpp.s"
	cd /Users/lipenghua/GitHub/SLAMbook/Ch9/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/lipenghua/GitHub/SLAMbook/Ch9/src/mappoint.cpp -o CMakeFiles/myslam.dir/mappoint.cpp.s

src/CMakeFiles/myslam.dir/mappoint.cpp.o.requires:

.PHONY : src/CMakeFiles/myslam.dir/mappoint.cpp.o.requires

src/CMakeFiles/myslam.dir/mappoint.cpp.o.provides: src/CMakeFiles/myslam.dir/mappoint.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/myslam.dir/build.make src/CMakeFiles/myslam.dir/mappoint.cpp.o.provides.build
.PHONY : src/CMakeFiles/myslam.dir/mappoint.cpp.o.provides

src/CMakeFiles/myslam.dir/mappoint.cpp.o.provides.build: src/CMakeFiles/myslam.dir/mappoint.cpp.o


src/CMakeFiles/myslam.dir/map.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/map.cpp.o: ../src/map.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/lipenghua/GitHub/SLAMbook/Ch9/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/CMakeFiles/myslam.dir/map.cpp.o"
	cd /Users/lipenghua/GitHub/SLAMbook/Ch9/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/map.cpp.o -c /Users/lipenghua/GitHub/SLAMbook/Ch9/src/map.cpp

src/CMakeFiles/myslam.dir/map.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/map.cpp.i"
	cd /Users/lipenghua/GitHub/SLAMbook/Ch9/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/lipenghua/GitHub/SLAMbook/Ch9/src/map.cpp > CMakeFiles/myslam.dir/map.cpp.i

src/CMakeFiles/myslam.dir/map.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/map.cpp.s"
	cd /Users/lipenghua/GitHub/SLAMbook/Ch9/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/lipenghua/GitHub/SLAMbook/Ch9/src/map.cpp -o CMakeFiles/myslam.dir/map.cpp.s

src/CMakeFiles/myslam.dir/map.cpp.o.requires:

.PHONY : src/CMakeFiles/myslam.dir/map.cpp.o.requires

src/CMakeFiles/myslam.dir/map.cpp.o.provides: src/CMakeFiles/myslam.dir/map.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/myslam.dir/build.make src/CMakeFiles/myslam.dir/map.cpp.o.provides.build
.PHONY : src/CMakeFiles/myslam.dir/map.cpp.o.provides

src/CMakeFiles/myslam.dir/map.cpp.o.provides.build: src/CMakeFiles/myslam.dir/map.cpp.o


src/CMakeFiles/myslam.dir/camera.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/camera.cpp.o: ../src/camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/lipenghua/GitHub/SLAMbook/Ch9/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/CMakeFiles/myslam.dir/camera.cpp.o"
	cd /Users/lipenghua/GitHub/SLAMbook/Ch9/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/camera.cpp.o -c /Users/lipenghua/GitHub/SLAMbook/Ch9/src/camera.cpp

src/CMakeFiles/myslam.dir/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/camera.cpp.i"
	cd /Users/lipenghua/GitHub/SLAMbook/Ch9/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/lipenghua/GitHub/SLAMbook/Ch9/src/camera.cpp > CMakeFiles/myslam.dir/camera.cpp.i

src/CMakeFiles/myslam.dir/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/camera.cpp.s"
	cd /Users/lipenghua/GitHub/SLAMbook/Ch9/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/lipenghua/GitHub/SLAMbook/Ch9/src/camera.cpp -o CMakeFiles/myslam.dir/camera.cpp.s

src/CMakeFiles/myslam.dir/camera.cpp.o.requires:

.PHONY : src/CMakeFiles/myslam.dir/camera.cpp.o.requires

src/CMakeFiles/myslam.dir/camera.cpp.o.provides: src/CMakeFiles/myslam.dir/camera.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/myslam.dir/build.make src/CMakeFiles/myslam.dir/camera.cpp.o.provides.build
.PHONY : src/CMakeFiles/myslam.dir/camera.cpp.o.provides

src/CMakeFiles/myslam.dir/camera.cpp.o.provides.build: src/CMakeFiles/myslam.dir/camera.cpp.o


src/CMakeFiles/myslam.dir/config.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/config.cpp.o: ../src/config.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/lipenghua/GitHub/SLAMbook/Ch9/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/CMakeFiles/myslam.dir/config.cpp.o"
	cd /Users/lipenghua/GitHub/SLAMbook/Ch9/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/config.cpp.o -c /Users/lipenghua/GitHub/SLAMbook/Ch9/src/config.cpp

src/CMakeFiles/myslam.dir/config.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/config.cpp.i"
	cd /Users/lipenghua/GitHub/SLAMbook/Ch9/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/lipenghua/GitHub/SLAMbook/Ch9/src/config.cpp > CMakeFiles/myslam.dir/config.cpp.i

src/CMakeFiles/myslam.dir/config.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/config.cpp.s"
	cd /Users/lipenghua/GitHub/SLAMbook/Ch9/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/lipenghua/GitHub/SLAMbook/Ch9/src/config.cpp -o CMakeFiles/myslam.dir/config.cpp.s

src/CMakeFiles/myslam.dir/config.cpp.o.requires:

.PHONY : src/CMakeFiles/myslam.dir/config.cpp.o.requires

src/CMakeFiles/myslam.dir/config.cpp.o.provides: src/CMakeFiles/myslam.dir/config.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/myslam.dir/build.make src/CMakeFiles/myslam.dir/config.cpp.o.provides.build
.PHONY : src/CMakeFiles/myslam.dir/config.cpp.o.provides

src/CMakeFiles/myslam.dir/config.cpp.o.provides.build: src/CMakeFiles/myslam.dir/config.cpp.o


# Object files for target myslam
myslam_OBJECTS = \
"CMakeFiles/myslam.dir/frame.cpp.o" \
"CMakeFiles/myslam.dir/mappoint.cpp.o" \
"CMakeFiles/myslam.dir/map.cpp.o" \
"CMakeFiles/myslam.dir/camera.cpp.o" \
"CMakeFiles/myslam.dir/config.cpp.o"

# External object files for target myslam
myslam_EXTERNAL_OBJECTS =

../lib/libmyslam.dylib: src/CMakeFiles/myslam.dir/frame.cpp.o
../lib/libmyslam.dylib: src/CMakeFiles/myslam.dir/mappoint.cpp.o
../lib/libmyslam.dylib: src/CMakeFiles/myslam.dir/map.cpp.o
../lib/libmyslam.dylib: src/CMakeFiles/myslam.dir/camera.cpp.o
../lib/libmyslam.dylib: src/CMakeFiles/myslam.dir/config.cpp.o
../lib/libmyslam.dylib: src/CMakeFiles/myslam.dir/build.make
../lib/libmyslam.dylib: /usr/local/lib/libopencv_stitching.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_superres.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_videostab.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_aruco.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_bgsegm.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_bioinspired.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_ccalib.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_dpm.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_face.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_fuzzy.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_hdf.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_img_hash.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_line_descriptor.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_optflow.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_reg.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_rgbd.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_saliency.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_sfm.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_stereo.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_structured_light.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_surface_matching.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_tracking.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_xfeatures2d.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_ximgproc.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_xobjdetect.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_xphoto.3.4.0.dylib
../lib/libmyslam.dylib: /Users/lipenghua/Sophus/build/libSophus.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_photo.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_datasets.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_plot.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_text.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_dnn.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_ml.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_shape.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_video.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_calib3d.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_features2d.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_highgui.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_videoio.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_viz.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_phase_unwrapping.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_flann.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_imgcodecs.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_objdetect.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_imgproc.3.4.0.dylib
../lib/libmyslam.dylib: /usr/local/lib/libopencv_core.3.4.0.dylib
../lib/libmyslam.dylib: src/CMakeFiles/myslam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/lipenghua/GitHub/SLAMbook/Ch9/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX shared library ../../lib/libmyslam.dylib"
	cd /Users/lipenghua/GitHub/SLAMbook/Ch9/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/myslam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/myslam.dir/build: ../lib/libmyslam.dylib

.PHONY : src/CMakeFiles/myslam.dir/build

src/CMakeFiles/myslam.dir/requires: src/CMakeFiles/myslam.dir/frame.cpp.o.requires
src/CMakeFiles/myslam.dir/requires: src/CMakeFiles/myslam.dir/mappoint.cpp.o.requires
src/CMakeFiles/myslam.dir/requires: src/CMakeFiles/myslam.dir/map.cpp.o.requires
src/CMakeFiles/myslam.dir/requires: src/CMakeFiles/myslam.dir/camera.cpp.o.requires
src/CMakeFiles/myslam.dir/requires: src/CMakeFiles/myslam.dir/config.cpp.o.requires

.PHONY : src/CMakeFiles/myslam.dir/requires

src/CMakeFiles/myslam.dir/clean:
	cd /Users/lipenghua/GitHub/SLAMbook/Ch9/build/src && $(CMAKE_COMMAND) -P CMakeFiles/myslam.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/myslam.dir/clean

src/CMakeFiles/myslam.dir/depend:
	cd /Users/lipenghua/GitHub/SLAMbook/Ch9/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/lipenghua/GitHub/SLAMbook/Ch9 /Users/lipenghua/GitHub/SLAMbook/Ch9/src /Users/lipenghua/GitHub/SLAMbook/Ch9/build /Users/lipenghua/GitHub/SLAMbook/Ch9/build/src /Users/lipenghua/GitHub/SLAMbook/Ch9/build/src/CMakeFiles/myslam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/myslam.dir/depend
