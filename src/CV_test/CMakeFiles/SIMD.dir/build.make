# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ws/src/CV_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ws/src/CV_test

# Include any dependencies generated for this target.
include CMakeFiles/SIMD.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/SIMD.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/SIMD.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SIMD.dir/flags.make

CMakeFiles/SIMD.dir/simd.cpp.o: CMakeFiles/SIMD.dir/flags.make
CMakeFiles/SIMD.dir/simd.cpp.o: simd.cpp
CMakeFiles/SIMD.dir/simd.cpp.o: CMakeFiles/SIMD.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ws/src/CV_test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/SIMD.dir/simd.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/SIMD.dir/simd.cpp.o -MF CMakeFiles/SIMD.dir/simd.cpp.o.d -o CMakeFiles/SIMD.dir/simd.cpp.o -c /home/ws/src/CV_test/simd.cpp

CMakeFiles/SIMD.dir/simd.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SIMD.dir/simd.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ws/src/CV_test/simd.cpp > CMakeFiles/SIMD.dir/simd.cpp.i

CMakeFiles/SIMD.dir/simd.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SIMD.dir/simd.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ws/src/CV_test/simd.cpp -o CMakeFiles/SIMD.dir/simd.cpp.s

# Object files for target SIMD
SIMD_OBJECTS = \
"CMakeFiles/SIMD.dir/simd.cpp.o"

# External object files for target SIMD
SIMD_EXTERNAL_OBJECTS =

SIMD: CMakeFiles/SIMD.dir/simd.cpp.o
SIMD: CMakeFiles/SIMD.dir/build.make
SIMD: /usr/local/lib/libopencv_gapi.so.4.9.0
SIMD: /usr/local/lib/libopencv_highgui.so.4.9.0
SIMD: /usr/local/lib/libopencv_ml.so.4.9.0
SIMD: /usr/local/lib/libopencv_objdetect.so.4.9.0
SIMD: /usr/local/lib/libopencv_photo.so.4.9.0
SIMD: /usr/local/lib/libopencv_stitching.so.4.9.0
SIMD: /usr/local/lib/libopencv_video.so.4.9.0
SIMD: /usr/local/lib/libopencv_videoio.so.4.9.0
SIMD: /usr/local/lib/libopencv_imgcodecs.so.4.9.0
SIMD: /usr/local/lib/libopencv_dnn.so.4.9.0
SIMD: /usr/local/lib/libopencv_calib3d.so.4.9.0
SIMD: /usr/local/lib/libopencv_features2d.so.4.9.0
SIMD: /usr/local/lib/libopencv_flann.so.4.9.0
SIMD: /usr/local/lib/libopencv_imgproc.so.4.9.0
SIMD: /usr/local/lib/libopencv_core.so.4.9.0
SIMD: CMakeFiles/SIMD.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ws/src/CV_test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable SIMD"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SIMD.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SIMD.dir/build: SIMD
.PHONY : CMakeFiles/SIMD.dir/build

CMakeFiles/SIMD.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SIMD.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SIMD.dir/clean

CMakeFiles/SIMD.dir/depend:
	cd /home/ws/src/CV_test && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ws/src/CV_test /home/ws/src/CV_test /home/ws/src/CV_test /home/ws/src/CV_test /home/ws/src/CV_test/CMakeFiles/SIMD.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SIMD.dir/depend

