# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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
CMAKE_SOURCE_DIR = /home/bec/文档/SolvePnP

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bec/文档/SolvePnP/build

# Include any dependencies generated for this target.
include CMakeFiles/camerareader.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/camerareader.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/camerareader.dir/flags.make

CMakeFiles/camerareader.dir/CameraCalibration.cpp.o: CMakeFiles/camerareader.dir/flags.make
CMakeFiles/camerareader.dir/CameraCalibration.cpp.o: ../CameraCalibration.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bec/文档/SolvePnP/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/camerareader.dir/CameraCalibration.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/camerareader.dir/CameraCalibration.cpp.o -c /home/bec/文档/SolvePnP/CameraCalibration.cpp

CMakeFiles/camerareader.dir/CameraCalibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camerareader.dir/CameraCalibration.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bec/文档/SolvePnP/CameraCalibration.cpp > CMakeFiles/camerareader.dir/CameraCalibration.cpp.i

CMakeFiles/camerareader.dir/CameraCalibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camerareader.dir/CameraCalibration.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bec/文档/SolvePnP/CameraCalibration.cpp -o CMakeFiles/camerareader.dir/CameraCalibration.cpp.s

CMakeFiles/camerareader.dir/CameraCalibration.cpp.o.requires:
.PHONY : CMakeFiles/camerareader.dir/CameraCalibration.cpp.o.requires

CMakeFiles/camerareader.dir/CameraCalibration.cpp.o.provides: CMakeFiles/camerareader.dir/CameraCalibration.cpp.o.requires
	$(MAKE) -f CMakeFiles/camerareader.dir/build.make CMakeFiles/camerareader.dir/CameraCalibration.cpp.o.provides.build
.PHONY : CMakeFiles/camerareader.dir/CameraCalibration.cpp.o.provides

CMakeFiles/camerareader.dir/CameraCalibration.cpp.o.provides.build: CMakeFiles/camerareader.dir/CameraCalibration.cpp.o

CMakeFiles/camerareader.dir/CameraReader.cpp.o: CMakeFiles/camerareader.dir/flags.make
CMakeFiles/camerareader.dir/CameraReader.cpp.o: ../CameraReader.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bec/文档/SolvePnP/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/camerareader.dir/CameraReader.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/camerareader.dir/CameraReader.cpp.o -c /home/bec/文档/SolvePnP/CameraReader.cpp

CMakeFiles/camerareader.dir/CameraReader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camerareader.dir/CameraReader.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bec/文档/SolvePnP/CameraReader.cpp > CMakeFiles/camerareader.dir/CameraReader.cpp.i

CMakeFiles/camerareader.dir/CameraReader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camerareader.dir/CameraReader.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bec/文档/SolvePnP/CameraReader.cpp -o CMakeFiles/camerareader.dir/CameraReader.cpp.s

CMakeFiles/camerareader.dir/CameraReader.cpp.o.requires:
.PHONY : CMakeFiles/camerareader.dir/CameraReader.cpp.o.requires

CMakeFiles/camerareader.dir/CameraReader.cpp.o.provides: CMakeFiles/camerareader.dir/CameraReader.cpp.o.requires
	$(MAKE) -f CMakeFiles/camerareader.dir/build.make CMakeFiles/camerareader.dir/CameraReader.cpp.o.provides.build
.PHONY : CMakeFiles/camerareader.dir/CameraReader.cpp.o.provides

CMakeFiles/camerareader.dir/CameraReader.cpp.o.provides.build: CMakeFiles/camerareader.dir/CameraReader.cpp.o

CMakeFiles/camerareader.dir/GeometryTypes.cpp.o: CMakeFiles/camerareader.dir/flags.make
CMakeFiles/camerareader.dir/GeometryTypes.cpp.o: ../GeometryTypes.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bec/文档/SolvePnP/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/camerareader.dir/GeometryTypes.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/camerareader.dir/GeometryTypes.cpp.o -c /home/bec/文档/SolvePnP/GeometryTypes.cpp

CMakeFiles/camerareader.dir/GeometryTypes.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camerareader.dir/GeometryTypes.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bec/文档/SolvePnP/GeometryTypes.cpp > CMakeFiles/camerareader.dir/GeometryTypes.cpp.i

CMakeFiles/camerareader.dir/GeometryTypes.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camerareader.dir/GeometryTypes.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bec/文档/SolvePnP/GeometryTypes.cpp -o CMakeFiles/camerareader.dir/GeometryTypes.cpp.s

CMakeFiles/camerareader.dir/GeometryTypes.cpp.o.requires:
.PHONY : CMakeFiles/camerareader.dir/GeometryTypes.cpp.o.requires

CMakeFiles/camerareader.dir/GeometryTypes.cpp.o.provides: CMakeFiles/camerareader.dir/GeometryTypes.cpp.o.requires
	$(MAKE) -f CMakeFiles/camerareader.dir/build.make CMakeFiles/camerareader.dir/GeometryTypes.cpp.o.provides.build
.PHONY : CMakeFiles/camerareader.dir/GeometryTypes.cpp.o.provides

CMakeFiles/camerareader.dir/GeometryTypes.cpp.o.provides.build: CMakeFiles/camerareader.dir/GeometryTypes.cpp.o

CMakeFiles/camerareader.dir/Marker.cpp.o: CMakeFiles/camerareader.dir/flags.make
CMakeFiles/camerareader.dir/Marker.cpp.o: ../Marker.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bec/文档/SolvePnP/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/camerareader.dir/Marker.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/camerareader.dir/Marker.cpp.o -c /home/bec/文档/SolvePnP/Marker.cpp

CMakeFiles/camerareader.dir/Marker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camerareader.dir/Marker.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bec/文档/SolvePnP/Marker.cpp > CMakeFiles/camerareader.dir/Marker.cpp.i

CMakeFiles/camerareader.dir/Marker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camerareader.dir/Marker.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bec/文档/SolvePnP/Marker.cpp -o CMakeFiles/camerareader.dir/Marker.cpp.s

CMakeFiles/camerareader.dir/Marker.cpp.o.requires:
.PHONY : CMakeFiles/camerareader.dir/Marker.cpp.o.requires

CMakeFiles/camerareader.dir/Marker.cpp.o.provides: CMakeFiles/camerareader.dir/Marker.cpp.o.requires
	$(MAKE) -f CMakeFiles/camerareader.dir/build.make CMakeFiles/camerareader.dir/Marker.cpp.o.provides.build
.PHONY : CMakeFiles/camerareader.dir/Marker.cpp.o.provides

CMakeFiles/camerareader.dir/Marker.cpp.o.provides.build: CMakeFiles/camerareader.dir/Marker.cpp.o

CMakeFiles/camerareader.dir/MarkerDetector.cpp.o: CMakeFiles/camerareader.dir/flags.make
CMakeFiles/camerareader.dir/MarkerDetector.cpp.o: ../MarkerDetector.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bec/文档/SolvePnP/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/camerareader.dir/MarkerDetector.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/camerareader.dir/MarkerDetector.cpp.o -c /home/bec/文档/SolvePnP/MarkerDetector.cpp

CMakeFiles/camerareader.dir/MarkerDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camerareader.dir/MarkerDetector.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bec/文档/SolvePnP/MarkerDetector.cpp > CMakeFiles/camerareader.dir/MarkerDetector.cpp.i

CMakeFiles/camerareader.dir/MarkerDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camerareader.dir/MarkerDetector.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bec/文档/SolvePnP/MarkerDetector.cpp -o CMakeFiles/camerareader.dir/MarkerDetector.cpp.s

CMakeFiles/camerareader.dir/MarkerDetector.cpp.o.requires:
.PHONY : CMakeFiles/camerareader.dir/MarkerDetector.cpp.o.requires

CMakeFiles/camerareader.dir/MarkerDetector.cpp.o.provides: CMakeFiles/camerareader.dir/MarkerDetector.cpp.o.requires
	$(MAKE) -f CMakeFiles/camerareader.dir/build.make CMakeFiles/camerareader.dir/MarkerDetector.cpp.o.provides.build
.PHONY : CMakeFiles/camerareader.dir/MarkerDetector.cpp.o.provides

CMakeFiles/camerareader.dir/MarkerDetector.cpp.o.provides.build: CMakeFiles/camerareader.dir/MarkerDetector.cpp.o

CMakeFiles/camerareader.dir/TinyLA.cpp.o: CMakeFiles/camerareader.dir/flags.make
CMakeFiles/camerareader.dir/TinyLA.cpp.o: ../TinyLA.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bec/文档/SolvePnP/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/camerareader.dir/TinyLA.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/camerareader.dir/TinyLA.cpp.o -c /home/bec/文档/SolvePnP/TinyLA.cpp

CMakeFiles/camerareader.dir/TinyLA.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camerareader.dir/TinyLA.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bec/文档/SolvePnP/TinyLA.cpp > CMakeFiles/camerareader.dir/TinyLA.cpp.i

CMakeFiles/camerareader.dir/TinyLA.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camerareader.dir/TinyLA.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bec/文档/SolvePnP/TinyLA.cpp -o CMakeFiles/camerareader.dir/TinyLA.cpp.s

CMakeFiles/camerareader.dir/TinyLA.cpp.o.requires:
.PHONY : CMakeFiles/camerareader.dir/TinyLA.cpp.o.requires

CMakeFiles/camerareader.dir/TinyLA.cpp.o.provides: CMakeFiles/camerareader.dir/TinyLA.cpp.o.requires
	$(MAKE) -f CMakeFiles/camerareader.dir/build.make CMakeFiles/camerareader.dir/TinyLA.cpp.o.provides.build
.PHONY : CMakeFiles/camerareader.dir/TinyLA.cpp.o.provides

CMakeFiles/camerareader.dir/TinyLA.cpp.o.provides.build: CMakeFiles/camerareader.dir/TinyLA.cpp.o

# Object files for target camerareader
camerareader_OBJECTS = \
"CMakeFiles/camerareader.dir/CameraCalibration.cpp.o" \
"CMakeFiles/camerareader.dir/CameraReader.cpp.o" \
"CMakeFiles/camerareader.dir/GeometryTypes.cpp.o" \
"CMakeFiles/camerareader.dir/Marker.cpp.o" \
"CMakeFiles/camerareader.dir/MarkerDetector.cpp.o" \
"CMakeFiles/camerareader.dir/TinyLA.cpp.o"

# External object files for target camerareader
camerareader_EXTERNAL_OBJECTS =

libcamerareader.so: CMakeFiles/camerareader.dir/CameraCalibration.cpp.o
libcamerareader.so: CMakeFiles/camerareader.dir/CameraReader.cpp.o
libcamerareader.so: CMakeFiles/camerareader.dir/GeometryTypes.cpp.o
libcamerareader.so: CMakeFiles/camerareader.dir/Marker.cpp.o
libcamerareader.so: CMakeFiles/camerareader.dir/MarkerDetector.cpp.o
libcamerareader.so: CMakeFiles/camerareader.dir/TinyLA.cpp.o
libcamerareader.so: CMakeFiles/camerareader.dir/build.make
libcamerareader.so: /usr/local/lib/libopencv_videostab.so.2.4.9
libcamerareader.so: /usr/local/lib/libopencv_ts.a
libcamerareader.so: /usr/local/lib/libopencv_superres.so.2.4.9
libcamerareader.so: /usr/local/lib/libopencv_stitching.so.2.4.9
libcamerareader.so: /usr/local/lib/libopencv_contrib.so.2.4.9
libcamerareader.so: /usr/local/lib/libopencv_nonfree.so.2.4.9
libcamerareader.so: /usr/local/lib/libopencv_ocl.so.2.4.9
libcamerareader.so: /usr/local/lib/libopencv_gpu.so.2.4.9
libcamerareader.so: /usr/local/lib/libopencv_photo.so.2.4.9
libcamerareader.so: /usr/local/lib/libopencv_objdetect.so.2.4.9
libcamerareader.so: /usr/local/lib/libopencv_legacy.so.2.4.9
libcamerareader.so: /usr/local/lib/libopencv_video.so.2.4.9
libcamerareader.so: /usr/local/lib/libopencv_ml.so.2.4.9
libcamerareader.so: /usr/local/lib/libopencv_calib3d.so.2.4.9
libcamerareader.so: /usr/local/lib/libopencv_features2d.so.2.4.9
libcamerareader.so: /usr/local/lib/libopencv_highgui.so.2.4.9
libcamerareader.so: /usr/local/lib/libopencv_imgproc.so.2.4.9
libcamerareader.so: /usr/local/lib/libopencv_flann.so.2.4.9
libcamerareader.so: /usr/local/lib/libopencv_core.so.2.4.9
libcamerareader.so: CMakeFiles/camerareader.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library libcamerareader.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/camerareader.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/camerareader.dir/build: libcamerareader.so
.PHONY : CMakeFiles/camerareader.dir/build

CMakeFiles/camerareader.dir/requires: CMakeFiles/camerareader.dir/CameraCalibration.cpp.o.requires
CMakeFiles/camerareader.dir/requires: CMakeFiles/camerareader.dir/CameraReader.cpp.o.requires
CMakeFiles/camerareader.dir/requires: CMakeFiles/camerareader.dir/GeometryTypes.cpp.o.requires
CMakeFiles/camerareader.dir/requires: CMakeFiles/camerareader.dir/Marker.cpp.o.requires
CMakeFiles/camerareader.dir/requires: CMakeFiles/camerareader.dir/MarkerDetector.cpp.o.requires
CMakeFiles/camerareader.dir/requires: CMakeFiles/camerareader.dir/TinyLA.cpp.o.requires
.PHONY : CMakeFiles/camerareader.dir/requires

CMakeFiles/camerareader.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/camerareader.dir/cmake_clean.cmake
.PHONY : CMakeFiles/camerareader.dir/clean

CMakeFiles/camerareader.dir/depend:
	cd /home/bec/文档/SolvePnP/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bec/文档/SolvePnP /home/bec/文档/SolvePnP /home/bec/文档/SolvePnP/build /home/bec/文档/SolvePnP/build /home/bec/文档/SolvePnP/build/CMakeFiles/camerareader.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/camerareader.dir/depend

