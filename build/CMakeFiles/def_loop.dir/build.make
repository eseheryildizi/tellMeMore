# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.0

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
CMAKE_SOURCE_DIR = /home/erman/tmp/pc-segmentation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/erman/tmp/pc-segmentation/build

# Include any dependencies generated for this target.
include CMakeFiles/def_loop.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/def_loop.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/def_loop.dir/flags.make

CMakeFiles/def_loop.dir/src/def_loop.cpp.o: CMakeFiles/def_loop.dir/flags.make
CMakeFiles/def_loop.dir/src/def_loop.cpp.o: ../src/def_loop.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/erman/tmp/pc-segmentation/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/def_loop.dir/src/def_loop.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/def_loop.dir/src/def_loop.cpp.o -c /home/erman/tmp/pc-segmentation/src/def_loop.cpp

CMakeFiles/def_loop.dir/src/def_loop.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/def_loop.dir/src/def_loop.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/erman/tmp/pc-segmentation/src/def_loop.cpp > CMakeFiles/def_loop.dir/src/def_loop.cpp.i

CMakeFiles/def_loop.dir/src/def_loop.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/def_loop.dir/src/def_loop.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/erman/tmp/pc-segmentation/src/def_loop.cpp -o CMakeFiles/def_loop.dir/src/def_loop.cpp.s

CMakeFiles/def_loop.dir/src/def_loop.cpp.o.requires:
.PHONY : CMakeFiles/def_loop.dir/src/def_loop.cpp.o.requires

CMakeFiles/def_loop.dir/src/def_loop.cpp.o.provides: CMakeFiles/def_loop.dir/src/def_loop.cpp.o.requires
	$(MAKE) -f CMakeFiles/def_loop.dir/build.make CMakeFiles/def_loop.dir/src/def_loop.cpp.o.provides.build
.PHONY : CMakeFiles/def_loop.dir/src/def_loop.cpp.o.provides

CMakeFiles/def_loop.dir/src/def_loop.cpp.o.provides.build: CMakeFiles/def_loop.dir/src/def_loop.cpp.o

# Object files for target def_loop
def_loop_OBJECTS = \
"CMakeFiles/def_loop.dir/src/def_loop.cpp.o"

# External object files for target def_loop
def_loop_EXTERNAL_OBJECTS =

def_loop: CMakeFiles/def_loop.dir/src/def_loop.cpp.o
def_loop: CMakeFiles/def_loop.dir/build.make
def_loop: /usr/lib/x86_64-linux-gnu/libboost_system.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_thread.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
def_loop: /usr/lib/x86_64-linux-gnu/libpthread.so
def_loop: /usr/lib/libpcl_common.so
def_loop: /usr/lib/libpcl_octree.so
def_loop: /usr/lib/libOpenNI.so
def_loop: /usr/lib/libvtkCommon.so.5.8.0
def_loop: /usr/lib/libvtkRendering.so.5.8.0
def_loop: /usr/lib/libvtkHybrid.so.5.8.0
def_loop: /usr/lib/libvtkCharts.so.5.8.0
def_loop: /usr/lib/libpcl_io.so
def_loop: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
def_loop: /usr/lib/libpcl_kdtree.so
def_loop: /usr/lib/libpcl_search.so
def_loop: /usr/lib/libpcl_sample_consensus.so
def_loop: /usr/lib/libpcl_filters.so
def_loop: /usr/lib/libpcl_features.so
def_loop: /usr/lib/libpcl_keypoints.so
def_loop: /usr/lib/libpcl_segmentation.so
def_loop: /usr/lib/libpcl_visualization.so
def_loop: /usr/lib/libpcl_outofcore.so
def_loop: /usr/lib/libpcl_registration.so
def_loop: /usr/lib/libpcl_recognition.so
def_loop: /usr/lib/x86_64-linux-gnu/libqhull.so
def_loop: /usr/lib/libpcl_surface.so
def_loop: /usr/lib/libpcl_people.so
def_loop: /usr/lib/libpcl_tracking.so
def_loop: /usr/lib/libpcl_apps.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_system.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_thread.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
def_loop: /usr/lib/x86_64-linux-gnu/libpthread.so
def_loop: /usr/lib/x86_64-linux-gnu/libqhull.so
def_loop: /usr/lib/libOpenNI.so
def_loop: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
def_loop: /usr/lib/libvtkCommon.so.5.8.0
def_loop: /usr/lib/libvtkRendering.so.5.8.0
def_loop: /usr/lib/libvtkHybrid.so.5.8.0
def_loop: /usr/lib/libvtkCharts.so.5.8.0
def_loop: libpcseg.so
def_loop: libircpHandler.so
def_loop: /home/erman/freenect2/lib/libfreenect2.so
def_loop: /opt/ros/indigo/lib/libroscpp.so
def_loop: /opt/ros/indigo/lib/libroscpp_serialization.so
def_loop: /opt/ros/indigo/lib/librosconsole.so
def_loop: /opt/ros/indigo/lib/libroslib.so
def_loop: /opt/ros/indigo/lib/librostime.so
def_loop: /opt/ros/indigo/lib/libtf2.so
def_loop: /opt/ros/indigo/lib/libtf2_ros.so
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
def_loop: /usr/lib/libvtkCharts.so.5.8.0
def_loop: /usr/lib/libvtkViews.so.5.8.0
def_loop: /usr/lib/libvtkInfovis.so.5.8.0
def_loop: /usr/lib/libvtkWidgets.so.5.8.0
def_loop: /usr/lib/libvtkHybrid.so.5.8.0
def_loop: /usr/lib/libvtkParallel.so.5.8.0
def_loop: /usr/lib/libvtkVolumeRendering.so.5.8.0
def_loop: /usr/lib/libvtkRendering.so.5.8.0
def_loop: /usr/lib/libvtkGraphics.so.5.8.0
def_loop: /usr/lib/libvtkImaging.so.5.8.0
def_loop: /usr/lib/libvtkIO.so.5.8.0
def_loop: /usr/lib/libvtkFiltering.so.5.8.0
def_loop: /usr/lib/libvtkCommon.so.5.8.0
def_loop: /usr/lib/libvtksys.so.5.8.0
def_loop: /usr/lib/x86_64-linux-gnu/libboost_system.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_thread.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
def_loop: /usr/lib/x86_64-linux-gnu/libpthread.so
def_loop: /usr/lib/libpcl_common.so
def_loop: /usr/lib/libpcl_octree.so
def_loop: /usr/lib/libOpenNI.so
def_loop: /usr/lib/libpcl_io.so
def_loop: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
def_loop: /usr/lib/libpcl_kdtree.so
def_loop: /usr/lib/libpcl_search.so
def_loop: /usr/lib/libpcl_sample_consensus.so
def_loop: /usr/lib/libpcl_filters.so
def_loop: /usr/lib/libpcl_features.so
def_loop: /usr/lib/libpcl_keypoints.so
def_loop: /usr/lib/libpcl_segmentation.so
def_loop: /usr/lib/libpcl_visualization.so
def_loop: /usr/lib/libpcl_outofcore.so
def_loop: /usr/lib/libpcl_registration.so
def_loop: /usr/lib/libpcl_recognition.so
def_loop: /usr/lib/x86_64-linux-gnu/libqhull.so
def_loop: /usr/lib/libpcl_surface.so
def_loop: /usr/lib/libpcl_people.so
def_loop: /usr/lib/libpcl_tracking.so
def_loop: /usr/lib/libpcl_apps.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_system.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_thread.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
def_loop: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
def_loop: /usr/lib/x86_64-linux-gnu/libpthread.so
def_loop: /usr/lib/libpcl_common.so
def_loop: /usr/lib/libpcl_octree.so
def_loop: /usr/lib/libOpenNI.so
def_loop: /usr/lib/libpcl_io.so
def_loop: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
def_loop: /usr/lib/libpcl_kdtree.so
def_loop: /usr/lib/libpcl_search.so
def_loop: /usr/lib/libpcl_sample_consensus.so
def_loop: /usr/lib/libpcl_filters.so
def_loop: /usr/lib/libpcl_features.so
def_loop: /usr/lib/libpcl_keypoints.so
def_loop: /usr/lib/libpcl_segmentation.so
def_loop: /usr/lib/libpcl_visualization.so
def_loop: /usr/lib/libpcl_outofcore.so
def_loop: /usr/lib/libpcl_registration.so
def_loop: /usr/lib/libpcl_recognition.so
def_loop: /usr/lib/x86_64-linux-gnu/libqhull.so
def_loop: /usr/lib/libpcl_surface.so
def_loop: /usr/lib/libpcl_people.so
def_loop: /usr/lib/libpcl_tracking.so
def_loop: /usr/lib/libpcl_apps.so
def_loop: /opt/ros/indigo/lib/libroscpp.so
def_loop: /opt/ros/indigo/lib/libroscpp_serialization.so
def_loop: /opt/ros/indigo/lib/librosconsole.so
def_loop: /opt/ros/indigo/lib/libroslib.so
def_loop: /opt/ros/indigo/lib/librostime.so
def_loop: /opt/ros/indigo/lib/libtf2.so
def_loop: /opt/ros/indigo/lib/libtf2_ros.so
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
def_loop: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
def_loop: CMakeFiles/def_loop.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable def_loop"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/def_loop.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/def_loop.dir/build: def_loop
.PHONY : CMakeFiles/def_loop.dir/build

CMakeFiles/def_loop.dir/requires: CMakeFiles/def_loop.dir/src/def_loop.cpp.o.requires
.PHONY : CMakeFiles/def_loop.dir/requires

CMakeFiles/def_loop.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/def_loop.dir/cmake_clean.cmake
.PHONY : CMakeFiles/def_loop.dir/clean

CMakeFiles/def_loop.dir/depend:
	cd /home/erman/tmp/pc-segmentation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/erman/tmp/pc-segmentation /home/erman/tmp/pc-segmentation /home/erman/tmp/pc-segmentation/build /home/erman/tmp/pc-segmentation/build /home/erman/tmp/pc-segmentation/build/CMakeFiles/def_loop.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/def_loop.dir/depend

