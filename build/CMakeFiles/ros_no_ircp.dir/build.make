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
include CMakeFiles/ros_no_ircp.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ros_no_ircp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ros_no_ircp.dir/flags.make

CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.o: CMakeFiles/ros_no_ircp.dir/flags.make
CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.o: ../src/ros_no_ircp.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/erman/tmp/pc-segmentation/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.o -c /home/erman/tmp/pc-segmentation/src/ros_no_ircp.cpp

CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/erman/tmp/pc-segmentation/src/ros_no_ircp.cpp > CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.i

CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/erman/tmp/pc-segmentation/src/ros_no_ircp.cpp -o CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.s

CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.o.requires:
.PHONY : CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.o.requires

CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.o.provides: CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.o.requires
	$(MAKE) -f CMakeFiles/ros_no_ircp.dir/build.make CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.o.provides.build
.PHONY : CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.o.provides

CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.o.provides.build: CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.o

# Object files for target ros_no_ircp
ros_no_ircp_OBJECTS = \
"CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.o"

# External object files for target ros_no_ircp
ros_no_ircp_EXTERNAL_OBJECTS =

ros_no_ircp: CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.o
ros_no_ircp: CMakeFiles/ros_no_ircp.dir/build.make
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_system.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_thread.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libpthread.so
ros_no_ircp: /usr/lib/libpcl_common.so
ros_no_ircp: /usr/lib/libpcl_octree.so
ros_no_ircp: /usr/lib/libOpenNI.so
ros_no_ircp: /usr/lib/libvtkCommon.so.5.8.0
ros_no_ircp: /usr/lib/libvtkRendering.so.5.8.0
ros_no_ircp: /usr/lib/libvtkHybrid.so.5.8.0
ros_no_ircp: /usr/lib/libvtkCharts.so.5.8.0
ros_no_ircp: /usr/lib/libpcl_io.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
ros_no_ircp: /usr/lib/libpcl_kdtree.so
ros_no_ircp: /usr/lib/libpcl_search.so
ros_no_ircp: /usr/lib/libpcl_sample_consensus.so
ros_no_ircp: /usr/lib/libpcl_filters.so
ros_no_ircp: /usr/lib/libpcl_features.so
ros_no_ircp: /usr/lib/libpcl_keypoints.so
ros_no_ircp: /usr/lib/libpcl_segmentation.so
ros_no_ircp: /usr/lib/libpcl_visualization.so
ros_no_ircp: /usr/lib/libpcl_outofcore.so
ros_no_ircp: /usr/lib/libpcl_registration.so
ros_no_ircp: /usr/lib/libpcl_recognition.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libqhull.so
ros_no_ircp: /usr/lib/libpcl_surface.so
ros_no_ircp: /usr/lib/libpcl_people.so
ros_no_ircp: /usr/lib/libpcl_tracking.so
ros_no_ircp: /usr/lib/libpcl_apps.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_system.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_thread.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libpthread.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libqhull.so
ros_no_ircp: /usr/lib/libOpenNI.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
ros_no_ircp: /usr/lib/libvtkCommon.so.5.8.0
ros_no_ircp: /usr/lib/libvtkRendering.so.5.8.0
ros_no_ircp: /usr/lib/libvtkHybrid.so.5.8.0
ros_no_ircp: /usr/lib/libvtkCharts.so.5.8.0
ros_no_ircp: /home/erman/freenect2/lib/libfreenect2.so
ros_no_ircp: libircpHandler.so
ros_no_ircp: libpcseg.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
ros_no_ircp: /opt/ros/indigo/lib/libroscpp.so
ros_no_ircp: /opt/ros/indigo/lib/libroscpp_serialization.so
ros_no_ircp: /opt/ros/indigo/lib/librosconsole.so
ros_no_ircp: /opt/ros/indigo/lib/libroslib.so
ros_no_ircp: /opt/ros/indigo/lib/librostime.so
ros_no_ircp: /opt/ros/indigo/lib/libtf2.so
ros_no_ircp: /opt/ros/indigo/lib/libtf2_ros.so
ros_no_ircp: /usr/lib/libvtkCharts.so.5.8.0
ros_no_ircp: /usr/lib/libvtkViews.so.5.8.0
ros_no_ircp: /usr/lib/libvtkInfovis.so.5.8.0
ros_no_ircp: /usr/lib/libvtkWidgets.so.5.8.0
ros_no_ircp: /usr/lib/libvtkHybrid.so.5.8.0
ros_no_ircp: /usr/lib/libvtkParallel.so.5.8.0
ros_no_ircp: /usr/lib/libvtkVolumeRendering.so.5.8.0
ros_no_ircp: /usr/lib/libvtkRendering.so.5.8.0
ros_no_ircp: /usr/lib/libvtkGraphics.so.5.8.0
ros_no_ircp: /usr/lib/libvtkImaging.so.5.8.0
ros_no_ircp: /usr/lib/libvtkIO.so.5.8.0
ros_no_ircp: /usr/lib/libvtkFiltering.so.5.8.0
ros_no_ircp: /usr/lib/libvtkCommon.so.5.8.0
ros_no_ircp: /usr/lib/libvtksys.so.5.8.0
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_system.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_thread.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libpthread.so
ros_no_ircp: /usr/lib/libpcl_common.so
ros_no_ircp: /usr/lib/libpcl_octree.so
ros_no_ircp: /usr/lib/libOpenNI.so
ros_no_ircp: /usr/lib/libpcl_io.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
ros_no_ircp: /usr/lib/libpcl_kdtree.so
ros_no_ircp: /usr/lib/libpcl_search.so
ros_no_ircp: /usr/lib/libpcl_sample_consensus.so
ros_no_ircp: /usr/lib/libpcl_filters.so
ros_no_ircp: /usr/lib/libpcl_features.so
ros_no_ircp: /usr/lib/libpcl_keypoints.so
ros_no_ircp: /usr/lib/libpcl_segmentation.so
ros_no_ircp: /usr/lib/libpcl_visualization.so
ros_no_ircp: /usr/lib/libpcl_outofcore.so
ros_no_ircp: /usr/lib/libpcl_registration.so
ros_no_ircp: /usr/lib/libpcl_recognition.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libqhull.so
ros_no_ircp: /usr/lib/libpcl_surface.so
ros_no_ircp: /usr/lib/libpcl_people.so
ros_no_ircp: /usr/lib/libpcl_tracking.so
ros_no_ircp: /usr/lib/libpcl_apps.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_system.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_thread.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libpthread.so
ros_no_ircp: /usr/lib/libpcl_common.so
ros_no_ircp: /usr/lib/libpcl_octree.so
ros_no_ircp: /usr/lib/libOpenNI.so
ros_no_ircp: /usr/lib/libpcl_io.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
ros_no_ircp: /usr/lib/libpcl_kdtree.so
ros_no_ircp: /usr/lib/libpcl_search.so
ros_no_ircp: /usr/lib/libpcl_sample_consensus.so
ros_no_ircp: /usr/lib/libpcl_filters.so
ros_no_ircp: /usr/lib/libpcl_features.so
ros_no_ircp: /usr/lib/libpcl_keypoints.so
ros_no_ircp: /usr/lib/libpcl_segmentation.so
ros_no_ircp: /usr/lib/libpcl_visualization.so
ros_no_ircp: /usr/lib/libpcl_outofcore.so
ros_no_ircp: /usr/lib/libpcl_registration.so
ros_no_ircp: /usr/lib/libpcl_recognition.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libqhull.so
ros_no_ircp: /usr/lib/libpcl_surface.so
ros_no_ircp: /usr/lib/libpcl_people.so
ros_no_ircp: /usr/lib/libpcl_tracking.so
ros_no_ircp: /usr/lib/libpcl_apps.so
ros_no_ircp: /opt/ros/indigo/lib/libroscpp.so
ros_no_ircp: /opt/ros/indigo/lib/libroscpp_serialization.so
ros_no_ircp: /opt/ros/indigo/lib/librosconsole.so
ros_no_ircp: /opt/ros/indigo/lib/libroslib.so
ros_no_ircp: /opt/ros/indigo/lib/librostime.so
ros_no_ircp: /opt/ros/indigo/lib/libtf2.so
ros_no_ircp: /opt/ros/indigo/lib/libtf2_ros.so
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
ros_no_ircp: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
ros_no_ircp: CMakeFiles/ros_no_ircp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ros_no_ircp"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ros_no_ircp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ros_no_ircp.dir/build: ros_no_ircp
.PHONY : CMakeFiles/ros_no_ircp.dir/build

CMakeFiles/ros_no_ircp.dir/requires: CMakeFiles/ros_no_ircp.dir/src/ros_no_ircp.cpp.o.requires
.PHONY : CMakeFiles/ros_no_ircp.dir/requires

CMakeFiles/ros_no_ircp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ros_no_ircp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ros_no_ircp.dir/clean

CMakeFiles/ros_no_ircp.dir/depend:
	cd /home/erman/tmp/pc-segmentation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/erman/tmp/pc-segmentation /home/erman/tmp/pc-segmentation /home/erman/tmp/pc-segmentation/build /home/erman/tmp/pc-segmentation/build /home/erman/tmp/pc-segmentation/build/CMakeFiles/ros_no_ircp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ros_no_ircp.dir/depend
