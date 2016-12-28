cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pc_seg)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--as-needed")

SET(CMAKE_BUILD_TYPE release)#relwithdebinfo)
#SET(CMAKE_CXX_FLAGS_DISTRIBUTION "-O3")
#SET(CMAKE_C_FLAGS_DISTRIBUTION "-O3")

set(OPENNI_INCLUDE_DIRS /usr/include/ni)
set(ROS_DISTRO indigo)
set(ROS_LIBRARIES /opt/ros/${ROS_DISTRO}/lib/libroscpp.so 
	/opt/ros/${ROS_DISTRO}/lib/libroscpp_serialization.so 
	/opt/ros/${ROS_DISTRO}/lib/librosconsole.so 	
	/opt/ros/${ROS_DISTRO}/lib/libroslib.so
  /opt/ros/${ROS_DISTRO}/lib/librostime.so
  /opt/ros/${ROS_DISTRO}/lib/libtf2.so
  /opt/ros/${ROS_DISTRO}/lib/libtf2_ros.so)

find_package(PCL 1.7 REQUIRED COMPONENTS)
find_package(freenect2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${FREENECT2_INCLUDE_DIRS})

set(IRCP_PATH ~/codes/misc/ircp)
link_directories(${IRCP_PATH}/lib ${FREENECT2_PATH}/lib)

find_package(OpenCV REQUIRED)

include_directories(${IRCP_PATH}/include)
include_directories(${freenect2_INCLUDE_DIRS})
include_directories(include)
include_directories(/opt/ros/${ROS_DISTRO}/include)

#SET(CMAKE_BUILD_TYPE Release)#relwithdebinfo)
#SET(CMAKE_CXX_FLAGS -march=corei7-avx)

#add_library(pcseg SHARED src/pc_segmentation.cpp src/rotcalipers.cpp src/utils.cpp src/region_growing_custom_color.cpp src/utils_pcl_ros.cpp)
add_library(pcseg SHARED src/pc_segmentation.cpp src/rotcalipers.cpp src/utils.cpp src/utils_pcl_ros.cpp)
target_link_libraries(pcseg ${PCL_LIBRARIES} ${ROS_LIBRARIES})

add_library(ircpHandler SHARED src/ircpHandler.cpp)
target_link_libraries(ircpHandler ircp)

#add_executable(offlineProcV2 src/offlineProcV2.cpp)
#target_link_libraries(offlineProcV2 ${PCL_LIBRARIES} pcseg ircp)

add_executable(def_loop src/def_loop.cpp)
target_link_libraries(def_loop ${PCL_LIBRARIES} pcseg ircpHandler ${freenect2_LIBRARY} ${ROS_LIBRARIES} ${OpenCV_LIBS})

add_executable(no_ros_kv2 src/no_ros_kv2.cpp)
target_link_libraries(no_ros_kv2  ${PCL_LIBRARIES} ${freenect2_LIBRARY} pcseg ${OpenCV_LIBS})

add_executable(ros_no_ircp src/ros_no_ircp.cpp)
target_link_libraries(ros_no_ircp ${PCL_LIBRARIES} ${freenect2_LIBRARY} ircpHandler pcseg ${OpenCV_LIBS}  ${ROS_LIBRARIES})



