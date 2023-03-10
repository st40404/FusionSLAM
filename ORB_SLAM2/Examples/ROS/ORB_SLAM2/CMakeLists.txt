cmake_minimum_required(VERSION 2.8)
project(orbslam2_ros)

find_package(catkin REQUIRED
   roscpp
   tf
   cv_bridge
   all_process
   image_transport
   sensor_msgs
)

# MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    all_process
)

include_directories(
   ${catkin_INCLUDE_DIRS}
   ${PROJECT_SOURCE_DIR}
   ${PROJECT_SOURCE_DIR}/../../../
   ${PROJECT_SOURCE_DIR}/../../../include
   ${Pangolin_INCLUDE_DIRS}
   ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(LIBS
   ${OpenCV_LIBS}
   ${EIGEN3_LIBS}
   ${Pangolin_LIBRARIES}
   ${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
   ${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
   ${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
   -lboost_system
)


# Node for monocular camera
add_executable(Mono src/ros_mono.cc)
target_link_libraries(Mono
   ${LIBS}
   ${catkin_LIBRARIES}
)

# Node for monocular camera (Augmented Reality Demo)
add_executable(MonoAR
   src/AR/ros_mono_ar.cc
   src/AR/ViewerAR.h
   src/AR/ViewerAR.cc
)
target_link_libraries(MonoAR
   ${LIBS}
   ${catkin_LIBRARIES}
)

# Node for stereo camera
add_executable(Stereo
   src/ros_stereo.cc
)
target_link_libraries(Stereo
   ${LIBS}
   ${catkin_LIBRARIES}
)

# Node for ZED camera
add_executable(Zed
   src/ros_zed.cc
)
target_link_libraries(Zed
   ${LIBS}
   ${catkin_LIBRARIES}
)

# Node for RGB-D camera
add_executable(RGBD
   src/ros_rgbd.cc
)
target_link_libraries(RGBD
   ${LIBS}
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
)
add_dependencies(RGBD ${catkin_EXPORTED_TARGETS})
