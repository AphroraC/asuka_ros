cmake_minimum_required(VERSION 3.0.2)
project(asuka_ros VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type." FORCE)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mcx16" ASUKA_ROS_HAS_MCX16)
if(ASUKA_ROS_HAS_MCX16)
  add_compile_options(-mcx16)
else()
  message(WARNING "asuka_ros: -mcx16 is not supported; shared_ptr atomic operation fallback to libstdc++ lock pool implementation")
endif()

add_definitions(-DPCL_NO_PRECOMPILE)

find_package(OpenCV REQUIRED COMPONENTS core)

set(ASUKA_ROS_CATKIN_COMPONENTS
  roscpp
  nodelet
  pluginlib
  pcl_ros
  pcl_conversions
  roslib
  rosbag
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  asuka
)
list(APPEND ASUKA_ROS_CATKIN_COMPONENTS cv_bridge visualization_msgs)

find_package(catkin REQUIRED COMPONENTS ${ASUKA_ROS_CATKIN_COMPONENTS})

find_package(OpenCV REQUIRED COMPONENTS core imgproc)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(spdlog REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES asuka_ros asuka_nodelet
  CATKIN_DEPENDS asuka
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(asuka_ros SHARED
  src/asuka_ros/asuka_ros.cpp
)
target_link_libraries(asuka_ros
  spdlog::spdlog
  ${catkin_LIBRARIES}
)

add_library(asuka_rviz_viewer SHARED
  src/asuka_ros/rviz_viewer.cpp
)
target_link_libraries(asuka_rviz_viewer
  spdlog::spdlog
  ${catkin_LIBRARIES}
)

add_library(asuka_imu_prediction SHARED
  src/asuka_ros/imu_prediction.cpp
)
target_link_libraries(asuka_imu_prediction
  spdlog::spdlog
  ${catkin_LIBRARIES}
)

add_library(asuka_nodelet apps/asuka_nodelet.cpp)
target_link_libraries(asuka_nodelet
  asuka_ros
  ${catkin_LIBRARIES}
)

add_executable(asuka_rosnode apps/asuka_rosnode.cpp)
target_link_libraries(asuka_rosnode
  asuka_ros
  ${catkin_LIBRARIES}
)

add_executable(asuka_rosbag apps/asuka_rosbag.cpp)
target_link_libraries(asuka_rosbag
  asuka_ros
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS asuka_ros asuka_rviz_viewer asuka_imu_prediction asuka_nodelet asuka_rosnode asuka_rosbag
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
