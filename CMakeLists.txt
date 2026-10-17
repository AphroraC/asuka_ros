cmake_minimum_required(VERSION 3.0.2)
project(asuka_ros VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type." FORCE)

# Must match the asuka package's compilation strategy: the callback slots
# publish shared_ptr snapshots through libstdc++'s atomic free functions,
# which only take the lock-free path with cmpxchg16b. Every TU in the process
# must be compiled with the same setting.
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mcx16" ASUKA_ROS_HAS_MCX16)
if(ASUKA_ROS_HAS_MCX16)
  add_compile_options(-mcx16)
else()
  message(WARNING "asuka_ros: -mcx16 is not supported; shared_ptr atomic operation fallback to libstdc++ lock pool implementation")
endif()

# Same custom-point-type strategy as the asuka package (PointXYZIOffset):
# instantiate every PCL template from headers. catkin links downstream
# packages to asuka_core by library path, which bypasses its target usage
# requirements, so this definition has to be set here as well.
add_definitions(-DPCL_NO_PRECOMPILE)

# OpenCV is an unconditional dependency of both packages (image input path,
# PointXYZIRGBOffset point type and the color-capable map saving).
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

# The ROS adapter package of asuka (mirrors glim_ros): sensor I/O, the
# rviz_viewer extension module and the three entry points. The library itself
# (asuka) carries no ROS dependency.
find_package(catkin REQUIRED COMPONENTS ${ASUKA_ROS_CATKIN_COMPONENTS})

# OpenCV headers are needed even in OFF builds: the viewer extensions and
# cv_bridge usage resolve OpenCV components explicitly.
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

# ROS adapter: converts sensor_msgs into asuka's neutral types, owns the
# odometry executor and the loaded extension modules. The three entry points
# below talk to asuka only through it.
add_library(asuka_ros SHARED
  src/asuka_ros/asuka_ros.cpp
)
target_link_libraries(asuka_ros
  spdlog::spdlog
  ${catkin_LIBRARIES}
)

# KeyFrame visualization extension module (dlopened via config_extensions.json;
# the target name libasuka_rviz_viewer.so is referenced by that config).
add_library(asuka_rviz_viewer SHARED
  src/asuka_ros/rviz_viewer.cpp
)
target_link_libraries(asuka_rviz_viewer
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
install(TARGETS asuka_ros asuka_rviz_viewer asuka_nodelet asuka_rosnode asuka_rosbag
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
