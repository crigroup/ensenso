cmake_minimum_required(VERSION 2.8.3)
project(ensenso)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cmake_modules
  cv_bridge
  eigen_conversions
  geometry_msgs
  message_generation
  pcl_conversions
  pcl_ros
  std_msgs
  std_srvs
  tf_conversions
  image_transport
)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen REQUIRED)
find_package(PCL 1.8.0 REQUIRED)

add_service_files(
  FILES
  CapturePattern.srv
  ComputeCalibration.srv
  ConfigureStreaming.srv
  GridSpacing.srv
  InitCalibration.srv
  Lights.srv
  SetBool.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ensenso_grabber
  CATKIN_DEPENDS 
    camera_info_manager
    cv_bridge
    eigen_conversions
    message_runtime
    geometry_msgs
    std_msgs
    std_srvs
    pcl_conversions
    pcl_ros
    tf_conversions
)

# Order of the include dirs matters!
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS}) 
include_directories(${catkin_INCLUDE_DIRS}) 
add_definitions(${PCL_DEFINITIONS})


# Libraries
add_library(ensenso_grabber src/ensenso_grabber.cpp)
target_link_libraries(ensenso_grabber ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(ensenso_node src/ensenso_node.cpp)
add_dependencies(ensenso_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ensenso_node ensenso_grabber ${PCL_LIBRARIES} ${catkin_LIBRARIES})
