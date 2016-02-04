cmake_minimum_required(VERSION 2.8.3)
project(pubsubtest)

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
  tf_conversions
  image_transport
)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen REQUIRED)
find_package(PCL 1.8.0 REQUIRED)


generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    camera_info_manager
    cv_bridge
    eigen_conversions
    message_runtime
    geometry_msgs
    std_msgs
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
add_executable(pubpc pubpc.cpp)
target_link_libraries(pubpc ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(subpc subpc.cpp)
target_link_libraries(subpc ${PCL_LIBRARIES} ${catkin_LIBRARIES})
