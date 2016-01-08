cmake_minimum_required(VERSION 2.8.3)
project(ensenso)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  eigen_conversions
  geometry_msgs
  pcl_conversions
  pcl_ros
  tf_conversions
)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen REQUIRED)
find_package(PCL 1.8.0 REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
    cv_bridge
    eigen_conversions
    geometry_msgs
    pcl_conversions
    pcl_ros
    tf_conversions
)

# Order of the include dirs matters!
include_directories(${PCL_INCLUDE_DIRS}) 
include_directories(${catkin_INCLUDE_DIRS}) 
add_definitions(${PCL_DEFINITIONS})


add_executable(ensenso_node src/ensenso_node.cpp)
target_link_libraries(ensenso_node ${PCL_LIBRARIES} ${catkin_LIBRARIES})
