cmake_minimum_required(VERSION 2.8.3)
project(ensenso)

find_package(catkin REQUIRED COMPONENTS
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

add_service_files(
  FILES
  CalibrationMoveRandom.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    cri_ensenso_grabber
  CATKIN_DEPENDS 
    cv_bridge
    eigen_conversions
    geometry_msgs
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
add_library(cri_ensenso_grabber src/cri_ensenso_grabber.cpp)
target_link_libraries(cri_ensenso_grabber ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(viewer src/viewer.cpp)
target_link_libraries(viewer cri_ensenso_grabber ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(handeye_calibration src/handeye_calibration.cpp)
add_dependencies(handeye_calibration ${PROJECT_NAME}_gencpp)
target_link_libraries(handeye_calibration cri_ensenso_grabber ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(test_rectified_images src/test_rectified_images.cpp)
add_dependencies(test_rectified_images ${PROJECT_NAME}_gencpp)
target_link_libraries(test_rectified_images cri_ensenso_grabber ${PCL_LIBRARIES} ${catkin_LIBRARIES})
