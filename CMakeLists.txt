cmake_minimum_required(VERSION 2.8.3)
project(starneto_driver)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf
  nav_msgs
  pcl_ros
  )

add_message_files(DIRECTORY msg FILES
  inspva.msg
  )

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(starneto_driver STATIC src/GPSDriver.cpp)
# add_library(visualize_starneto_driver STATIC src/visualize_GPS_path.cpp)

add_executable(${PROJECT_NAME}_node src/starneto_driver_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  starneto_driver
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

# add_executable(visualize_rtk_node src/visualize_rtk_node.cpp)
# target_link_libraries(visualize_rtk_node
#   visualize_starneto_driver
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   )

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node #visualize_rtk_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} #visualize_starneto_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(
  DIRECTORY 
  launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY 
  config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)