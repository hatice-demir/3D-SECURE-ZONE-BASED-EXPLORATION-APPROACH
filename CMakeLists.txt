cmake_minimum_required(VERSION 3.0.2)
project(bitirme)

find_package(octomap REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pcl_conversions
  octomap_msgs
  roscpp
  rospy
  std_msgs
  costmap_2d
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES third_task
#  CATKIN_DEPENDS octomap_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_LIBRARY_DIRS}
)

add_executable(listenMap src/listenMap.cpp)
target_link_libraries(listenMap ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(destination src/destination.cpp)
target_link_libraries(destination ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(mapConvertion src/mapConvertion.cpp)
target_link_libraries(mapConvertion ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(costmapGenerator src/costmapGenerator.cpp)
target_link_libraries(costmapGenerator ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})


catkin_install_python(PROGRAMS scripts/mapConvertion.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/costmapGeneration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)