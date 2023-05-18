cmake_minimum_required(VERSION 3.1.0)
project(hri_person_manager)

## Compile as C++11, supported in ROS Kinetic and newer
set (CMAKE_CXX_STANDARD 11)


find_package(catkin REQUIRED COMPONENTS
  hri
  tf2_ros
  hri_msgs
  std_srvs
  roscpp
  diagnostic_msgs
  diagnostic_updater
)

find_package(Boost REQUIRED) # for Boost graph library


include_directories(
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES hri_person_manager
  CATKIN_DEPENDS hri tf2_ros hri_msgs roscpp std_srvs diagnostic_msgs diagnostic_updater
)



add_executable(${PROJECT_NAME}_node 
    src/hri_person_manager.cpp
    src/person_matcher.cpp
    src/managed_person.cpp)



## Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME ${PROJECT_NAME} PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)


###########################################################
# Tests

if(CATKIN_ENABLE_TESTING)

include_directories(
    src
)

find_package(rostest REQUIRED)

add_rostest_gmock(test_hri_person_manager test/test_hri_person_manager.test test/test_hri_person_manager.cpp test/mermaid.cpp src/person_matcher.cpp)
target_include_directories(test_hri_person_manager PRIVATE ${rostest_INCLUDE_DIRS})
target_link_libraries(test_hri_person_manager ${catkin_LIBRARIES})

endif() # CATKIN_ENABLE_TESTING


###########################################################
# Install

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


