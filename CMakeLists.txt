# Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.


cmake_minimum_required(VERSION 3.8)
project(hri_person_manager)

# Global flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_auto_find_build_dependencies()

# Targets
ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

ament_auto_add_library(${PROJECT_NAME}_lib
  src/hri_person_manager/node_person_manager.cpp
  src/hri_person_manager/managed_person.cpp
  src/hri_person_manager/person_matcher.cpp
)

ament_auto_add_executable(${PROJECT_NAME}
  src/main.cpp
)

# Additional install
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()

  ament_auto_add_gtest(${PROJECT_NAME}_node_test
    test/test_node_person_manager.cpp
  )

  ament_auto_add_gtest(${PROJECT_NAME}_lib_test
    test/test_person_matcher.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
  )

endif()

ament_auto_package()
