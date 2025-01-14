# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(hri_face_body_matcher)

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
ament_auto_find_build_dependencies()
find_package(dlib REQUIRED)
find_package(BLAS REQUIRED)

# Targets
ament_auto_add_library(${PROJECT_NAME}_lib
  src/hri_face_body_matcher/node_face_body_matcher.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
  dlib::dlib
  ${BLAS_LIBRARIES}
)

ament_auto_add_executable(${PROJECT_NAME}
  src/main.cpp
)

# Additional install
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(pal_module_cmake_FOUND)
  pal_register_modules(
    module/hri_face_body_matcher_module.yaml
  )
endif()

# Testing
if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()

  # TODO(LJU): replace with ament_auto_add_gmock() when available
  ament_add_gmock(${PROJECT_NAME}_test test/test_node_face_body_matcher.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_lib)
endif()

ament_auto_package()
