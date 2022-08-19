# Copyright (c) 2022，Horizon Robotics.
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

cmake_minimum_required(VERSION 3.5)
project(hobot_cv)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include
${PROJECT_SOURCE_DIR}
${CMAKE_SYSROOT}/usr/include/opencv/
thirdparty/include/
)

link_directories(
  ${CMAKE_SYSROOT}/usr/lib/hbbpu/
  ${CMAKE_SYSROOT}/usr/lib/hbmedia/
  thirdparty/lib/
)

SET(LINK_LIBS dnn opencv_world)
SET(VPS_LIBS isp_algo z hbmedia isp ion multimedia iar tinyalsa cjson diag gdcbin vio dl)
SET(NEON_LIBS hobotcv_neon_blur)

add_library(${PROJECT_NAME} SHARED
src/utils.cpp
src/hobotcv_imgproc/hobotcv_imgproc.cpp
src/hobotcv_imgproc/hobotcv_front.cpp
src/hobotcv_imgproc/hobotcv_single.cpp
src/hobotcv_gaussian_blur.cpp
src/hobotcv_gaussian_blur_imp.cpp)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS} ${VPS_LIBS} ${NEON_LIBS})

add_executable(example test/test_crop_resize_rotate.cpp)
target_link_libraries(example
  ${LINK_LIBS}
  ${PROJECT_NAME}
  ${VPS_LIBS}
  ${NEON_LIBS})

add_executable(crop_example test/test_crop.cpp)
target_link_libraries(crop_example
  ${LINK_LIBS}
  ${PROJECT_NAME}
  ${VPS_LIBS})

add_executable(resize_example test/test_resize.cpp)
target_link_libraries(resize_example
  ${LINK_LIBS}
  ${PROJECT_NAME}
  ${VPS_LIBS})

add_executable(rotate_example test/test_rotate.cpp)
target_link_libraries(rotate_example
  ${LINK_LIBS}
  ${PROJECT_NAME}
  ${VPS_LIBS})

add_executable(pyramid_example test/test_pyramid.cpp)
target_link_libraries(pyramid_example
  ${LINK_LIBS}
  ${PROJECT_NAME}
  ${VPS_LIBS})

add_executable(neon_example test/test_neon_blur.cpp)
target_link_libraries(neon_example
  ${LINK_LIBS}
  ${PROJECT_NAME}
  ${VPS_LIBS}
  ${NEON_LIBS})

add_executable(test_gaussian_blur test/test_gaussian_blur.cpp)
target_link_libraries(test_gaussian_blur
  ${LINK_LIBS}
  ${PROJECT_NAME})

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/)

# Install include files
install(
  FILES include/hobotcv_imgproc/hobotcv_imgproc.h
  FILES include/hobotcv_gaussian_blur.h
  FILES include/hobotcv_gaussian_blur_imp.hpp
  FILES thirdparty/include/hobotcv_neon_blur.h
  DESTINATION include/${PROJECT_NAME}/
)

# Install library
install(FILES
${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/libhobotcv_neon_blur.so
DESTINATION lib/)

# Install executables
install(
  TARGETS example crop_example resize_example rotate_example pyramid_example test_gaussian_blur neon_example
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install config
install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/
)

install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "RCLCPP_BUILDING_LIBRARY")

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# specific order: dependents before dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

ament_package()
