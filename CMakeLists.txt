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

# x3|j5|x86
set(PREFIX_PATH x3)
set(SYS_ROOT ${CMAKE_SYSROOT})

if(PLATFORM_X3)
  message("build platform X3")
  add_definitions(-DPLATFORM_X3)
  set(PREFIX_PATH x3)
elseif(PLATFORM_J5)
  message("build platform J5")
  add_definitions(-DPLATFORM_J5)
  set(PREFIX_PATH j5)
elseif(PLATFORM_X86)
  message("build platform x86")
  add_definitions(-DPLATFORM_X86)
  set(PREFIX_PATH x86)
  set(SYS_ROOT ${THIRD_PARTY})
elseif(PLATFORM_J5)
  message("build platform j5")
  add_definitions(-DPLATFORM_J5)
  set(PREFIX_PATH j5)
  set(SYS_ROOT ${THIRD_PARTY})
else()
  if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    message("invalid platform, build platform X3 default")
    add_definitions(-DPLATFORM_X3)
    set(PLATFORM_X3 ON)
    set(PREFIX_PATH x3)
  elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    message("build platform X86")
    add_definitions(-DPLATFORM_X86)
    set(PLATFORM_X86 ON)
    set(PREFIX_PATH x86)
    set(SYS_ROOT ${THIRD_PARTY})
  endif()
endif()

message("PREFIX_PATH is " ${PREFIX_PATH})
message("SYS_ROOT is " ${SYS_ROOT})
message("PROJECT_SOURCE_DIR" ${PROJECT_SOURCE_DIR})

include_directories(include
  include/${PREFIX_PATH}
  ${PROJECT_SOURCE_DIR}
  ${SYS_ROOT}/usr/include/
  ${SYS_ROOT}/usr/include/opencv/
  thirdparty/include/
)

link_directories(
  ${SYS_ROOT}/usr/lib/
  ${SYS_ROOT}/usr/lib/hbbpu/
  ${SYS_ROOT}/usr/lib/hbmedia/
  thirdparty/lib/
)

SET(LINK_LIBS dnn opencv_world)

if(PLATFORM_X3)

  SET(VPS_LIBS isp_algo z hbmedia isp ion multimedia iar tinyalsa cjson diag gdcbin vio dl)
  SET(NEON_LIBS hobotcv_neon_blur)

  add_library(${PROJECT_NAME} SHARED
    src/utils.cpp
    src/x3/hobotcv_imgproc/hobotcv_imgproc.cpp
    src/x3/hobotcv_imgproc/hobotcv_front.cpp
    src/x3/hobotcv_imgproc/hobotcv_single.cpp
    src/x3/hobotcv_gaussian_blur.cpp
    src/x3/hobotcv_gaussian_blur_imp.cpp
  )
  target_link_libraries(${PROJECT_NAME} ${LINK_LIBS} ${VPS_LIBS} ${NEON_LIBS})

  add_executable(example test/test_crop_resize_rotate.cpp)
  target_link_libraries(example
    ${LINK_LIBS}
    ${PROJECT_NAME}
    ${VPS_LIBS})

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

  add_executable(padding_example test/test_padding.cpp)
  target_link_libraries(padding_example
    ${LINK_LIBS}
    ${PROJECT_NAME})

  add_executable(neon_example test/test_neon_blur.cpp)
  target_link_libraries(neon_example
    ${LINK_LIBS}
    ${PROJECT_NAME}
    ${VPS_LIBS})

  add_executable(test_gaussian_blur test/test_gaussian_blur.cpp)
  target_link_libraries(test_gaussian_blur
    ${LINK_LIBS}
    ${PROJECT_NAME})

  add_executable(hobotcv_benchmark
  benchmark/main.cpp
  benchmark/hobotcv_benchmark_node.cpp)
  target_link_libraries(hobotcv_benchmark
    ${LINK_LIBS}
    ${PROJECT_NAME}
    ${VPS_LIBS})

  ament_target_dependencies(
    ${PROJECT_NAME}
    rclcpp
  )

  # Install libraries
  install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/)

  # Install include files
  install(
    FILES include/x3/hobotcv_imgproc/hobotcv_imgproc.h
    FILES include/x3/hobotcv_gaussian_blur.h
    FILES include/x3/hobotcv_gaussian_blur_imp.hpp
    FILES thirdparty/include/hobotcv_neon_blur.h
    DESTINATION include/${PROJECT_NAME}/
  )

  # Install library
  install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/libhobotcv_neon_blur.so
  DESTINATION lib/)

  # Install executables
  install(
    TARGETS example crop_example resize_example rotate_example pyramid_example padding_example test_gaussian_blur neon_example hobotcv_benchmark
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
elseif(PLATFORM_X86)

  add_library(${PROJECT_NAME} SHARED
    src/utils.cpp
    src/x86/hobotcv_imgproc/hobotcv_imgproc.cpp
    # src/x86/hobotcv_imgproc/hobotcv_front.cpp
    # src/x86/hobotcv_imgproc/hobotcv_single.cpp
    # src/x86/hobotcv_gaussian_blur.cpp
    # src/x86/hobotcv_gaussian_blur_imp.cpp
  )
  target_link_libraries(${PROJECT_NAME} ${LINK_LIBS} ${VPS_LIBS} ${NEON_LIBS})

  # add_executable(example test/test_crop_resize_rotate.cpp)
  # target_link_libraries(example
  # ${LINK_LIBS}
  # ${PROJECT_NAME}
  # ${VPS_LIBS})

  # add_executable(crop_example test/test_crop.cpp)
  # target_link_libraries(crop_example
  # ${LINK_LIBS}
  # ${PROJECT_NAME}
  # ${VPS_LIBS})

  add_executable(resize_example test/test_resize.cpp)
  target_link_libraries(resize_example
  ${LINK_LIBS}
  ${PROJECT_NAME}
  ${VPS_LIBS})

  # add_executable(rotate_example test/test_rotate.cpp)
  # target_link_libraries(rotate_example
  # ${LINK_LIBS}
  # ${PROJECT_NAME}
  # ${VPS_LIBS})

  # add_executable(pyramid_example test/test_pyramid.cpp)
  # target_link_libraries(pyramid_example
  # ${LINK_LIBS}
  # ${PROJECT_NAME}
  # ${VPS_LIBS})

  # add_executable(padding_example test/test_padding.cpp)
  # target_link_libraries(padding_example
  # ${LINK_LIBS}
  # ${PROJECT_NAME})

  # add_executable(neon_example test/test_neon_blur.cpp)
  # target_link_libraries(neon_example
  # ${LINK_LIBS}
  # ${PROJECT_NAME}
  # ${VPS_LIBS})

  # add_executable(test_gaussian_blur test/test_gaussian_blur.cpp)
  # target_link_libraries(test_gaussian_blur
  # ${LINK_LIBS}
  # ${PROJECT_NAME})

  # add_executable(hobotcv_benchmark
  # benchmark/main.cpp
  # benchmark/hobotcv_benchmark_node.cpp)
  # target_link_libraries(hobotcv_benchmark
  # ${LINK_LIBS}
  # ${PROJECT_NAME}
  # ${VPS_LIBS})

  ament_target_dependencies(
    ${PROJECT_NAME}
    rclcpp
  )

  # Install libraries
  install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/)

  install(FILES
  ${SYS_ROOT}/usr/lib/libopencv_world.so.3.4
  DESTINATION lib/)

  # Install include files
  # install(
  #   FILES include/hobotcv_imgproc/hobotcv_imgproc.h
  #   FILES include/hobotcv_gaussian_blur.h
  #   FILES include/hobotcv_gaussian_blur_imp.hpp
  #   FILES thirdparty/include/hobotcv_neon_blur.h
  #   DESTINATION include/${PROJECT_NAME}/
  # )
  install(
    FILES include/x86/hobotcv_imgproc/hobotcv_imgproc.h
    # FILES include/x86/hobotcv_gaussian_blur.h
    # FILES include/x86/hobotcv_gaussian_blur_imp.hpp
    # FILES thirdparty/include/hobotcv_neon_blur.h
    DESTINATION include/${PROJECT_NAME}/
  )


  # Install library
  # install(FILES
  # ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/libhobotcv_neon_blur.so
  # DESTINATION lib/)

  # Install executables
  # install(
  # TARGETS example crop_example resize_example rotate_example pyramid_example padding_example test_gaussian_blur neon_example hobotcv_benchmark
  # RUNTIME DESTINATION lib/${PROJECT_NAME}
  # )

  install(
  TARGETS resize_example
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
elseif(PLATFORM_J5)
  add_library(${PROJECT_NAME} SHARED
    src/utils.cpp
    src/j5/hobotcv_imgproc/hobotcv_imgproc.cpp
  )
  target_link_libraries(${PROJECT_NAME} ${LINK_LIBS} ${VPS_LIBS} ${NEON_LIBS})

  add_executable(resize_example test/test_resize.cpp)
  target_link_libraries(resize_example
  ${LINK_LIBS}
  ${PROJECT_NAME}
  ${VPS_LIBS})

  ament_target_dependencies(
    ${PROJECT_NAME}
    rclcpp
  )

  # Install libraries
  install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/)

  install(FILES
  ${SYS_ROOT}/usr/lib/libopencv_world.so.3.4
  DESTINATION lib/)

  install(
    FILES include/j5/hobotcv_imgproc/hobotcv_imgproc.h
    DESTINATION include/${PROJECT_NAME}/
  )

  install(
  TARGETS resize_example
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
endif()

# specific order: dependents before dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

ament_package()
