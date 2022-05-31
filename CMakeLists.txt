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
)

include_directories(
  ${CMAKE_SYSROOT}/usr/include/opencv/
)

link_directories(
  ${CMAKE_SYSROOT}/usr/lib/hbbpu/
)

SET(LINK_LIBS dnn opencv_world)

add_library(${PROJECT_NAME} SHARED src/utils.cpp src/hobotcv_dnnresize.cpp )
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})

add_executable(example src/example.cpp)
target_link_libraries(example
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
  FILES include/hobotcv_dnnresize.h
  DESTINATION include/${PROJECT_NAME}/
)

# Install executables
install(
  TARGETS example
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install config
install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/
)

ament_package()
