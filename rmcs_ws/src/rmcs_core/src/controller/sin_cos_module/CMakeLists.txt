cmake_minimum_required(VERSION 3.5)
project(sin_cos_module)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rmcs_core REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  component_a.cpp
  component_b.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rmcs_core
  rclcpp
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES sin_cos_config.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()