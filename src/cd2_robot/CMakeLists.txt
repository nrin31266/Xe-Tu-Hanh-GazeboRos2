cmake_minimum_required(VERSION 3.8)
project(cd2_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Cài đặt các thư mục resource
install(DIRECTORY
  urdf
  worlds
  launch
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
