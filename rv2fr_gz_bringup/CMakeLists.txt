cmake_minimum_required(VERSION 3.8)
project(rv2fr_gz_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds config models urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
