cmake_minimum_required(VERSION 3.5)
project(bluerov2_ctrl_serv)

find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()