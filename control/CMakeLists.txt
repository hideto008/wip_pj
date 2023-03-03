cmake_minimum_required(VERSION 3.8)
project(control)

# find dependencies
find_package(ament_cmake_auto REQUIRED)

# find dependency from package.xml
ament_auto_find_build_dependencies()

# define executable
ament_auto_add_executable(balance_controller src/balance_controller.cpp)

# automate 
ament_auto_package(
  INSTALL_TO_SHARE
  config
)
