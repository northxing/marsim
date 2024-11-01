cmake_minimum_required(VERSION 3.5)
project(marsim_gazebo)

find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-sensors8 REQUIRED)
find_package(gz-rendering8 REQUIRED)

# Set RTTI flag for the entire project
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frtti")

set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})
set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})
set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})

# Add the plugin libraries
add_library(GaussMarkovOceanCurrent SHARED plugins/GaussMarkovOceanCurrent.cc)
add_library(DVLRosConverter SHARED plugins/DVLRosConverter.cc)

target_include_directories(GaussMarkovOceanCurrent PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/plugins
  $<TARGET_PROPERTY:gz-sim${GZ_SIM_VER},INTERFACE_INCLUDE_DIRECTORIES>
  $<TARGET_PROPERTY:gz-transport${GZ_TRANSPORT_VER},INTERFACE_INCLUDE_DIRECTORIES>
  $<TARGET_PROPERTY:gz-msgs${GZ_MSGS_VER},INTERFACE_INCLUDE_DIRECTORIES>
  $<TARGET_PROPERTY:gz-sensors${GZ_SENSORS_VER},INTERFACE_INCLUDE_DIRECTORIES>
  $<TARGET_PROPERTY:gz-rendering${GZ_RENDERING_VER},INTERFACE_INCLUDE_DIRECTORIES>
  $<TARGET_PROPERTY:gz-plugin${GZ_PLUGIN_VER},INTERFACE_INCLUDE_DIRECTORIES>
)
target_include_directories(DVLRosConverter PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/plugins
  $<TARGET_PROPERTY:gz-sim${GZ_SIM_VER},INTERFACE_INCLUDE_DIRECTORIES>
  $<TARGET_PROPERTY:gz-transport${GZ_TRANSPORT_VER},INTERFACE_INCLUDE_DIRECTORIES>
  $<TARGET_PROPERTY:gz-msgs${GZ_MSGS_VER},INTERFACE_INCLUDE_DIRECTORIES>
  $<TARGET_PROPERTY:gz-plugin${GZ_PLUGIN_VER},INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(GaussMarkovOceanCurrent
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)
target_link_libraries(DVLRosConverter
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)

# Install the plugin
install(TARGETS 
  GaussMarkovOceanCurrent
  DVLRosConverter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install plugin header
install(
  FILES 
    plugins/GaussMarkovOceanCurrent.hh
    plugins/DVLRosConverter.hh
  DESTINATION include/${PROJECT_NAME}
)

# Install directories
install(
  DIRECTORY config launch models rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python programs
install(PROGRAMS scripts/ros_keyboard_controller.py DESTINATION lib/${PROJECT_NAME} RENAME ros_keyboard_controller)
install(PROGRAMS scripts/twist_stamper.py DESTINATION lib/${PROJECT_NAME} RENAME twist_stamper)
install(PROGRAMS scripts/noisy_odometry.py DESTINATION lib/${PROJECT_NAME} RENAME noisy_odometry)
install(PROGRAMS scripts/depth_sensor_from_odom.py DESTINATION lib/${PROJECT_NAME} RENAME depth_sensor_from_odom)

# Create symlinks for Python scripts
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink 
     ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/ros_keyboard_controller
     ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/ros_keyboard_controller.py)")
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
     ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/twist_stamper
     ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/twist_stamper.py)")
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
     ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/noisy_odometry
     ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/noisy_odometry.py)")
     install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
     ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/depth_sensor_from_odom
     ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/depth_sensor_from_odom.py)")

# Hook to define the environment paths for 'models' 
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/marsim_gazebo.sh.in")

ament_package()