cmake_minimum_required(VERSION 3.5)
project(ros2_livox_simulation)
add_compile_options(-std=c++17)
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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(livox_ros_driver2 QUIET)
find_package(Boost REQUIRED COMPONENTS chrono)
# Handle JsonCpp target conflict more robustly
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
if(TARGET JsonCpp::JsonCpp)
  get_target_property(JSONCPP_TYPE JsonCpp::JsonCpp TYPE)
  if(NOT JSONCPP_TYPE STREQUAL "INTERFACE_LIBRARY")
    message(STATUS "JsonCpp::JsonCpp target already exists, reusing it")
  endif()
else()
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(JSONCPP jsoncpp QUIET)
  endif()
endif()

# Use a more specific approach for Gazebo dependencies
find_package(PkgConfig REQUIRED)
pkg_check_modules(GZ_SIM gz-sim8 REQUIRED)
pkg_check_modules(GZ_PLUGIN gz-plugin2 REQUIRED)
pkg_check_modules(GZ_MSGS gz-msgs10 REQUIRED)
pkg_check_modules(GZ_TRANSPORT gz-transport13 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  include
)



install(DIRECTORY include/
        DESTINATION include
)



add_library(ros2_livox SHARED src/livox_harmonic_plugin.cpp)
target_link_libraries(ros2_livox 
  ${GZ_SIM_LIBRARIES}
  ${GZ_PLUGIN_LIBRARIES}
  ${GZ_MSGS_LIBRARIES}
  ${GZ_TRANSPORT_LIBRARIES}
)
ament_target_dependencies(ros2_livox rclcpp std_msgs sensor_msgs geometry_msgs ros_gz_sim tf2_ros)
target_link_libraries(ros2_livox ${Boost_LIBRARIES})
target_include_directories(ros2_livox PRIVATE 
  include
  ${GZ_SIM_INCLUDE_DIRS}
  ${GZ_PLUGIN_INCLUDE_DIRS}
  ${GZ_MSGS_INCLUDE_DIRS}
  ${GZ_TRANSPORT_INCLUDE_DIRS}
)

# Add compile flags for Gazebo dependencies
target_compile_options(ros2_livox PRIVATE 
  ${GZ_SIM_CFLAGS_OTHER}
  ${GZ_PLUGIN_CFLAGS_OTHER}
  ${GZ_MSGS_CFLAGS_OTHER}
  ${GZ_TRANSPORT_CFLAGS_OTHER}
)

#install(TARGETS ros2_livox DESTINATION lib/${PROJECT_NAME})

install(TARGETS ros2_livox
    LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(
  ros2_livox
)

set(CSV_FILES
    scan_mode/mid40.csv
    scan_mode/mid70.csv
    scan_mode/mid360.csv
    scan_mode/tele.csv
    scan_mode/horizon.csv
    scan_mode/HAP.csv
    scan_mode/avia.csv
)
foreach(CSV_FILE ${CSV_FILES})
    configure_file(${CSV_FILE} ${CMAKE_CURRENT_BINARY_DIR}/${CSV_FILE} COPYONLY)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${CSV_FILE}
        DESTINATION share/${PROJECT_NAME}/scan_mode
    )
endforeach()

set(URDF_FILES
    urdf/mid40.xacro
    urdf/mid70.xacro
    urdf/mid360.xacro
    urdf/tele.xacro
    urdf/horizon.xacro
    urdf/HAP.xacro
    urdf/avia.xacro
)
foreach(URDF_FILES ${URDF_FILES})
    configure_file(${URDF_FILES} ${CMAKE_CURRENT_BINARY_DIR}/${URDF_FILES} COPYONLY)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${URDF_FILES}
        DESTINATION share/${PROJECT_NAME}/urdf
    )
endforeach()

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
