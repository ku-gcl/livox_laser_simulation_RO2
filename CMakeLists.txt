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
# find_package(gazebo_dev REQUIRED)
# find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS chrono)
# find_package(jsoncpp CONFIG REQUIRED)

# gz sim
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)
# find_package(gz-sim8 REQUIRED)
# find_package(gz-plugin2 REQUIRED )
# find_package(Protobuf REQUIRED)
# include_directories(
#   include
#   ${gz-sim8_INCLUDE_DIRS}
#   ${gz-plugin2_INCLUDE_DIRS}
#   # ${Boost_INCLUDE_DIRS}
# )

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# include_directories(
#   include
# )
# include_directories(${GAZEBO_INCLUDE_DIRS})
# include_directories(/usr/include/gazebo-11/gazebo)
# link_directories(${GAZEBO_LIBRARY_DIRS})



install(DIRECTORY include/
        DESTINATION include
)



add_library(ros2_livox SHARED src/livox_points_plugin.cpp src/livox_ode_multiray_shape.cpp)
target_link_libraries(ros2_livox ${GAZEBO_LIBRARIES} RayPlugin GpuRayPlugin)
# ament_target_dependencies(ros2_livox 
#   rclcpp std_msgs sensor_msgs geometry_msgs gazebo_dev gazebo_ros tf2_ros 
#   livox_ros_driver2 
# )
# target_link_libraries(ros2_livox libprotobuf.so.9)
# target_link_libraries(ros2_livox libboost_chrono.so.1.71.0)
# target_include_directories(ros2_livox PRIVATE include)

ament_target_dependencies(ros2_livox
  rclcpp std_msgs sensor_msgs geometry_msgs tf2_ros livox_ros_driver2
  ros_gz_bridge ros_gz_sim Protobuf
)
# target_link_libraries(ros2_livox
#   gz-sim8::core
#   gz-plugin2::register
#   ${Boost_LIBRARIES}
#   Protobuf::libprotobuf
# )
# target_include_directories(ros2_livox PRIVATE
#   ${Boost_INCLUDE_DIRS}
# )

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

ament_package()
