cmake_minimum_required(VERSION 3.8)
project(lidar2osm_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_msgs REQUIRED)                   # For srv inputs
find_package(Eigen3 REQUIRED)                      #
find_package(rosidl_default_generators REQUIRED)  #
find_package(message_filters REQUIRED)            # For ensuring two global maps are sync'd
# find_package(clipper REQUIRED)                    # For map-merging :D

# Headers
include_directories(${EIGEN3_INCLUDE_DIRS})
# include_directories(include)

# ==========================  Interfaces ==========================
# set(srv_files
#   "srv/RegisterMap.srv"
# )

# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${srv_files}
#   DEPENDENCIES std_msgs
# )

# ament_export_dependencies(rosidl_default_runtime)

# ==========================  Executables ==========================

# # Robot distance checker node
# add_executable(robot_distance_checker src/robot_distance_checker.cpp src/rviz_marker_utils.cpp)
# ament_target_dependencies(robot_distance_checker rclcpp sensor_msgs nav_msgs tf2_ros visualization_msgs)

# # Robot map accumulator node
# add_executable(robot_map_accumulator src/robot_map_accumulator.cpp)
# ament_target_dependencies(robot_map_accumulator rclcpp sensor_msgs nav_msgs tf2_ros tf2_geometry_msgs tf2_eigen pcl_conversions)

# # Robot map reciever node
# add_executable(robot_map_reciever src/robot_map_reciever.cpp)
# ament_target_dependencies(robot_map_reciever rclcpp sensor_msgs nav_msgs tf2_ros tf2_eigen pcl_conversions clipper)

# # Example server/client nodes
# add_executable(map_registration_server src/map_registration_server.cpp)
# ament_target_dependencies(map_registration_server rclcpp std_msgs)

# add_executable(map_registration_client src/map_registration_client.cpp)
# ament_target_dependencies(map_registration_client rclcpp std_msgs)

# target_link_libraries(robot_map_accumulator
#   tf2
#   tf2_ros
#   tf2_eigen
# )

# ==========================  Install shared files ==========================

# # Install the headers
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}/
# )

# Install URDF in share dir
install(DIRECTORY urdf/
    DESTINATION share/${PROJECT_NAME}/urdf
)

# Install rviz dir in share
install(DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz
)

# Install the launch files
install(DIRECTORY launch 
    DESTINATION share/${PROJECT_NAME}/launch
)

# # Install the node executable
# install(TARGETS
#   robot_distance_checker
#   robot_map_accumulator
#   robot_map_reciever
#   map_registration_server
#   map_registration_client
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Install config dir
# install(
#   DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/
# )

# ========================== Link Libraries ==========================

# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
# target_link_libraries(map_registration_server "${cpp_typesupport_target}")
# target_link_libraries(map_registration_client "${cpp_typesupport_target}")

# target_link_libraries(robot_map_reciever
#   ${catkin_LIBRARIES}
#   clipper
# )

# ========================== Test / build ==========================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
