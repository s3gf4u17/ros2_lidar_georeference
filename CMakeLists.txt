cmake_minimum_required(VERSION 3.8)
project(ros2_lidar_georeference)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#########################################
# Interface generation
#########################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FileList.msg"
  "srv/FileDelete.srv"
)

#########################################
# Node
#########################################
add_executable(file_manager
  src/file_manager.cpp
)

ament_target_dependencies(file_manager
  rclcpp
)

rosidl_target_interfaces(file_manager
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS file_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY web/
  DESTINATION share/${PROJECT_NAME}/web
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
