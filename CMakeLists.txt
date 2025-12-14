cmake_minimum_required(VERSION 3.8)
project(ros2_lidar_georeference)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

#########################################
# Interface generation
#########################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FileList.msg"
  "msg/MeasurementCollect.msg"
  "msg/MeasurementFeedback.msg"
  "srv/FileDelete.srv"
  "srv/MeasurementGoal.srv"
  "srv/MeasurementProcess.srv"
  "srv/MeasurementResult.srv"
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

add_executable(measurement_collector
  src/measurement_collector.cpp
)
ament_target_dependencies(measurement_collector
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)
rosidl_target_interfaces(measurement_collector
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
install(TARGETS measurement_collector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY web/
  DESTINATION share/${PROJECT_NAME}/web
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
