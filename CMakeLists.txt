cmake_minimum_required(VERSION 3.8)
project(ros2_lidar_georeference)

#########################################
# Dependencies
#########################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

#########################################
# Interface generation
#########################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FileList.msg"
  "msg/MeasurementCollect.msg"
  "srv/FileDelete.srv"
  # "msg/MeasurementFeedback.msg"
  # "srv/MeasurementResult.srv"
  # "srv/MeasurementGoal.srv"
  "srv/MeasurementProcess.srv"
  "action/Measurement.action"
)

#########################################
# File manager node
#########################################
add_executable(file_manager
  src/file_manager.cpp
)

ament_target_dependencies(file_manager
  rclcpp
)

#########################################
# Measurement collector (online)
#########################################
add_executable(measurement_collector
  src/measurement_collector.cpp
)

ament_target_dependencies(measurement_collector
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
)

#########################################
# Measurement processor (offline LAS writer)
#########################################
add_executable(measurement_processor
  src/measurement_processor.cpp
)

ament_target_dependencies(measurement_processor
  rclcpp
  nav_msgs
  tf2
)

#########################################
# Measurement manager (orchestrator)
#########################################
add_executable(measurement_manager
  src/measurement_manager.cpp
)

ament_target_dependencies(measurement_manager
  rclcpp
  rclcpp_action
)

#########################################
# Measurement WEB bridge
#########################################
add_executable(measurement_web_bridge
  src/measurement_web_bridge.cpp
)

ament_target_dependencies(measurement_web_bridge
  rclcpp
  rclcpp_action
  std_msgs
)

#########################################
# ROSIDL type support (Jazzy-correct)
#########################################
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(file_manager
  ${cpp_typesupport_target})

target_link_libraries(measurement_collector
  ${cpp_typesupport_target})

target_link_libraries(measurement_processor
  ${cpp_typesupport_target})

target_link_libraries(measurement_manager
  ${cpp_typesupport_target})

target_link_libraries(measurement_web_bridge
  ${cpp_typesupport_target})

#########################################
# Install targets
#########################################
install(TARGETS
  file_manager
  measurement_collector
  measurement_processor
  measurement_manager
  measurement_web_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY web/
  DESTINATION share/${PROJECT_NAME}/web
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
