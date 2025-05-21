cmake_minimum_required(VERSION 3.15)
project(carla_global_planner)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)

# Install service definitions
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanGlobalPath.srv"
  DEPENDENCIES nav_msgs
)

# Install entry point script (if you have any)
install(PROGRAMS script/carla_global_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY utilities
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


