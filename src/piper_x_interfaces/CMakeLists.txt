cmake_minimum_required(VERSION 3.8)
project(piper_x_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmStatus.msg"
  "msg/JointCommand.msg"
  "msg/CartesianCommand.msg"
  "msg/MitCommand.msg"
  "msg/GripperCommand.msg"
  "srv/Enable.srv"
  "srv/SetMode.srv"
  "srv/ConfigureArm.srv"
  "srv/DebugMit.srv"
  "srv/CalibrateEffector.srv"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
