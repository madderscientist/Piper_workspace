cmake_minimum_required(VERSION 3.8)
project(piper_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch meshes rviz urdf mujoco_model
  DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
install(PROGRAMS piper_description/model_state.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_state_relay)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(description test/test_description.py
    ENV ROS_DOMAIN_ID=215
    TIMEOUT 60)
endif()

ament_package()
