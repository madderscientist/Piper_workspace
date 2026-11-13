cmake_minimum_required(VERSION 3.16)
project(piper_x_driver)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Default build type" FORCE)
endif()

include(CheckIPOSupported)
check_ipo_supported(RESULT PIPER_IPO_SUPPORTED OUTPUT PIPER_IPO_ERROR)

set(PIPER_PERF_COMPILE_FLAGS
  -O3
  -DNDEBUG
  -DEIGEN_NO_DEBUG
  -fno-math-errno
  -fno-trapping-math
  -ffp-contract=fast
  -funroll-loops
  -Wall
  -Wextra
  -Wpedantic
)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$")
  list(APPEND PIPER_PERF_COMPILE_FLAGS -march=native -mtune=native)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(piper_x_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tinyxml2_vendor QUIET)
find_package(TinyXML2 QUIET)
find_library(TINYXML2_LIB tinyxml2 REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor QUIET)
find_package(pybind11 REQUIRED)

add_library(piper_x_driver_core STATIC
  src/terminal.cpp
  src/gravity.cpp
  src/friction.cpp
  src/configuration.cpp
  src/can_protocol.cpp
  src/socketcan_arm.cpp
  src/controller.cpp
  src/node.cpp
)
target_compile_features(piper_x_driver_core PUBLIC cxx_std_17)
set_target_properties(piper_x_driver_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(piper_x_driver_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_compile_definitions(piper_x_driver_core PUBLIC
  PIPER_X_DRIVER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(piper_x_driver_core PUBLIC
  yaml-cpp
  ${TINYXML2_LIB}
  Threads::Threads
)
ament_target_dependencies(piper_x_driver_core PUBLIC
  ament_index_cpp
  rclcpp
  piper_x_interfaces
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
)
target_compile_options(piper_x_driver_core PRIVATE ${PIPER_PERF_COMPILE_FLAGS})

add_executable(driver src/main.cpp)
target_link_libraries(driver PRIVATE piper_x_driver_core)
target_compile_options(driver PRIVATE ${PIPER_PERF_COMPILE_FLAGS})

add_executable(power_off src/power_off_main.cpp)
target_link_libraries(power_off PRIVATE piper_x_driver_core)
target_compile_options(power_off PRIVATE ${PIPER_PERF_COMPILE_FLAGS})

install(TARGETS driver power_off
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS piper_x_driver_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)

ament_python_install_package(${PROJECT_NAME})

pybind11_add_module(_core MODULE src/pybind_module.cpp)
target_link_libraries(_core PRIVATE piper_x_driver_core)
target_compile_options(_core PRIVATE ${PIPER_PERF_COMPILE_FLAGS})
install(TARGETS _core
  LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

if(PIPER_IPO_SUPPORTED)
  set_target_properties(piper_x_driver_core driver power_off _core PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION TRUE
  )
endif()

install(PROGRAMS scripts/damping
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md CAN_PROTOCOL.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cpp_driver test/test_cpp_driver.cpp)
  target_link_libraries(test_cpp_driver piper_x_driver_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(piper_x_driver_core yaml-cpp ${TINYXML2_LIB})
ament_export_dependencies(
  ament_index_cpp
  rclcpp
  piper_x_interfaces
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  Eigen3
)
ament_package()
