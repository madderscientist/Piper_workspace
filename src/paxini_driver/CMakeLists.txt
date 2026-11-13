cmake_minimum_required(VERSION 3.16)
project(paxini_driver)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Default build type" FORCE)
endif()

include(CheckIPOSupported)
check_ipo_supported(RESULT PAXINI_IPO_SUPPORTED OUTPUT PAXINI_IPO_ERROR)

set(PAXINI_PERF_COMPILE_FLAGS
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
  list(APPEND PAXINI_PERF_COMPILE_FLAGS -march=native -mtune=native)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(piper_x_driver REQUIRED)
find_package(Threads REQUIRED)

add_library(paxini_driver_core STATIC
  src/protocol.cpp
  src/transport.cpp
  src/calibration.cpp
  src/configuration.cpp
  src/node.cpp
)
target_compile_features(paxini_driver_core PUBLIC cxx_std_17)
target_include_directories(paxini_driver_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(paxini_driver_core PUBLIC yaml-cpp Threads::Threads)
ament_target_dependencies(paxini_driver_core PUBLIC
  ament_index_cpp
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  piper_x_driver
)
target_compile_options(paxini_driver_core PRIVATE ${PAXINI_PERF_COMPILE_FLAGS})

add_executable(driver src/main.cpp)
target_link_libraries(driver PRIVATE paxini_driver_core)
target_compile_options(driver PRIVATE ${PAXINI_PERF_COMPILE_FLAGS})

add_executable(paxini_driver_exec src/main.cpp)
set_target_properties(paxini_driver_exec PROPERTIES OUTPUT_NAME paxini_driver)
target_link_libraries(paxini_driver_exec PRIVATE paxini_driver_core)
target_compile_options(paxini_driver_exec PRIVATE ${PAXINI_PERF_COMPILE_FLAGS})

if(PAXINI_IPO_SUPPORTED)
  set_target_properties(paxini_driver_core driver paxini_driver_exec PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION TRUE
  )
endif()

install(TARGETS driver paxini_driver_exec
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/calibrate
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_driver test/test_driver.cpp)
  target_link_libraries(test_driver paxini_driver_core)
  target_compile_definitions(test_driver PRIVATE
    PAXINI_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )

  ament_add_pytest_test(test_paxini_driver test/test_paxini_driver.py
    TIMEOUT 60
  )
endif()

ament_package()
