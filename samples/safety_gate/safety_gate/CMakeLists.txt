cmake_minimum_required(VERSION 3.8)
project(safety_gate)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(safety_gate_logic src/Safety_gate.cpp)
target_include_directories(safety_gate_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(safety_gate_logic ${PCL_LIBRARIES} yaml-cpp)

add_executable(safety_gate_node src/Safety_gate_node.cpp)
target_include_directories(safety_gate_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(safety_gate_node safety_gate_logic ${PCL_LIBRARIES})
ament_target_dependencies(safety_gate_node
  ament_index_cpp geometry_msgs pcl_conversions rclcpp sensor_msgs)

install(TARGETS safety_gate_logic safety_gate_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_safety_gate test/test_safety_gate.cpp)
  target_link_libraries(test_safety_gate safety_gate_logic ${PCL_LIBRARIES})
  target_include_directories(test_safety_gate PRIVATE ${PCL_INCLUDE_DIRS})
  target_compile_definitions(test_safety_gate PRIVATE TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/config.yaml")
endif()

ament_export_include_directories(include)
ament_export_libraries(safety_gate_logic)
ament_package()
