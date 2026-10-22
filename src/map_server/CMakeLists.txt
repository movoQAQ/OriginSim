cmake_minimum_required(VERSION 3.16)
project(map_server LANGUAGES C CXX)
option(MAP_SERVER_CORE_ONLY "Build pure C++ MapServer without ROS" OFF)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree io)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
add_library(map_server_core SHARED
  src/static_map.cpp src/dynamic_cloud_filter.cpp src/dynamic_memory.cpp
  src/radar_target_cache.cpp src/radar_rasterizer.cpp src/edt_2d.cpp
  src/costmap_generator.cpp src/map_server.cpp)
set_target_properties(map_server_core PROPERTIES OUTPUT_NAME origin_navigation_map_core)
target_compile_features(map_server_core PUBLIC cxx_std_17)
target_include_directories(map_server_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_include_directories(map_server_core SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_include_directories(map_server_core PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(map_server_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES}
  PRIVATE yaml-cpp ${OpenCV_LIBS} Threads::Threads)
target_compile_options(map_server_core PRIVATE -Wall -Wextra -Wpedantic)

if(NOT MAP_SERVER_CORE_ONLY)
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  set(ros_dependencies rclcpp nav_msgs std_msgs sensor_msgs geometry_msgs builtin_interfaces
    pcl_conversions tf2 tf2_ros tf2_geometry_msgs)
  foreach(dependency IN LISTS ros_dependencies)
    find_package(${dependency} REQUIRED)
  endforeach()
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/RadarTarget.msg" "msg/RadarTargets.msg" "msg/DynamicHealth.msg"
    "msg/DerivedMap.msg" "msg/DynamicObstacles.msg"
    "msg/StaticMap.msg" "msg/ObstacleCircle.msg" "msg/MapStatus.msg"
    DEPENDENCIES std_msgs nav_msgs geometry_msgs builtin_interfaces)
  rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
  add_library(map_server_ros SHARED src/map_geometry.cpp src/map_client.cpp
    ros/cloud_adapter.cpp ros/radar_adapter.cpp)
  target_compile_features(map_server_ros PUBLIC cxx_std_17)
  target_link_libraries(map_server_ros PUBLIC map_server_core ${typesupport_target})
  ament_target_dependencies(map_server_ros PUBLIC ${ros_dependencies})
  add_executable(map_server_node ros/map_server_node.cpp)
  target_link_libraries(map_server_node PRIVATE map_server_ros)
  install(TARGETS map_server_core map_server_ros EXPORT map_server_targets
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
  install(TARGETS map_server_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY include/ DESTINATION include)
  install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
  ament_export_targets(map_server_targets HAS_LIBRARY_TARGET)
  ament_export_include_directories(include)
  ament_export_dependencies(${ros_dependencies} rosidl_default_runtime Eigen3 PCL)
endif()

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  add_executable(test_map_server test/test_map_server.cpp)
  target_include_directories(test_map_server PRIVATE ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../navigation/topology_global_planner/include)
  target_link_libraries(test_map_server PRIVATE map_server_core GTest::gtest_main ${OpenCV_LIBS})
  add_test(NAME map_server_contracts COMMAND test_map_server)
  if(NOT MAP_SERVER_CORE_ONLY)
    add_executable(test_ros_adapters test/test_ros_adapters.cpp)
    target_link_libraries(test_ros_adapters PRIVATE map_server_ros GTest::gtest_main)
    add_test(NAME map_server_ros_adapters COMMAND test_ros_adapters)
    add_executable(test_map_client test/test_map_client.cpp)
    target_link_libraries(test_map_client PRIVATE map_server_ros GTest::gtest_main)
    add_test(NAME map_server_client_restart COMMAND test_map_client)
    set_tests_properties(map_server_client_restart PROPERTIES ENVIRONMENT
      "ROS_DOMAIN_ID=87;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_test_logs")
  endif()
  add_executable(benchmark_map_server test/benchmark_map_server.cpp)
  target_link_libraries(benchmark_map_server PRIVATE map_server_core)
endif()
if(NOT MAP_SERVER_CORE_ONLY)
  ament_package()
endif()
