cmake_minimum_required(VERSION 3.8)
project(mqtt_publisher_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
find_package(nlohmann_json REQUIRED)   # <-- tambah ini

pkg_check_modules(MOSQUITTO REQUIRED IMPORTED_TARGET libmosquitto)

add_executable(mqtt_publisher_node src/mqtt_publisher_node.cpp)
ament_target_dependencies(mqtt_publisher_node rclcpp)

target_link_libraries(mqtt_publisher_node
  PkgConfig::MOSQUITTO
  nlohmann_json::nlohmann_json   # <-- link target header-only
  Threads::Threads
)

install(TARGETS mqtt_publisher_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
