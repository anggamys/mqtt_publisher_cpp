cmake_minimum_required(VERSION 3.8)
project(mqtt_publisher_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== Find deps =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)   # <-- dipakai oleh headers
find_package(sensor_msgs REQUIRED)     # <-- kalau tak dipakai, boleh hapus
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
find_package(nlohmann_json REQUIRED)   # dari nlohmann-json3-dev

# Mosquitto via pkg-config sebagai imported target
pkg_check_modules(MOSQUITTO REQUIRED IMPORTED_TARGET libmosquitto)

# ===== Library internal (berisi implementasi handler/publisher) =====
add_library(${PROJECT_NAME} SHARED
  src/publisher.cpp
  src/acc_free_handler.cpp
  src/rpy_handler.cpp
)

# Export include publik dari folder include/
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Tambahkan dependensi ROS agar include & link otomatis
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  sensor_msgs
)

# Link lib eksternal
target_link_libraries(${PROJECT_NAME}
  PkgConfig::MOSQUITTO
  nlohmann_json::nlohmann_json
  Threads::Threads
)

# ===== Executable node =====
add_executable(mqtt_publisher_node src/main_node.cpp)
ament_target_dependencies(mqtt_publisher_node rclcpp)
target_link_libraries(mqtt_publisher_node ${PROJECT_NAME})

# ===== Install =====
install(TARGETS
  ${PROJECT_NAME}
  mqtt_publisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
