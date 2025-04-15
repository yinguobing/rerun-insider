cmake_minimum_required(VERSION 3.16...3.27)
project(rerun-insider LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Build options
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# # Download Rerun SDK
# include(FetchContent)
# FetchContent_Declare(
#   rerun_sdk
#   URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
# )
# FetchContent_MakeAvailable(rerun_sdk)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rerun_sdk REQUIRED)

# The node. Require C99 and C++17.
add_executable(insider src/main.cpp src/engine.cpp)
target_include_directories(insider PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(insider rerun_sdk)
target_compile_features(insider PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(insider rclcpp sensor_msgs cv_bridge nav_msgs)

# Installation
install(TARGETS insider DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
