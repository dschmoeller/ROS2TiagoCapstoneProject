cmake_minimum_required(VERSION 3.5)
project(tiago_explore)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)



set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  custom_interfaces
  geometry_msgs
)


include_directories(include)


add_library(${PROJECT_NAME} SHARED
  src/tiago_explore/See.cpp
  src/tiago_explore/Think.cpp
  src/tiago_explore/Act.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})


add_executable(explore src/tiago_explore_main.cpp)
ament_target_dependencies(explore ${dependencies})
target_link_libraries(explore ${PROJECT_NAME})


install(TARGETS
  ${PROJECT_NAME}
  explore
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY include
  DESTINATION include
)


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
