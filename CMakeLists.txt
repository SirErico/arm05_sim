cmake_minimum_required(VERSION 3.8)
project(arm05_sim)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SearchAruco.action"
)

install(DIRECTORY
  worlds
  models
  launch
  map
  param
  DESTINATION share/${PROJECT_NAME}/
)

ament_python_install_package(scripts_arm05)

install(PROGRAMS
  scripts_arm05/spawn_aruco.py
  scripts_arm05/aruco_search_action_server.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
