cmake_minimum_required(VERSION 3.5)
project(ros2_bebop_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(ros2_parrot_arsdk REQUIRED)

add_executable(bebop_driver src/bebop_driver_node.cpp src/bebop.cpp)
include_directories(include)
ament_target_dependencies(bebop_driver ros2_parrot_arsdk rclcpp std_msgs camera_info_manager image_transport)
target_link_libraries(bebop_driver ${ARSDK_LIBS})
target_link_directories(bebop_driver PUBLIC ${ARSDK_LIB_DIR})


target_include_directories(bebop_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS bebop_driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
	config
	DESTINATION share/${PROJECT_NAME}
	)

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
