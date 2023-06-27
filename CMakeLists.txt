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

# If we debug
add_compile_options(-g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(ros2_parrot_arsdk REQUIRED)

find_package(PkgConfig)
pkg_check_modules(AVDEVICE REQUIRED libavdevice>=58)
pkg_check_modules(AVFORMAT REQUIRED libavformat>=58)
pkg_check_modules(AVCODEC REQUIRED libavcodec>=58)
pkg_check_modules(AVUTIL REQUIRED libavutil>=56)
pkg_check_modules(SWSCALE REQUIRED libswscale>=5)


add_executable(bebop_driver src/bebop_driver_node.cpp src/bebop.cpp src/video_decoder.cpp src/ardrone3_state_callbacks.cpp)
include_directories(include)
ament_target_dependencies(bebop_driver ros2_parrot_arsdk rclcpp std_msgs geometry_msgs nav_msgs camera_info_manager image_transport tf2 tf2_ros)
target_link_libraries(bebop_driver ${ARSDK_LIBS})
target_link_directories(bebop_driver PUBLIC ${ARSDK_LIB_DIR})
target_link_libraries(bebop_driver ${AVDEVICE_LIBRARIES} ${AVFORMAT_LIBRARIES} ${AVCODEC_LIBRARIES} ${AVUTIL_LIBRARIES} ${SWSCALE_LIBRARIES})
message("${AVDEVICE_LIBRARIES};${AVFORMAT_LIBRARIES};${AVCODEC_LIBRARIES};${AVUTIL_LIBRARIES};${SWSCALE_LIBRARIES}")


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

# Install Python executables
install(PROGRAMS
	scripts/joy_teleop.py
	DESTINATION lib/${PROJECT_NAME}
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
