cmake_minimum_required(VERSION 3.5)
project(ros2_bebop_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(ros2_parrot_arsdk REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the states from the arsdk xml files
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
include(generate_from_xml)

SET(ARSDKXML_HASH ab28dab91845cd36c4d7002b55f70805deaff3c8)

# Generate the msg/headers and documentation files from the arsdk xml files
generate_states("common.xml")
generate_states("ardrone3.xml")
generate_settings("ardrone3.xml")

# Adds the install rules for all the generated files
file(GLOB GENERATED_MSG_FILES
	LIST_DIRECTORIES false RELATIVE ${CMAKE_BINARY_DIR} "${CMAKE_BINARY_DIR}/msg/*.msg")
# Rewrite the list to match the required format of  ABSOLUTE_PATH:RELATIVE_PATH_TO_MSG
foreach(_msg ${GENERATED_MSG_FILES})
	list(APPEND msg_files_to_process ${CMAKE_BINARY_DIR}:${_msg})
endforeach()
# Then trigger the interface generation
rosidl_generate_interfaces(${PROJECT_NAME}
	${msg_files_to_process}
	)


add_executable(bebop_driver src/bebop_driver_node.cpp src/bebop.cpp)
include_directories(include ${CMAKE_BINARY_DIR}/include/)
ament_target_dependencies(bebop_driver ros2_parrot_arsdk)
target_link_libraries(bebop_driver ${ARSDK_LIBS})
target_link_directories(bebop_driver PUBLIC ${ARSDK_LIB_DIR})

target_include_directories(bebop_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS bebop_driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME})

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
