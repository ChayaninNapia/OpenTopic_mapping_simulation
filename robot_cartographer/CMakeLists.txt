cmake_minimum_required(VERSION 3.8)
project(robot_cartographer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################ FIND DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)		# cpp (optional)
find_package(rclcpp REQUIRED)			# cpp (optional)
find_package(ament_cmake_python REQUIRED)   	# python (optional)
find_package(rclpy REQUIRED)                	# python (optional)
######################################################

# Install Python executables
install(PROGRAMS

  DESTINATION lib/${PROJECT_NAME}
)

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  # add directories here
  launch 
  rviz
  config
  DESTINATION share/${PROJECT_NAME})


ament_package()
