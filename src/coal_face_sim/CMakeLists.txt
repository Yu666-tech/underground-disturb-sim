cmake_minimum_required(VERSION 3.8)
project(coal_face_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)

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

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/shearer_truth_tf.py DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/shearer_imu.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY scenarios/coal_face_v1/models scenarios/coal_face_v1/worlds
  DESTINATION share/${PROJECT_NAME}/scenarios/coal_face_v1)



find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
add_library(coal_face_trajectory SHARED src/coal_face_trajectory.cpp)
target_compile_features(coal_face_trajectory PUBLIC cxx_std_17)
target_include_directories(coal_face_trajectory PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(coal_face_trajectory ${GAZEBO_LIBRARIES})
ament_target_dependencies(coal_face_trajectory gazebo_ros rclcpp nav_msgs)
install(TARGETS coal_face_trajectory LIBRARY DESTINATION lib)

ament_package()
