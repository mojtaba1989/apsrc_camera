cmake_minimum_required(VERSION 3.8)
project(apsrc_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV 4.5 REQUIRED)

set(ROS2_DEPS
    sensor_msgs
    apsrc_msgs
    std_msgs
    rclcpp
    cv_bridge
)

foreach(pkg ${ROS2_DEPS})
    find_package(${pkg} REQUIRED)  
endforeach()

add_executable(webcam_node src/webcam_node.cpp)

target_include_directories(webcam_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(webcam_node ${ROS2_DEPS})
target_link_libraries(webcam_node ${OpenCV_LIBRARIES})

install(TARGETS
  webcam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()