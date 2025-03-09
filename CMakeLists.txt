cmake_minimum_required(VERSION 3.0.2)
project(apsrc_camera)

## c++11 feature
set(CMAKE_CXX_FLAGS "-std=c++14 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

set(catkin_deps
    sensor_msgs
    apsrc_msgs
    roscpp
    roslib
    roslint
    std_msgs
    cv_bridge
)

find_package(catkin REQUIRED ${catkin_deps})
find_package(Boost REQUIRED)
find_package(OpenCV 3.2 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${catkin_deps}
)

roslint_cpp()


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# cv_to_ros
add_executable(webcam_node src/webcam_node.cpp)
target_link_libraries(webcam_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

############
# Install ##
############

install(TARGETS
    webcam_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)