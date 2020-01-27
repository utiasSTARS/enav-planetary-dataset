cmake_minimum_required(VERSION 2.8.3)
project(enav_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    tf2_ros
    std_msgs
    message_generation
    grid_map_ros
    grid_map_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    Pyranometer.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

##F C++ executables
add_executable(omni_tf_broadcaster src/omni_tf_broadcaster.cpp)
target_link_libraries(omni_tf_broadcaster ${catkin_LIBRARIES})

add_executable(site_loader_node src/site_loader_node.cpp)
target_link_libraries(site_loader_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
