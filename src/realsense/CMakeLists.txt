cmake_minimum_required(VERSION 3.18)
project(realsense)

set(CMAKE_CXX_STANDARD 20)


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Realsense2 REQUIRED)

add_executable(realsense realsense.cpp)
target_include_directories(realsense PUBLIC include)
target_link_libraries(realsense realsense2::realsense2 ${OpenCV_LIBS})
ament_target_dependencies(realsense rclcpp std_msgs sensor_msgs tf2_ros)


install(TARGETS
        realsense
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()