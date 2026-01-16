cmake_minimum_required(VERSION 3.18)
project(realsense_utils)

set(CMAKE_CXX_STANDARD 20)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include(FetchContent)
set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)
FetchContent_Declare(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG 8d2bee75ecbe1321c3f333345c85dc74527de900
)
FetchContent_MakeAvailable(threepp)

add_subdirectory(${threepp_SOURCE_DIR}/examples/external ${CMAKE_CURRENT_BINARY_DIR}/threepp_examples_external)



add_executable(image_visualizer image_visualizer.cpp)
target_include_directories(image_visualizer PUBLIC include)
target_link_libraries(image_visualizer ${OpenCV_LIBS})
ament_target_dependencies(image_visualizer rclcpp std_msgs sensor_msgs)

add_executable(points_visualizer points_visualizer.cpp)
target_include_directories(points_visualizer PUBLIC include)
target_link_libraries(points_visualizer threepp::threepp)
ament_target_dependencies(points_visualizer rclcpp std_msgs sensor_msgs)


install(TARGETS
        image_visualizer
        points_visualizer
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()