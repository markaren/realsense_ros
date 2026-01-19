cmake_minimum_required(VERSION 3.18)
project(realsense_bringup)


find_package(ament_cmake REQUIRED)


install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

install(
        DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
