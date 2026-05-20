cmake_minimum_required(VERSION 3.8)
project(mission_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models airframes launch scripts
        DESTINATION share/${PROJECT_NAME}/
        USE_SOURCE_PERMISSIONS)

ament_package()
