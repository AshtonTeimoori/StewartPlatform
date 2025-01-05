cmake_minimum_required(VERSION 3.16.3)

project(TestPlatform)
# set(CMAKE_BUILD_TYPE Debug) 

add_executable(${PROJECT_NAME} main.cpp)

add_subdirectory(include)

target_link_libraries(${PROJECT_NAME} platform)
