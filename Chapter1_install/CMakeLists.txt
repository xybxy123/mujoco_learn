cmake_minimum_required(VERSION 3.16)
project(test_mujoco)

# 设置C++17（MuJoCo要求）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MUJOCO_FOLDER /opt/mujoco/lib/cmake)
find_package(mujoco REQUIRED PATHS ${MUJOCO_FOLDER} NO_DEFAULT_PATH)
if (mujoco_FOUND)
    message(STATUS "Find mujoco:  ${mujoco_INCLUDE_DIRS}, ${mujoco_LIBRARIES}")
endif()

add_executable(mujoco_demo main.cpp)

target_link_libraries(mujoco_demo mujoco::mujoco glut GL GLU glfw)

# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/simulate)
# link_directories(${CMAKE_CURRENT_SOURCE_DIR}/simulate)
# file(GLOB SIMULATE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/simulate/*.cc)
# add_executable(simulate ${SIMULATE_SRC})
# target_link_libraries(simulate mujoco::mujoco glut GL GLU glfw lodepng)