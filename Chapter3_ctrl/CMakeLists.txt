cmake_minimum_required(VERSION 3.10)
project(Chapter3_ctrl)

set(CMAKE_CXX_STANDARD 17)

# 寻找 MuJoCo 依赖包 (假设 Mujoco 已经被正确安装并添加到了环境变量)
find_package(mujoco REQUIRED)

# 寻找 GLFW，用于图形界面渲染
find_package(glfw3 REQUIRED)

# 共享模块（两个可执行文件都需要）
set(SHARED_SOURCES
    src/gait_controller.cpp
    src/leg_ik.cpp
)

# mj_ctrl：原始固定步态演示（main.cpp）
add_executable(mj_ctrl
    src/main.cpp
    ${SHARED_SOURCES}
)
target_include_directories(mj_ctrl PRIVATE src)
target_link_libraries(mj_ctrl PRIVATE mujoco::mujoco glfw)

# mj_test：键盘控制前进/后退演示（test.cpp）
add_executable(mj_test
    src/test.cpp
    src/leg_ik.cpp
)
target_include_directories(mj_test PRIVATE src)
target_link_libraries(mj_test PRIVATE mujoco::mujoco glfw)