cmake_minimum_required(VERSION 3.10)
project(Chapter3_ctrl)

set(CMAKE_CXX_STANDARD 17)

# 寻找 MuJoCo 依赖包 (假设 Mujoco 已经被正确安装并添加到了环境变量)
find_package(mujoco REQUIRED)

# 寻找 GLFW，用于图形界面渲染
find_package(glfw3 REQUIRED)

# 添加你的源代码源文件
file(GLOB SOURCES "src/*.cpp" "src/*.c")
add_executable(mj_ctrl ${SOURCES})

# 添加头文件路径
target_include_directories(mj_ctrl PRIVATE src)

# 链接 MuJoCo 和 GLFW
target_link_libraries(mj_ctrl PRIVATE mujoco::mujoco glfw)