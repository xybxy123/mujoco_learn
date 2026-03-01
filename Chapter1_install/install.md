# ===================== 第一步：编译安装Mujoco核心库 =====================
# 1. 克隆Mujoco源码仓库
git clone https://github.com/deepmind/mujoco.git

# 2. 进入Mujoco源码根目录
cd ~/mujoco  

# 3. 创建并进入编译目录
mkdir build && cd build

# 4. CMake配置
sudo cmake .. -DCMAKE_INSTALL_PREFIX=/opt/mujoco -DCMAKE_BUILD_TYPE=Release

# 5. 编译
sudo cmake --build . -j$(nproc)

# 6. 安装Mujoco到指定路径
sudo cmake --install .

# 7. 验证安装是否成功
ls /opt/mujoco/lib/cmake/mujoco/mujocoConfig.cmake

# ===================== 第二步：编译并运行Mujoco示例程序 =====================
# 1. 进入示例程序的编译目录
cd /path/to/mujoco_learn/Chapter1_install/build

# 2. 清空旧编译文件
rm -rf *

# 3. CMake配置示例程序
cmake .. 

# 4. 编译示例程序
make 

# 5. 运行示例程序
./mujoco_demo ../pointer.xml
