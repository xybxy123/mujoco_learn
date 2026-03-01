# MuJoCo 编译与安装指南

本文档介绍了如何在 Linux 环境下从源码编译安装 MuJoCo 核心库，并运行示例 Demo 程序。

---

## 一、 编译安装 MuJoCo 核心库
这一阶段将 MuJoCo 作为底层库安装到系统的 `/opt/mujoco` 目录下。

```bash
# 克隆 Mujoco 源码仓库
git clone https://github.com/deepmind/mujoco.git

# 进入源码根目录
cd mujoco  

# 创建并进入编译目录
mkdir build && cd build

# CMake 配置
sudo cmake .. -DCMAKE_INSTALL_PREFIX=/opt/mujoco -DCMAKE_BUILD_TYPE=Release

# 执行编译
sudo cmake --build . -j$(nproc)

# 安装到指定路径
sudo cmake --install .

# 验证安装
ls /opt/mujoco/lib/cmake/mujoco/mujocoConfig.cmake
```

## 二、 编译并运行示例程序

在核心库安装完成后，即可编译仿真项目

```bash
# 进入示例程序的编译目录
cd /path/to/mujoco_learn/Chapter1_install/build

# 清空旧编译文件
rm -rf *

# CMake 配置示例程序
cmake .. 

# 编译生成可执行程序
make

# 运行示例程序，加载模型文件
./mujoco_demo ../pointer.xml

```