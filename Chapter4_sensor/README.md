# Chapter3_ctrl 控制模块指南

本模块 (`Chapter3_ctrl`) 是一个关于 **“如何使用 MuJoCo C++ API 编写四足机器人步态控制”** 的最小化演示核心项目。

通过这个项目，你将了解：一只能在 MuJoCo 仿真中跑起来的狗，在代码层面到底是怎样串联起来的。

---

## 1. 项目核心思想

该项目的核心目标是**跑通控制全流程**，它实现了：
1. 读取四足机器狗的 XML 模型。
2. 通过 C++ 编写 MuJoCo 控制回调函数 (`mjcb_control`)。
3. 产生小跑步态（Trot）的足端轨迹。
4. 解算二维二连杆逆运动学（IK），把足端目标位置转化为关节的期望角度。
5. 将这些期望角度发送给模型中定义的执行器（Position Actuators）。

为了保持代码的易读性，当前版本：
- **开环控制**：不读取任何传感器反馈（不判断姿态平衡和足端是否真的触地）。
- **固定步态**：固定朝着一个方向（当前配置为前进）以稳定节拍输出 trot 步态。

## 2. 目录结构

```text
Chapter3_ctrl/
├── CMakeLists.txt          # CMake 构建配置文件
├── README.md               # 本说明文档
├── assets/
│   └── xml_test.xml        # 四足机器人的 MuJoCo 宏模型定义文件
├── src/
│   ├── main.cpp            # 主循环，负责初始化 GLFW 窗口、加载模型并注册控制回调
│   ├── gait_controller.h   # 步态控制器定义，负责将步态时间映射为关节角
│   ├── gait_controller.cpp # 步态发生器及参数的核心实现
│   ├── leg_ik.h            # 腿部逆运动学相关定义
│   └── leg_ik.cpp          # 二连杆几何逆运动学方程求解
└── build_local/            # 编译目录
```

## 3. 代码是怎么运转的（数据流向）

1. **入口启动 (`main.cpp`)**
   - 调用 `mj_loadXML` 加载并解析机器狗。
   - 调用 `g_controller.SetInitialPose` 进行初始站立位置的分配，防止一开局就摔倒。
   - 将 `mycontroller` 绑定给 MuJoCo 的回调指针 `mjcb_control`。
   - 进入 `while` 循环，持续步进 `mj_step` 和渲染画面。

2. **步步触发 (`mycontroller` -> `g_controller.Update`)**
   - 每次 `mj_step` 计算物理引擎受力之前，MuJoCo 会自动调用 `mycontroller`。
   - 然后进入到 `GaitController::Update` 中，通过 `data->time` 计算当前腿处于什么相位（Phase）。

3. **从相位到关节 (`gait_controller.cpp` & `leg_ik.cpp`)**
   - **`computeTrotTarget`**：根据周期把一段运动分成 **支撑相**（向后蹬地推进）与 **摆动相**（抬腿往前迈）。得出当前足端目标在躯干下方的坐标 `(x, z)`。
   - **`leg_ik_.Solve`**：将 `(x, z)` 送入平面几何构型中，通过余弦定理求出此时需要的 **臀部角度(hip)** 和 **膝部角度(knee)**。
   - **执行器赋值**：算出目标角度后，将值写入 `data->ctrl`。这实际上是通过 MuJoCo 的 `<position .../>` 驱动器给关节施加转矩，使得其跟踪目标角度。

## 4. 重点参数与调节指南

如果你想玩一玩项目，绝大多参数都写直在了 `src/gait_controller.cpp` 开头的 `namespace` 中。你可以修改后重新编译来查看效果：

| 参数 | 物理意义 | 如何调节 |
|---|---|---|
| `kForwardStepSign` | 步态前进方向 | `-1.0` 表示向前；`1.0` 表示向后倒退 |
| `kStepLength` | 迈步步长 | 减小则走的更碎；增大能提速，但太大容易劈叉 |
| `kStepHeight` | 抬腿高度 | 针对跨越障碍物有用，平地通常设小一点 |
| `kDutyFactor` | 支撑相所占的时间比例 | `0.58` 代表脚有 58% 的时间在地上。数值越大跑得越保守稳健 |
| `kCyclePeriod`| 走完一个完整步伐所需的周期时间 | 把时间调小频率就会提升，机器狗踩单车会变快 |
| `kStandingBodyHeight` | 初始机身离地高度 | 可以适当调节看看能不能趴低一点走 |

## 5. 如何编译与运行

在命令行终端中操作：

```bash
cd /home/cc/genesis_ws/mujoco_learn/Chapter3_ctrl
mkdir build_local && cd build_local
cmake ..
cmake --build .
```

构建成功后，在 `build_local` 目录下执行：
```bash
./mj_ctrl
```
_注：必须在构建目录下执行，因为代码内部找相对路径是 `../assets/xml_test.xml`。_

## 6. 交互说明

在弹出的 MuJoCo 原生窗口中：
- `鼠标左键` + 拖动：旋转摄像机视角
- `鼠标右键` + 拖动：平移摄像机视角
- `滚轮`：拉近/拉远
- 键盘 `Backspace` (退格键)：快速一键重置机器狗的姿态

## 7. 进一步学习的建议 (TODO方向)

当前只是个很初级的演示，你可以在阅读理解并彻底消化后，向这些方向扩充：
- **速度控制**：通过传入摇杆/键盘指令来动态修改 `kStepLength`。
- **转向控制**：通过不同的偏航速度偏置，让左侧腿和右侧腿迈步长度产生内轮差外轮差，实现转弯。
- **添加感知**：从 `mjData` 里读取机身的四元数/欧拉角倾角，引入简单的 PD 控制叠加在位置目标上。