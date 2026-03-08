# MuJoCo Joint 操作 API 指南
本文档分类整理 MuJoCo 中与 Joint（关节）相关的核心 API，涵盖查询关节列表、读取关节状态、设置关节状态三大场景，并提供实用示例和核心逻辑说明，适配实际开发需求。

---

## 一、查看 Joint 列表/信息（静态模型查询）
这类函数用于获取模型中关节的元信息（数量、名称、ID、属性等），核心操作对象是 `mjModel`（存储模型静态结构）。

| 函数名                | 功能说明                                                                 |
|-----------------------|--------------------------------------------------------------------------|
| `mj_name2id`          | 核心：通过关节名称获取 ID（需指定 `objtype=mjOBJ_JOINT`），验证关节是否存在 |
| `mj_id2name`          | 通过关节 ID 获取名称，遍历 ID 可列出所有关节                             |
| `mjs_findJoint`       | 直接查找指定名称的关节元素（动态修改模型时常用）                         |
| `mjs_firstChild/mjs_nextChild` | 遍历 body 子元素，筛选出关节类型（关节是 body 的子元素）                |
| `mjs_firstElement/mjs_nextElement` | 遍历模型所有元素，筛选出关节类型                                      |
| `mj_printModel/mj_printFormattedModel` | 打印模型完整信息，包含所有关节参数（调试用）                         |
| `mjs_getName`         | 获取指定关节元素的名称（动态模型修改场景）                               |

### 实用示例（C 语言）
```c
// 遍历所有关节并打印名称
// m 为 mjModel 指针，存储模型静态结构
int njnt = m->njnt; // 获取模型中关节总数
for (int i = 0; i < njnt; i++) {
    char name[100];
    // 通过ID获取关节名称，参数：模型、对象类型、ID、存储名称的数组、数组长度
    mj_id2name(m, mjOBJ_JOINT, i, name, 100);
    printf("Joint %d: %s\n", i, name);
}

// 通过名称查询关节ID（验证关节是否存在）
int joint1_id = mj_name2id(m, mjOBJ_JOINT, "joint_link1");
if (joint1_id >= 0) {
    printf("关节 joint_link1 存在，ID：%d\n", joint1_id);
} else {
    printf("关节 joint_link1 不存在\n");
}
```

## 二、读取 Joint 状态（实时动态数据）
这类操作用于获取仿真过程中关节的位置、速度、力矩等动态状态，核心操作对象是 `mjData`（存储模型运行时状态）。

| 函数名                | 功能说明                                                                 |
|-----------------------|--------------------------------------------------------------------------|
| `mj_sensorPos`        | 底层函数：计算关节位置传感器数据（传感器依赖此函数）                     |
| `mj_sensorVel`        | 底层函数：计算关节速度传感器数据                                         |
| `jointpos/jointvel`   | XML 配置的关节位置/速度传感器（运行时从 `mjData` 读取）                  |
| `mj_readSensor`       | 读取所有传感器数据（包含关节位置/速度传感器）                           |
| `mj_printData/mj_printFormattedData` | 打印 `mjData` 完整信息，包含关节 qpos/qvel 等（调试用）              |
| `mju_getDouble`       | 从动态模型元素中读取关节数值属性（如当前位置）                           |

### 核心直接读取方式（最常用）
无需调用封装函数，直接访问 `mjData` 数组，效率最高：
```c
// mjModel* m; mjData* d; （需提前初始化）
int joint1_id = mj_name2id(m, mjOBJ_JOINT, "joint_link1");

// 1. 读取关节位置（qpos：所有关节位置，维度m->nq，按ID顺序存储）
double joint1_pos = d->qpos[joint1_id]; 
// 2. 读取关节速度（qvel：所有关节速度，维度m->nv，按ID顺序存储）
double joint1_vel = d->qvel[joint1_id];
// 3. 读取关节驱动力矩（qfrc_actuator：关节执行器力矩）
double joint1_torque = d->qfrc_actuator[joint1_id];

printf("关节1位置：%.2f rad，速度：%.2f rad/s，力矩：%.2f N·m\n", 
       joint1_pos, joint1_vel, joint1_torque);
```

## 三、设置 Joint 状态（修改动态参数）
这类操作用于手动调整关节的位置、速度，或设置控制指令，核心是修改 `mjData` 或动态调整模型。

| 函数名                | 功能说明                                                                 |
|-----------------------|--------------------------------------------------------------------------|
| `mj_setState`         | 直接设置整个模型状态（包含所有关节的位置/速度）                           |
| `mj_extractState/mj_getState` | 提取当前状态，修改后通过 `mj_setState` 重新设置                       |
| `mj_copyState`        | 复制一个状态到另一个，间接修改关节状态                                   |
| `mj_resetData`        | 重置 `mjData`，恢复关节到初始状态                                        |
| `mj_resetDataKeyframe` | 重置关节状态到 XML 中定义的指定关键帧                                   |
| `mj_setKeyframe`      | 设置关键帧，间接修改关节目标状态                                         |
| `mjv_applyPerturbPose` | 交互式修改关节位姿（可视化界面操作时用）                               |
| `mj_readCtrl`         | 读取当前控制指令，修改后写入 `d->ctrl` 数组                              |
| `mjs_setFloat/mjs_setDouble` | 动态修改关节数值属性（如目标位置、刚度）                               |
| `mj_setConst`         | 设置模型常量，间接修改关节约束参数                                       |

### 核心直接设置方式（最常用）
```c
// mjModel* m; mjData* d;
int joint1_id = mj_name2id(m, mjOBJ_JOINT, "joint_link1");
int actuator1_id = mj_name2id(m, mjOBJ_ACTUATOR, "pos_link1");

// 1. 直接设置关节位置（需调用mj_forward更新动力学）
d->qpos[joint1_id] = 0.5; // 设置关节1位置为0.5弧度
mj_forward(m, d); // 刷新模型动力学状态

// 2. 设置控制指令（驱动关节运动）
d->ctrl[actuator1_id] = 10.0; // 给关节1的执行器设置10N·m力矩
mj_step(m, d); // 执行仿真步，关节按控制指令运动

// 3. 重置关节到初始状态
mj_resetData(m, d);

// 4. 批量设置关节状态（位置+速度）
double state[m->nq + m->nv];
mj_getState(m, d, state); // 提取当前状态到数组
state[joint1_id] = 1.0; // 修改关节1位置
state[m->nq + joint1_id] = 0.5; // 修改关节1速度
mj_setState(m, d, state); // 应用新状态
```

---

## 关键补充说明
### 1. 核心逻辑
- 静态信息（有哪些关节、参数是什么）→ 存储在 `mjModel` 中，用 `mj_name2id/mj_id2name` 查询；
- 动态状态（关节当前位置/速度）→ 存储在 `mjData` 中，直接访问 `d->qpos/d->qvel` 最高效；
- 设置状态 → 直接修改 `mjData` 数组，或用 `mj_setState` 批量设置。

### 2. 优先级建议
- 查看关节：优先用 `mj_name2id/mj_id2name`（最直接）；
- 读取状态：优先直接访问 `d->qpos/d->qvel`（比传感器更高效）；
- 设置状态：优先直接修改 `d->qpos/d->ctrl`（封装函数适合复杂场景）。

## 总结
| 类别           | 核心函数/方式                  | 场景说明                                   |
|----------------|--------------------------------|--------------------------------------------|
| 查看 Joint     | `mj_name2id/mj_id2name`、`mjs_findJoint` | 验证关节是否存在、遍历所有关节             |
| 读取 Joint 状态 | 直接访问 `d->qpos/d->qvel`、`mj_readSensor` | 实时获取位置/速度，传感器适合标准化读取     |
| 设置 Joint 状态 | 直接修改 `d->qpos/d->ctrl`、`mj_setState` | 手动调整位置、设置控制力矩/速度             |

所有操作的核心：`mjModel` 存储关节的「静态信息」，`mjData` 存储关节的「动态状态」，大部分场景下直接操作这两个结构体的数组是最高效的方式。