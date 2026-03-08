# MuJoCo XML 编写指南
本文档介绍 MuJoCo 仿真场景 XML 文件的核心编写规则，以双层连杆立方体场景为例，覆盖基础结构、核心节点配置及关键注意事项。

---

## 一、XML 整体结构
MuJoCo 场景文件以 `<mujoco>` 为根节点：
```xml
<mujoco>
  <compiler/>   <!-- 编译配置：角度单位、资源路径等 -->
  <option/>     <!-- 物理参数：步长、重力、积分器等 -->
  <asset/>      <!-- 视觉资源：纹理、材质定义 -->
  <worldbody/>  <!-- 物理世界：物体、关节、灯光等 -->
  <actuator/>   <!-- 控制器：关节驱动执行器 -->
</mujoco>
```

## 二、基础配置节点
### 2.1 compiler 编译设置
定义全局编译规则，核心参数控制单位和资源路径：
```xml
<compiler angle="radian" meshdir="meshes" autolimits="true" />
```
- `angle="radian"`：角度单位设为弧度（MuJoCo 推荐）
- `meshdir="meshes"`：模型/纹理文件的根目录
- `autolimits="true"`：自动计算仿真范围

### 2.2 option 物理参数
配置仿真核心物理规则：
```xml
<option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast" />
```
- `timestep="0.002"`：仿真步长（单位：秒，越小精度越高）
- `gravity="0 0 -9.81"`：重力向量（Z轴向下，大小9.81m/s²）
- `integrator="implicitfast"`：快速隐式积分器（适合实时仿真）

## 三、视觉资源：asset 节点
定义纹理、材质，实现物体视觉区分，核心是 `<texture>` 和 `<material>` 组合：
```xml
<asset>
  <!-- 2D纹理：关联图片文件 -->
  <texture name="floor_tex" type="2d" file="Concrete.png" width="512" height="512"/>
  <!-- 材质：绑定纹理，设置视觉属性 -->
  <material name="floor_mat" texture="floor_tex" texrepeat="20 20" rgba="1 1 1 1"/>
  <!-- 纯色材质：无需纹理，直接定义颜色 -->
  <material name="cube_mat" rgba="0.2 0.6 0.8 1" specular="0.2" shininess="0.3"/>
</material>
```
- `texrepeat="20 20"`：纹理重复次数（放大纹理覆盖范围）
- `rgba`：颜色（RGB）+ 透明度（A），取值范围 0~1
- `specular/shininess`：高光和反光度（控制材质光泽）

## 四、物理世界：worldbody 节点
核心节点，定义所有物理物体、关节和层级关系，遵循「body 嵌套=父子关系」规则。

### 4.1 基础几何体（平面/立方体）
通过 `<body>` 定义物体位置，`<geom>` 定义物体形状/物理属性：
```xml
<worldbody>
  <!-- 地面：无限平面 -->
  <geom name="floor" pos="0 0 0" size="50 50 0.1" type="plane" material="floor_mat" condim="3" />
  
  <!-- 立方体：body定义中心位置，geom定义形状尺寸 -->
  <body name="vertical_cube" pos="0 0 0.25"> 
    <geom name="cube_geom" type="box" size="0.125 0.05 0.25" material="cube_mat" mass="1.0" condim="3"/>
  </body>
</worldbody>
```
- `type="plane/box"`：几何体类型（平面/立方体）
- `pos`：物体中心坐标（单位：米）
- `size`：尺寸（box 为半长/半宽/半高；plane 为平面范围+厚度）
- `mass`：质量（单位：千克）
- `condim="3"`：接触维度（3表示全维度接触）

### 4.2 关节与连杆
通过 `<joint>` 定义物体间的运动约束，需嵌套在 `<body>` 内且位于 `<geom>` 之前：
```xml
<body name="link1" pos="-0.15 -0.1 -0.07">
  <!-- 铰链关节：绕Y轴旋转，带角度限制 -->
  <joint type="hinge" name="joint_link1" pos="0 0 0.05" axis="0 1 0" 
         damping="0.1" frictionloss="0.01" range="-1.57 1.57" limited="true" />
  <!-- 连杆几何体：胶囊体 -->
  <geom name="link1_geom" type="capsule" size="0.015 0.035" material="link_mat" mass="0.01"/>
  
  <!-- 二级连杆：嵌套body实现父子连杆 -->
  <body name="link1_2" pos="0 0 -0.1">
    <joint type="hinge" name="joint_link1_2" pos="0 0 0.05" axis="0 1 0" 
           range="-3.1416 0" limited="true" />
    <geom name="link1_2_geom" type="capsule" size="0.01 0.04" material="link2_mat" mass="0.01"/>
  </body>
</body>
```
- `joint type="hinge"`：铰链关节（单轴旋转）
- `axis="0 1 0"`：旋转轴（X/Y/Z轴分别对应 1 0 0 / 0 1 0 / 0 0 1）
- `range="-1.57 1.57"`：关节旋转范围（单位：弧度，-1.57≈-90°，1.57≈90°）
- `limited="true"`：启用角度限制（必须配合 range 使用）
- `damping/frictionloss`：阻尼/摩擦损耗（模拟物理阻力，取值0~1）
- `capsule` 尺寸：`size="半径 半长"`（胶囊体的核心尺寸定义）

## 五、控制器：actuator 节点
为关节添加驱动执行器，实现关节角度/力控制，常用 `<position>` 位置控制器：
```xml
<actuator>
  <position name="pos_link1" joint="joint_link1" kp="100" dampratio="1" inheritrange="1.0"/>
  <position name="pos_link1_2" joint="joint_link1_2" kp="100" dampratio="1" inheritrange="1.0"/>
</actuator>
```
- `joint="joint_link1"`：绑定需控制的关节名
- `kp="100"`：比例增益（越大关节响应越快，过大会震荡）
- `dampratio="1"`：阻尼比（1为临界阻尼，无超调）
- `inherirange="1.0"`：继承关节的角度限制（无需重复定义范围）


参考文档：
- [MuJoCo XML 参考手册 - body/joint](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint)
- [MuJoCo XML 参考手册 - actuator](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator)

---
