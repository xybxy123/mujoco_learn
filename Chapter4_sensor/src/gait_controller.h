#pragma once

#include <array>
#include <mujoco/mujoco.h>

#include "leg_ik.h"

namespace quad {

constexpr int kLegCount = 4;

struct FootTarget {
    mjtNum x = 0.0;
    mjtNum z = 0.0;
};

struct ControllerState {
    bool initialized = false;
    bool valid = false;
    int  motion_direction  = 0;    // 0=静止  1=前进  -1=后退
    mjtNum gait_time       = 0.0;  // 步态内部时钟（仅运动时推进）
    mjtNum last_sim_time   = 0.0;  // 上一帧仿真时间
    std::array<int, kLegCount> hip_actuator_ids{};
    std::array<int, kLegCount> knee_actuator_ids{};
};

class GaitController {
public:
    GaitController();

    void Initialize(const mjModel* model);
    void SetInitialPose(const mjModel* model, mjData* data);
    // direction: 1=前进, -1=后退, 0=静止
    void SetMotionDirection(int direction);
    void Update(const mjModel* model, mjData* data);

private:
    ControllerState state_;
    LegIK leg_ik_;
};

}  // namespace quad