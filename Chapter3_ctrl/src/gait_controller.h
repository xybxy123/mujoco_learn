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
    std::array<int, kLegCount> hip_actuator_ids{};
    std::array<int, kLegCount> knee_actuator_ids{};
    std::array<JointAngles, kLegCount> filtered_commands{};
};

class GaitController {
public:
    GaitController();

    void Initialize(const mjModel* model);
    void SetInitialPose(const mjModel* model, mjData* data);
    void Update(const mjModel* model, mjData* data);
    
    // reset pose internally without resetting data
    void Reset();

private:
    ControllerState state_;
    LegIK leg_ik_;
};

}  // namespace quad