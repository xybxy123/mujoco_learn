#pragma once

#include <mujoco/mujoco.h>

namespace quad {

struct JointAngles {
    mjtNum hip = 0.0;
    mjtNum knee = 0.0;
};

class LegIK {
public:
    LegIK(mjtNum upper_leg_length, mjtNum lower_leg_length);

    // Returns true if the target is reachable and solution is valid
    bool Solve(mjtNum foot_x, mjtNum foot_z, JointAngles& angles) const;

private:
    mjtNum upper_leg_length_;
    mjtNum lower_leg_length_;
};

}  // namespace quad