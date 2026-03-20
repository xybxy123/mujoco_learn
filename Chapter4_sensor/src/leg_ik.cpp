#include "leg_ik.h"

#include <algorithm>
#include <cmath>

namespace {
mjtNum clampValue(mjtNum value, mjtNum lower, mjtNum upper) {
    return std::max(lower, std::min(value, upper));
}
}  // namespace

namespace quad {

LegIK::LegIK(mjtNum upper_leg_length, mjtNum lower_leg_length)
    : upper_leg_length_(upper_leg_length), lower_leg_length_(lower_leg_length) {}

bool LegIK::Solve(mjtNum foot_x, mjtNum foot_z, JointAngles& angles) const {
    const mjtNum distance_sq = foot_x * foot_x + foot_z * foot_z;
    const mjtNum reach_min = std::abs(upper_leg_length_ - lower_leg_length_) + 1e-6;
    const mjtNum reach_max = upper_leg_length_ + lower_leg_length_ - 1e-6;
    const mjtNum distance = clampValue(std::sqrt(distance_sq), reach_min, reach_max);
    const mjtNum clamped_distance_sq = distance * distance;

    mjtNum cos_knee = (clamped_distance_sq - upper_leg_length_ * upper_leg_length_ -
                       lower_leg_length_ * lower_leg_length_) /
                      (2.0 * upper_leg_length_ * lower_leg_length_);
    cos_knee = clampValue(cos_knee, -1.0, 1.0);

    angles.knee = -std::acos(cos_knee);
    angles.hip = std::atan2(foot_x, foot_z) -
                 std::atan2(lower_leg_length_ * std::sin(angles.knee),
                            upper_leg_length_ + lower_leg_length_ * std::cos(angles.knee));
    return true;
}

}  // namespace quad