#include "gait_controller.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace quad {

namespace {

constexpr mjtNum kUpperLegLength = 0.105;
constexpr mjtNum kLowerLegLength = 0.1163;
constexpr mjtNum kNominalHeight = 0.205;
constexpr mjtNum kStepHeight = 0.028;
constexpr mjtNum kStanceSink = 0.004;
constexpr mjtNum kStepLength = 0.075;
constexpr mjtNum kForwardStepSign = -1.0;
constexpr mjtNum kCyclePeriod = 0.60;
constexpr mjtNum kDutyFactor = 0.58;
constexpr mjtNum kStandingBodyHeight = 0.5;
constexpr mjtNum kFrontBias = 0.03;
constexpr mjtNum kRearBias = -0.03;

struct LegConfig {
    const char* hip_actuator;
    const char* knee_actuator;
    const char* hip_joint;
    const char* knee_joint;
    mjtNum phase_offset;
    mjtNum x_bias;
};

constexpr std::array<LegConfig, kLegCount> kLegs = {{
    {"h_front_left_pos", "l_front_left_pos", "h_front_left", "l_front_left", 0.0, kFrontBias},
    {"h_front_right_pos", "l_front_right_pos", "h_front_right", "l_front_right", 0.5, kFrontBias},
    {"h_back_left_pos", "l_back_left_pos", "h_back_left", "l_back_left", 0.5, kRearBias},
    {"h_back_right_pos", "l_back_right_pos", "h_back_right", "l_back_right", 0.0, kRearBias},
}};

mjtNum clampValue(mjtNum value, mjtNum lower, mjtNum upper) {
    return std::max(lower, std::min(value, upper));
}

mjtNum actuatorLimitedCommand(const mjModel* model, int actuator_id, mjtNum command) {
    if (actuator_id < 0) {
        return command;
    }

    if (model->actuator_ctrllimited[actuator_id]) {
        const mjtNum low = model->actuator_ctrlrange[2 * actuator_id];
        const mjtNum high = model->actuator_ctrlrange[2 * actuator_id + 1];
        return clampValue(command, low, high);
    }

    const int joint_id = model->actuator_trnid[2 * actuator_id];
    if (joint_id >= 0 && model->jnt_limited[joint_id]) {
        const mjtNum low = model->jnt_range[2 * joint_id];
        const mjtNum high = model->jnt_range[2 * joint_id + 1];
        return clampValue(command, low, high);
    }

    return command;
}

FootTarget computeTrotTarget(const LegConfig& leg, mjtNum time) {
    const mjtNum cycle_phase = std::fmod(time / kCyclePeriod + leg.phase_offset, 1.0);
    const mjtNum phase = cycle_phase < 0.0 ? cycle_phase + 1.0 : cycle_phase;
    const mjtNum half_step = 0.5 * kStepLength;
    const mjtNum step_sign = kForwardStepSign;

    FootTarget target;
    if (phase < kDutyFactor) {
        const mjtNum stance_phase = phase / kDutyFactor;
        target.x = leg.x_bias + step_sign * (half_step - kStepLength * stance_phase);
        target.z = kNominalHeight + kStanceSink * std::sin(mjPI * stance_phase);
    } else {
        const mjtNum swing_phase = (phase - kDutyFactor) / (1.0 - kDutyFactor);
        target.x = leg.x_bias + step_sign * (-half_step + kStepLength * swing_phase);
        target.z = kNominalHeight - kStepHeight * std::sin(mjPI * swing_phase);
    }

    return target;
}

}  // namespace

GaitController::GaitController() : state_{}, leg_ik_(kUpperLegLength, kLowerLegLength) {}


void GaitController::Initialize(const mjModel* model) {
    state_ = ControllerState{};
    state_.initialized = true;
    state_.valid = true;

    for (int leg_index = 0; leg_index < kLegCount; ++leg_index) {
        
        state_.hip_actuator_ids[leg_index] =mj_name2id(model, mjOBJ_ACTUATOR, kLegs[leg_index].hip_actuator);

        state_.knee_actuator_ids[leg_index] =mj_name2id(model, mjOBJ_ACTUATOR, kLegs[leg_index].knee_actuator);

        if (state_.hip_actuator_ids[leg_index] < 0 ||
            state_.knee_actuator_ids[leg_index] < 0) {
            std::cerr << "Actuator lookup failed for leg " << leg_index << std::endl;
            state_.valid = false;
        }
    }
}

void GaitController::SetInitialPose(const mjModel* model, mjData* data) {
    //设置初始姿态，站立在地面上
    data->qpos[0] = 0.0;
    data->qpos[1] = 0.0;
    data->qpos[2] = kStandingBodyHeight;
    data->qpos[3] = 1.0;
    data->qpos[4] = 0.0;
    data->qpos[5] = 0.0;
    data->qpos[6] = 0.0;

    mj_forward(model, data);
}

void GaitController::Update(const mjModel* model, mjData* data) {
    if (!state_.initialized) {
        Initialize(model);
    }
    if (!state_.valid) {
        return;
    }

    for (int leg_index = 0; leg_index < kLegCount; ++leg_index) {
        const FootTarget target = computeTrotTarget(kLegs[leg_index], data->time);
        JointAngles target_angles;
        if (!leg_ik_.Solve(target.x, target.z, target_angles)) {
            continue;
        }

        const int hip_actuator_id = state_.hip_actuator_ids[leg_index];
        const int knee_actuator_id = state_.knee_actuator_ids[leg_index];

        data->ctrl[hip_actuator_id] =actuatorLimitedCommand(model, hip_actuator_id, target_angles.hip);
        data->ctrl[knee_actuator_id] =actuatorLimitedCommand(model, knee_actuator_id, target_angles.knee);
    }
}

}  // namespace quad