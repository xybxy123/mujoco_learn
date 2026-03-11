#include "gait_controller.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace quad {

namespace {

constexpr mjtNum kUpperLegLength    = 0.105;
constexpr mjtNum kLowerLegLength    = 0.1163;
constexpr mjtNum kNominalHeight     = 0.205;   // 足端到髋关节默认高度
constexpr mjtNum kStepHeight        = 0.028;   // 摆动相最大抬脚高度
constexpr mjtNum kStanceSink        = 0.004;   // 支撑相轻微下沉量
constexpr mjtNum kStepLength        = 0.075;   // 前后步幅
constexpr mjtNum kCyclePeriod       = 0.60;    // 步态周期（秒）
constexpr mjtNum kDutyFactor        = 0.58;    // 支撑相占比
constexpr mjtNum kStandingBodyHeight= 0.5;     // 机身初始离地高度
constexpr mjtNum kFrontBias         = 0.03;    // 前腿 x 默认偏置
constexpr mjtNum kRearBias          = -0.03;   // 后腿 x 默认偏置

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

// 静止站立目标：足端保持在默认偏置位置
FootTarget computeStandingTarget(const LegConfig& leg) {
    return {leg.x_bias, kNominalHeight};
}

// Trot 步态足端目标（前进方向，x 偏置由外部乘以方向系数翻转）
FootTarget computeTrotTarget(const LegConfig& leg, mjtNum time) {
    mjtNum phase = std::fmod(time / kCyclePeriod + leg.phase_offset, 1.0);
    if (phase < 0.0) phase += 1.0;
    const mjtNum half = 0.5 * kStepLength;

    FootTarget target;
    if (phase < kDutyFactor) {
        // 支撑相：足端向后划过，推动机身前进
        const mjtNum sp = phase / kDutyFactor;
        target.x = leg.x_bias + (half - kStepLength * sp);
        target.z = kNominalHeight + kStanceSink * std::sin(mjPI * sp);
    } else {
        // 摆动相：抬腿向前迈步
        const mjtNum sw = (phase - kDutyFactor) / (1.0 - kDutyFactor);
        target.x = leg.x_bias + (-half + kStepLength * sw);
        target.z = kNominalHeight - kStepHeight * std::sin(mjPI * sw);
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
    state_.motion_direction = 0;
    state_.gait_time        = 0.0;
    state_.last_sim_time    = data->time;

    // 机身初始位置
    if (model->nq >= 7) {
        data->qpos[0] = 0.0; data->qpos[1] = 0.0; data->qpos[2] = kStandingBodyHeight;
        data->qpos[3] = 1.0; data->qpos[4] = 0.0; data->qpos[5] = 0.0; data->qpos[6] = 0.0;
    }

    // 四腿直接写入站立关节角，避免启动时抖动
    for (int i = 0; i < kLegCount; ++i) {
        const FootTarget t = computeStandingTarget(kLegs[i]);
        JointAngles a;
        if (!leg_ik_.Solve(t.x, t.z, a)) continue;

        const int hj = mj_name2id(model, mjOBJ_JOINT, kLegs[i].hip_joint);
        const int kj = mj_name2id(model, mjOBJ_JOINT, kLegs[i].knee_joint);
        if (hj >= 0) data->qpos[model->jnt_qposadr[hj]] = a.hip;
        if (kj >= 0) data->qpos[model->jnt_qposadr[kj]] = a.knee;

        const int ha = state_.hip_actuator_ids[i];
        const int ka = state_.knee_actuator_ids[i];
        if (ha >= 0) data->ctrl[ha] = a.hip;
        if (ka >= 0) data->ctrl[ka] = a.knee;
    }

    mj_forward(model, data);
}

void GaitController::SetMotionDirection(int direction) {
    const int clamped = (direction > 0) ? 1 : (direction < 0) ? -1 : 0;
    if (clamped != 0 && state_.motion_direction == 0) {
        // 从静止启动：置于支撑中段相位，承重腿足端恰好在 x_bias，零跳变
        state_.gait_time = 0.5 * kDutyFactor * kCyclePeriod;
    } else if (clamped == 0 && state_.motion_direction != 0) {
        // 停止时重置步态时钟
        state_.gait_time = 0.0;
    }
    state_.motion_direction = clamped;
}

void GaitController::Update(const mjModel* model, mjData* data) {
    if (!state_.initialized) {
        Initialize(model);
        state_.last_sim_time = data->time;
    }
    if (!state_.valid) return;

    // 仅在运动时推进步态时钟
    const mjtNum dt = std::max<mjtNum>(0.0, data->time - state_.last_sim_time);
    state_.last_sim_time = data->time;
    if (state_.motion_direction != 0) state_.gait_time += dt;

    for (int i = 0; i < kLegCount; ++i) {
        FootTarget target;
        if (state_.motion_direction == 0) {
            // 静止：保持默认站立足端位置
            target = computeStandingTarget(kLegs[i]);
        } else {
            // 运动：步态轨迹 + 方向翻转
            target = computeTrotTarget(kLegs[i], state_.gait_time);
            const mjtNum offset = target.x - kLegs[i].x_bias;
            target.x = kLegs[i].x_bias + state_.motion_direction * offset;
        }

        JointAngles angles;
        if (!leg_ik_.Solve(target.x, target.z, angles)) continue;

        const int ha = state_.hip_actuator_ids[i];
        const int ka = state_.knee_actuator_ids[i];
        if (ha >= 0) data->ctrl[ha] = actuatorLimitedCommand(model, ha, angles.hip);
        if (ka >= 0) data->ctrl[ka] = actuatorLimitedCommand(model, ka, angles.knee);
    }
}

}  // namespace quad