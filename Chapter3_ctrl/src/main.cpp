#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

namespace {

constexpr int kLegCount = 4;
constexpr mjtNum kUpperLegLength = 0.105;
constexpr mjtNum kLowerLegLength = 0.1163;
constexpr mjtNum kNominalHeight = 0.205;
constexpr mjtNum kStepHeight = 0.028;
constexpr mjtNum kStanceSink = 0.004;
constexpr mjtNum kStepLength = 0.075;
constexpr mjtNum kCyclePeriod = 0.60;
constexpr mjtNum kDutyFactor = 0.58;
constexpr mjtNum kJointCommandFilter = 0.25;
constexpr mjtNum kStandingBodyHeight = 0.255;
constexpr mjtNum kFrontBias = 0.03;
constexpr mjtNum kRearBias = -0.03;

struct JointAngles {
    mjtNum hip = 0.0;
    mjtNum knee = 0.0;
};

struct FootTarget {
    mjtNum x = 0.0;
    mjtNum z = kNominalHeight;
};

struct LegConfig {
    const char* hip_actuator;
    const char* knee_actuator;
    const char* hip_joint;
    const char* knee_joint;
    mjtNum phase_offset;
    mjtNum x_bias;
};

struct ControllerState {
    bool initialized = false;
    bool valid = false;
    std::array<int, kLegCount> hip_actuator_ids{};
    std::array<int, kLegCount> knee_actuator_ids{};
    std::array<JointAngles, kLegCount> filtered_commands{};
};

constexpr std::array<LegConfig, kLegCount> kLegs = {{
    {"h_front_left_pos", "l_front_left_pos", "h_front_left", "l_front_left", 0.0, kFrontBias},
    {"h_front_right_pos", "l_front_right_pos", "h_front_right", "l_front_right", 0.5, kFrontBias},
    {"h_back_left_pos", "l_back_left_pos", "h_back_left", "l_back_left", 0.5, kRearBias},
    {"h_back_right_pos", "l_back_right_pos", "h_back_right", "l_back_right", 0.0, kRearBias},
}};

ControllerState g_controller;

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

bool solveLegIK(mjtNum foot_x, mjtNum foot_z, JointAngles& angles) {
    const mjtNum distance_sq = foot_x * foot_x + foot_z * foot_z;
    const mjtNum reach_min = std::abs(kUpperLegLength - kLowerLegLength) + 1e-6;
    const mjtNum reach_max = kUpperLegLength + kLowerLegLength - 1e-6;
    const mjtNum distance = clampValue(std::sqrt(distance_sq), reach_min, reach_max);
    const mjtNum clamped_distance_sq = distance * distance;

    mjtNum cos_knee = (clamped_distance_sq - kUpperLegLength * kUpperLegLength -
                       kLowerLegLength * kLowerLegLength) /
                      (2.0 * kUpperLegLength * kLowerLegLength);
    cos_knee = clampValue(cos_knee, -1.0, 1.0);

    angles.knee = -std::acos(cos_knee);
    angles.hip = std::atan2(foot_x, foot_z) -
                 std::atan2(kLowerLegLength * std::sin(angles.knee),
                            kUpperLegLength + kLowerLegLength * std::cos(angles.knee));
    return true;
}

FootTarget computeTrotTarget(const LegConfig& leg, mjtNum time) {
    const mjtNum cycle_phase = std::fmod(time / kCyclePeriod + leg.phase_offset, 1.0);
    const mjtNum phase = cycle_phase < 0.0 ? cycle_phase + 1.0 : cycle_phase;
    const mjtNum half_step = 0.5 * kStepLength;

    FootTarget target;
    if (phase < kDutyFactor) {
        const mjtNum stance_phase = phase / kDutyFactor;
        target.x = leg.x_bias + half_step - kStepLength * stance_phase;
        target.z = kNominalHeight + kStanceSink * std::sin(mjPI * stance_phase);
    } else {
        const mjtNum swing_phase = (phase - kDutyFactor) / (1.0 - kDutyFactor);
        target.x = leg.x_bias - half_step + kStepLength * swing_phase;
        target.z = kNominalHeight - kStepHeight * std::sin(mjPI * swing_phase);
    }

    return target;
}

void initializeController(const mjModel* model) {
    g_controller = ControllerState{};
    g_controller.initialized = true;
    g_controller.valid = true;

    for (int leg_index = 0; leg_index < kLegCount; ++leg_index) {
        g_controller.hip_actuator_ids[leg_index] =
            mj_name2id(model, mjOBJ_ACTUATOR, kLegs[leg_index].hip_actuator);
        g_controller.knee_actuator_ids[leg_index] =
            mj_name2id(model, mjOBJ_ACTUATOR, kLegs[leg_index].knee_actuator);

        if (g_controller.hip_actuator_ids[leg_index] < 0 ||
            g_controller.knee_actuator_ids[leg_index] < 0) {
            std::cerr << "Actuator lookup failed for leg " << leg_index << std::endl;
            g_controller.valid = false;
        }
    }
}

void setInitialPose(const mjModel* model, mjData* data) {
    if (model->nq >= 7) {
        data->qpos[0] = 0.0;
        data->qpos[1] = 0.0;
        data->qpos[2] = kStandingBodyHeight;
        data->qpos[3] = 1.0;
        data->qpos[4] = 0.0;
        data->qpos[5] = 0.0;
        data->qpos[6] = 0.0;
    }

    for (int leg_index = 0; leg_index < kLegCount; ++leg_index) {
        JointAngles angles;
        solveLegIK(kLegs[leg_index].x_bias, kNominalHeight, angles);

        const int hip_joint_id = mj_name2id(model, mjOBJ_JOINT, kLegs[leg_index].hip_joint);
        const int knee_joint_id = mj_name2id(model, mjOBJ_JOINT, kLegs[leg_index].knee_joint);

        if (hip_joint_id >= 0) {
            data->qpos[model->jnt_qposadr[hip_joint_id]] = angles.hip;
        }
        if (knee_joint_id >= 0) {
            data->qpos[model->jnt_qposadr[knee_joint_id]] = angles.knee;
        }

        g_controller.filtered_commands[leg_index] = angles;

        const int hip_actuator_id = g_controller.hip_actuator_ids[leg_index];
        const int knee_actuator_id = g_controller.knee_actuator_ids[leg_index];
        if (hip_actuator_id >= 0) {
            data->ctrl[hip_actuator_id] = angles.hip;
        }
        if (knee_actuator_id >= 0) {
            data->ctrl[knee_actuator_id] = angles.knee;
        }
    }

    mj_forward(model, data);
}

void mycontroller(const mjModel* model, mjData* data) {
    if (!g_controller.initialized) {
        initializeController(model);
    }
    if (!g_controller.valid) {
        return;
    }

    for (int leg_index = 0; leg_index < kLegCount; ++leg_index) {
        const FootTarget target = computeTrotTarget(kLegs[leg_index], data->time);
        JointAngles target_angles;
        if (!solveLegIK(target.x, target.z, target_angles)) {
            continue;
        }

        JointAngles& filtered = g_controller.filtered_commands[leg_index];
        filtered.hip += kJointCommandFilter * (target_angles.hip - filtered.hip);
        filtered.knee += kJointCommandFilter * (target_angles.knee - filtered.knee);

        const int hip_actuator_id = g_controller.hip_actuator_ids[leg_index];
        const int knee_actuator_id = g_controller.knee_actuator_ids[leg_index];
        if (hip_actuator_id >= 0) {
            data->ctrl[hip_actuator_id] =
                actuatorLimitedCommand(model, hip_actuator_id, filtered.hip);
        }
        if (knee_actuator_id >= 0) {
            data->ctrl[knee_actuator_id] =
                actuatorLimitedCommand(model, knee_actuator_id, filtered.knee);
        }
    }
}

}  // namespace

mjModel* m = nullptr;
mjData* d = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    (void)window;
    (void)scancode;
    (void)mods;

    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
        mj_resetData(m, d);
        initializeController(m);
        setInitialPose(m, d);
    }
}

void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    (void)window;
    (void)button;
    (void)act;
    (void)mods;
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    if (!button_left && !button_middle && !button_right) {
        return;
    }

    const double dx = xpos - lastx;
    const double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width = 0;
    int height = 0;
    glfwGetWindowSize(window, &width, &height);

    const bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                            glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (button_right) {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (button_left) {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    (void)window;
    (void)xoffset;
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

int main() {
    char error[1000] = "Could not load XML model";
    m = mj_loadXML("../assets/xml_test.xml", nullptr, error, sizeof(error));
    if (!m) {
        std::cerr << "Load model error: " << error << std::endl;
        return 1;
    }

    d = mj_makeData(m);
    mjcb_control = mycontroller;
    initializeController(m);
    setInitialPose(m, d);

    if (!glfwInit()) {
        mju_error("Could not initialize GLFW");
    }

    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo Trot Demo", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    cam.azimuth = 90.0;
    cam.elevation = -30.0;
    cam.distance = 1.8;
    cam.lookat[0] = 0.0;
    cam.lookat[1] = 0.0;
    cam.lookat[2] = 0.25;

    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    std::cout << "Running fixed trot gait. Press BACKSPACE to reset the pose." << std::endl;

    while (!glfwWindowShouldClose(window)) {
        const mjtNum simstart = d->time;
        while (d->time - simstart < 1.0 / 60.0) {
            mj_step(m, d);
        }

        int viewport_width = 0;
        int viewport_height = 0;
        glfwGetFramebufferSize(window, &viewport_width, &viewport_height);
        mjrRect rect = {0, 0, viewport_width, viewport_height};

        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(rect, &scn, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
    return 0;
}
