#include <array>
#include <algorithm>
#include <cmath>
#include <iostream>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

namespace {

constexpr int kLegCount = 4;
constexpr mjtNum kUpperLegLength = 0.105;
constexpr mjtNum kLowerLegLength = 0.1163;
constexpr mjtNum kNominalFootHeight = 0.185;
constexpr mjtNum kStepLength = 0.05;
constexpr mjtNum kStepHeight = 0.025;
constexpr mjtNum kCyclePeriod = 0.6;
constexpr mjtNum kStartupBlendTime = 1.0;
constexpr mjtNum kCommandFilter = 0.15;
constexpr mjtNum kPi = 3.14159265358979323846;

struct LegConfig {
    const char* hip_actuator;
    const char* knee_actuator;
    mjtNum phase_offset;
    mjtNum x_bias;
};

struct ControllerState {
    bool initialized = false;
    bool valid = false;
    std::array<int, kLegCount> hip_actuator_ids{};
    std::array<int, kLegCount> knee_actuator_ids{};
    std::array<mjtNum, kLegCount> hip_commands{};
    std::array<mjtNum, kLegCount> knee_commands{};
};

constexpr std::array<LegConfig, kLegCount> kLegConfigs = {{
    {"h_front_left_pos", "l_front_left_pos", 0.0, 0.02},
    {"h_front_right_pos", "l_front_right_pos", 0.5, 0.02},
    {"h_back_left_pos", "l_back_left_pos", 0.5, -0.02},
    {"h_back_right_pos", "l_back_right_pos", 0.0, -0.02},
}};

ControllerState g_controller;

mjtNum clampValue(mjtNum value, mjtNum lower, mjtNum upper) {
    return std::max(lower, std::min(value, upper));
}

mjtNum normalizePhase(mjtNum phase) {
    phase = std::fmod(phase, 1.0);
    if (phase < 0.0) {
        phase += 1.0;
    }
    return phase;
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

bool solveLegIK(mjtNum foot_x, mjtNum foot_z, mjtNum& hip_angle, mjtNum& knee_angle) {
    const mjtNum max_reach = kUpperLegLength + kLowerLegLength - 1e-6;
    const mjtNum min_reach = std::fabs(kUpperLegLength - kLowerLegLength) + 1e-6;

    mjtNum radius = std::sqrt(foot_x * foot_x + foot_z * foot_z);
    if (radius < 1e-8) {
        radius = min_reach;
    }

    const mjtNum clamped_radius = clampValue(radius, min_reach, max_reach);
    const mjtNum scale = clamped_radius / radius;
    foot_x *= scale;
    foot_z *= scale;

    const mjtNum cos_knee = clampValue(
        (foot_x * foot_x + foot_z * foot_z - kUpperLegLength * kUpperLegLength -
         kLowerLegLength * kLowerLegLength) /
            (2.0 * kUpperLegLength * kLowerLegLength),
        -1.0, 1.0);

    knee_angle = -std::acos(cos_knee);

    const mjtNum link_projection = kUpperLegLength + kLowerLegLength * std::cos(knee_angle);
    const mjtNum link_offset = kLowerLegLength * std::sin(knee_angle);
    hip_angle = std::atan2(foot_x, foot_z) - std::atan2(link_offset, link_projection);

    return std::isfinite(hip_angle) && std::isfinite(knee_angle);
}

void resetControllerState() {
    g_controller = ControllerState{};
}

void initializeController(const mjModel* model) {
    g_controller.initialized = true;
    g_controller.valid = true;

    for (int leg_index = 0; leg_index < kLegCount; ++leg_index) {
        const auto& leg = kLegConfigs[leg_index];
        g_controller.hip_actuator_ids[leg_index] = mj_name2id(model, mjOBJ_ACTUATOR, leg.hip_actuator);
        g_controller.knee_actuator_ids[leg_index] = mj_name2id(model, mjOBJ_ACTUATOR, leg.knee_actuator);

        if (g_controller.hip_actuator_ids[leg_index] < 0 ||
            g_controller.knee_actuator_ids[leg_index] < 0) {
            std::cerr << "Actuator lookup failed for leg " << leg_index << std::endl;
            g_controller.valid = false;
        }
    }
}

void mycontroller(const mjModel* model, mjData* data) {
    if (!g_controller.initialized) {
        initializeController(model);
    }

    if (!g_controller.valid) {
        return;
    }

    const mjtNum blend = clampValue(data->time / kStartupBlendTime, 0.0, 1.0);

    for (int leg_index = 0; leg_index < kLegCount; ++leg_index) {
        const auto& leg = kLegConfigs[leg_index];
        const mjtNum phase = normalizePhase(data->time / kCyclePeriod + leg.phase_offset);

        mjtNum foot_x = leg.x_bias;
        mjtNum foot_z = kNominalFootHeight;

        if (phase < 0.5) {
            const mjtNum stance_phase = phase / 0.5;
            foot_x += 0.5 * kStepLength - kStepLength * stance_phase;
        } else {
            const mjtNum swing_phase = (phase - 0.5) / 0.5;
            foot_x += -0.5 * kStepLength + kStepLength * swing_phase;
            foot_z -= kStepHeight * std::sin(kPi * swing_phase);
        }

        foot_x = leg.x_bias + blend * (foot_x - leg.x_bias);
        foot_z = kNominalFootHeight + blend * (foot_z - kNominalFootHeight);

        mjtNum hip_target = 0.0;
        mjtNum knee_target = 0.0;
        if (!solveLegIK(foot_x, foot_z, hip_target, knee_target)) {
            continue;
        }

        g_controller.hip_commands[leg_index] +=
            kCommandFilter * (hip_target - g_controller.hip_commands[leg_index]);
        g_controller.knee_commands[leg_index] +=
            kCommandFilter * (knee_target - g_controller.knee_commands[leg_index]);

        const int hip_actuator_id = g_controller.hip_actuator_ids[leg_index];
        const int knee_actuator_id = g_controller.knee_actuator_ids[leg_index];

        data->ctrl[hip_actuator_id] =
            actuatorLimitedCommand(model, hip_actuator_id, g_controller.hip_commands[leg_index]);
        data->ctrl[knee_actuator_id] =
            actuatorLimitedCommand(model, knee_actuator_id, g_controller.knee_commands[leg_index]);
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
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
        mj_resetData(m, d);
        resetControllerState();
        mj_forward(m, d);
    }
}

void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    if (!button_left && !button_middle && !button_right) {
        return;
    }

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
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
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

int main() {
    char error[1000] = "Could not load binary model";
    m = mj_loadXML("../assets/xml_test.xml", 0, error, 1000);
    if (!m) {
        std::cerr << "Load model error: " << error << std::endl;
        return 1;
    }

    d = mj_makeData(m);
    mjcb_control = mycontroller;

    if (!glfwInit()) {
        mju_error("Could not initialize GLFW");
    }

    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo Control Interface", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);

    cam.azimuth = 90;
    cam.elevation = -45;
    cam.distance = 2.5;
    cam.lookat[0] = 0;
    cam.lookat[1] = 0;
    cam.lookat[2] = 0.5;

    mjr_makeContext(m, &con, mjFONTSCALE_150);

    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    while (!glfwWindowShouldClose(window)) {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0 / 60.0) {
            mj_step(m, d);
        }

        int viewport_width, viewport_height;
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
