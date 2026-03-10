/**
 * test.cpp — 键盘控制四足机器狗前进 / 后退
 *
 * 操作说明：
 *   W          按住前进，松开停止
 *   S          按住后退，松开停止
 *   Backspace  重置姿态
 *   鼠标       拖拽/滚轮调整视角
 *
 * 设计思路：
 *   - 默认静止：四条腿锁定在各自的站立默认位置
 *   - 运动时使用独立的步态时钟 (g_gait_time)，与仿真时间解耦
 *     → 停止后重置步态时钟，下次启动从相位 0 开始
 *   - 方向由 g_direction 控制（1=前进, -1=后退, 0=静止）
 *     W/S 同时按下时互相抵消 → 静止
 *
 * 只依赖 leg_ik.h / leg_ik.cpp，不依赖 gait_controller。
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "leg_ik.h"

// ============================================================
// 步态参数（按需调整）
// ============================================================
namespace {

constexpr mjtNum kUpperLeg      = 0.105;
constexpr mjtNum kLowerLeg      = 0.1163;
constexpr mjtNum kNominalHeight = 0.205;   // 足端到髋关节默认高度
constexpr mjtNum kStepHeight    = 0.028;   // 摆动相最大抬脚高度
constexpr mjtNum kStanceSink    = 0.004;   // 支撑相轻微下沉量
constexpr mjtNum kStepLength    = 0.075;   // 前后步幅
constexpr mjtNum kCyclePeriod   = 0.60;    // 步态周期（秒）
constexpr mjtNum kDutyFactor    = 0.58;    // 支撑相占比
constexpr mjtNum kBodyHeight    = 0.5;     // 机身初始离地高度
constexpr mjtNum kFrontBias     = 0.03;    // 前腿 x 默认偏置
constexpr mjtNum kRearBias      = -0.03;   // 后腿 x 默认偏置

// ============================================================
// 腿部配置（执行器名 / 关节名 / 步态相位 / 前后偏置）
// ============================================================
struct LegCfg {
    const char* hip_act;
    const char* knee_act;
    const char* hip_jnt;
    const char* knee_jnt;
    mjtNum      phase_offset;
    mjtNum      x_bias;
};

constexpr int kNLegs = 4;
constexpr std::array<LegCfg, kNLegs> kLegs = {{
    // 对角腿同相 → Trot：左前+右后  /  右前+左后
    {"h_front_left_pos",  "l_front_left_pos",  "h_front_left",  "l_front_left",  0.0, kFrontBias},
    {"h_front_right_pos", "l_front_right_pos", "h_front_right", "l_front_right", 0.5, kFrontBias},
    {"h_back_left_pos",   "l_back_left_pos",   "h_back_left",   "l_back_left",   0.5, kRearBias},
    {"h_back_right_pos",  "l_back_right_pos",  "h_back_right",  "l_back_right",  0.0, kRearBias},
}};

// ============================================================
// 全局仿真对象
// ============================================================
mjModel*   m = nullptr;
mjData*    d = nullptr;
mjvCamera  cam;
mjvOption  opt;
mjvScene   scn;
mjrContext con;

// 逆运动学求解器（二连杆平面 IK）
quad::LegIK g_ik(kUpperLeg, kLowerLeg);

// 每条腿的执行器 ID（Initialize 时填充）
std::array<int, kNLegs> g_hip_ids{};
std::array<int, kNLegs> g_knee_ids{};

// ============================================================
// 运动控制状态
// ============================================================
int    g_direction     = 0;    // 0=静止  1=前进  -1=后退
mjtNum g_gait_time     = 0.0;  // 步态内部时钟（仅运动时推进）
mjtNum g_last_sim_time = 0.0;  // 上一控制帧仿真时间

bool   g_key_w = false;        // W 是否按下
bool   g_key_s = false;        // S 是否按下

// 鼠标状态
bool   button_left   = false;
bool   button_middle = false;
bool   button_right  = false;
double lastx = 0, lasty = 0;

// ============================================================
// 辅助函数
// ============================================================
mjtNum clamp(mjtNum v, mjtNum lo, mjtNum hi) {
    return std::max(lo, std::min(v, hi));
}

// 更新运动方向并管理步态时钟
void syncDirection() {
    const int new_dir = (g_key_w ? 1 : 0) - (g_key_s ? 1 : 0);
    if (new_dir == 0 && g_direction != 0) {
        // 停止时重置步态时钟
        g_gait_time = 0.0;
    } else if (new_dir != 0 && g_direction == 0) {
        // 从静止启动：将步态时钟置于支撑中段相位
        //   offset=0  的腿：支撑相中点（x = x_bias，零跳变，承重腿不抖动）
        //   offset=0.5 的腿：摆动相中点（x = x_bias，零跳变，不承重所以 z 稍抬无影响）
        g_gait_time = 0.5 * kDutyFactor * kCyclePeriod;
    }
    g_direction = new_dir;
}

// ============================================================
// 足端目标计算
// ============================================================

// 静止：足端保持在默认站立位置
struct FootPt { mjtNum x, z; };

FootPt standingTarget(const LegCfg& leg) {
    return {leg.x_bias, kNominalHeight};
}

// 运动：trot 步态足端轨迹（前进方向，x 随后由方向系数翻转）
FootPt trotTarget(const LegCfg& leg, mjtNum time) {
    mjtNum phase = std::fmod(time / kCyclePeriod + leg.phase_offset, 1.0);
    if (phase < 0.0) phase += 1.0;

    const mjtNum half = 0.5 * kStepLength;
    FootPt pt;

    if (phase < kDutyFactor) {
        // 支撑相：足端向后划过，推动机身前进
        const mjtNum sp = phase / kDutyFactor;
        pt.x = leg.x_bias + (half - kStepLength * sp);
        pt.z = kNominalHeight + kStanceSink * std::sin(mjPI * sp);
    } else {
        // 摆动相：抬腿向前迈步
        const mjtNum sw = (phase - kDutyFactor) / (1.0 - kDutyFactor);
        pt.x = leg.x_bias + (-half + kStepLength * sw);
        pt.z = kNominalHeight - kStepHeight * std::sin(mjPI * sw);
    }
    return pt;
}

// ============================================================
// 机器人初始化
// ============================================================
void initRobot() {
    // 填充执行器 ID
    for (int i = 0; i < kNLegs; ++i) {
        g_hip_ids[i]  = mj_name2id(m, mjOBJ_ACTUATOR, kLegs[i].hip_act);
        g_knee_ids[i] = mj_name2id(m, mjOBJ_ACTUATOR, kLegs[i].knee_act);
        if (g_hip_ids[i] < 0 || g_knee_ids[i] < 0) {
            std::cerr << "[initRobot] actuator not found for leg " << i << "\n";
        }
    }

    // 机身初始位置（freejoint: qpos[0..6]）
    if (m->nq >= 7) {
        d->qpos[0] = 0.0; d->qpos[1] = 0.0; d->qpos[2] = kBodyHeight;
        d->qpos[3] = 1.0; d->qpos[4] = 0.0; d->qpos[5] = 0.0; d->qpos[6] = 0.0;
    }

    // 四腿直接设置到站立默认关节角，避免启动时抖动
    for (int i = 0; i < kNLegs; ++i) {
        auto [fx, fz] = standingTarget(kLegs[i]);
        quad::JointAngles a;
        if (!g_ik.Solve(fx, fz, a)) continue;

        int hj = mj_name2id(m, mjOBJ_JOINT, kLegs[i].hip_jnt);
        int kj = mj_name2id(m, mjOBJ_JOINT, kLegs[i].knee_jnt);
        if (hj >= 0) d->qpos[m->jnt_qposadr[hj]] = a.hip;
        if (kj >= 0) d->qpos[m->jnt_qposadr[kj]] = a.knee;

        if (g_hip_ids[i]  >= 0) d->ctrl[g_hip_ids[i]]  = a.hip;
        if (g_knee_ids[i] >= 0) d->ctrl[g_knee_ids[i]] = a.knee;
    }

    mj_forward(m, d);

    // 重置运动状态
    g_direction     = 0;
    g_gait_time     = 0.0;
    g_last_sim_time = d->time;
    g_key_w = false;
    g_key_s = false;
}

// ============================================================
// MuJoCo 控制回调（每个仿真步调用一次）
// ============================================================
void controller(const mjModel* model, mjData* data) {
    // 推进步态时钟（只在运动时累计 dt）
    const mjtNum dt = std::max<mjtNum>(0.0, data->time - g_last_sim_time);
    g_last_sim_time = data->time;
    if (g_direction != 0) g_gait_time += dt;

    for (int i = 0; i < kNLegs; ++i) {
        FootPt fp;
        if (g_direction == 0) {
            // ── 静止：保持默认站立位置 ──
            fp = standingTarget(kLegs[i]);
        } else {
            // ── 运动：步态轨迹 + 方向翻转 ──
            fp = trotTarget(kLegs[i], g_gait_time);
            // g_direction=1 → 前进（x 偏置保持正方向）
            // g_direction=-1 → 后退（x 偏置对称翻转）
            const mjtNum offset = fp.x - kLegs[i].x_bias;
            fp.x = kLegs[i].x_bias + g_direction * offset;
        }

        quad::JointAngles a;
        if (!g_ik.Solve(fp.x, fp.z, a)) continue;

        // 下发关节命令（钳位到执行器范围）
        const int ha = g_hip_ids[i], ka = g_knee_ids[i];
        if (ha >= 0) {
            mjtNum cmd = a.hip;
            if (model->actuator_ctrllimited[ha])
                cmd = clamp(cmd, model->actuator_ctrlrange[2*ha], model->actuator_ctrlrange[2*ha+1]);
            data->ctrl[ha] = cmd;
        }
        if (ka >= 0) {
            mjtNum cmd = a.knee;
            if (model->actuator_ctrllimited[ka])
                cmd = clamp(cmd, model->actuator_ctrlrange[2*ka], model->actuator_ctrlrange[2*ka+1]);
            data->ctrl[ka] = cmd;
        }
    }
}

// ============================================================
// GLFW 事件回调
// ============================================================
void onKey(GLFWwindow*, int key, int /*sc*/, int act, int /*mods*/) {
    if (key == GLFW_KEY_W) {
        g_key_w = (act != GLFW_RELEASE);
        syncDirection();
    } else if (key == GLFW_KEY_S) {
        g_key_s = (act != GLFW_RELEASE);
        syncDirection();
    } else if (key == GLFW_KEY_BACKSPACE && act == GLFW_PRESS) {
        mj_resetData(m, d);
        initRobot();
    }
}

void onMouseButton(GLFWwindow* win, int /*btn*/, int /*act*/, int /*mods*/) {
    button_left   = glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS;
    button_middle = glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    button_right  = glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS;
    glfwGetCursorPos(win, &lastx, &lasty);
}

void onMouseMove(GLFWwindow* win, double xpos, double ypos) {
    if (!button_left && !button_middle && !button_right) return;
    const double dx = xpos - lastx, dy = ypos - lasty;
    lastx = xpos; lasty = ypos;
    int w, h; glfwGetWindowSize(win, &w, &h);
    const bool shift = glfwGetKey(win, GLFW_KEY_LEFT_SHIFT)  == GLFW_PRESS
                    || glfwGetKey(win, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    mjtMouse act;
    if      (button_right) act = shift ? mjMOUSE_MOVE_H   : mjMOUSE_MOVE_V;
    else if (button_left)  act = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else                   act = mjMOUSE_ZOOM;
    mjv_moveCamera(m, act, dx/h, dy/h, &scn, &cam);
}

void onScroll(GLFWwindow*, double /*xoff*/, double yoff) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoff, &scn, &cam);
}

}  // namespace

// ============================================================
// main
// ============================================================
int main() {
    // 加载模型
    char err[1000] = "Could not load XML";
    m = mj_loadXML("../assets/xml_test.xml", nullptr, err, sizeof(err));
    if (!m) { std::cerr << "Load error: " << err << "\n"; return 1; }

    d = mj_makeData(m);
    initRobot();
    mjcb_control = controller;

    // 初始化 GLFW 窗口
    if (!glfwInit()) { mju_error("GLFW init failed"); }
    GLFWwindow* win = glfwCreateWindow(1200, 900, "Keyboard Control", nullptr, nullptr);
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);

    // 初始化 MuJoCo 渲染上下文
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // 初始相机视角
    cam.azimuth   = 90.0;
    cam.elevation = -30.0;
    cam.distance  = 1.8;
    cam.lookat[0] = cam.lookat[1] = 0.0;
    cam.lookat[2] = 0.25;

    // 注册回调
    glfwSetKeyCallback(win,         onKey);
    glfwSetMouseButtonCallback(win, onMouseButton);
    glfwSetCursorPosCallback(win,   onMouseMove);
    glfwSetScrollCallback(win,      onScroll);

    std::cout << "W=前进  S=后退  松开=停止  Backspace=重置\n";

    // 主循环：每帧步进约 1/60 秒仿真时间
    while (!glfwWindowShouldClose(win)) {
        const mjtNum t0 = d->time;
        while (d->time - t0 < 1.0 / 60.0) mj_step(m, d);

        int vw, vh;
        glfwGetFramebufferSize(win, &vw, &vh);
        const mjrRect rect{0, 0, vw, vh};
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(rect, &scn, &con);
        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    // 清理
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
    return 0;
}
