#include <iostream>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstring>
#include <cmath>
extern "C" {
#include "leg_ik.h"
}

// MuJoCo 核心数据结构全局指针
mjModel* m = NULL;                  // MuJoCo 模型
mjData* d = NULL;                   // MuJoCo 动态数据
mjvCamera cam;                      // 抽象相机
mjvOption opt;                      // 渲染选项
mjvScene scn;                       // 抽象场景
mjrContext con;                     // 自定义GPU渲染上下文

// 鼠标交互状态
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

void mycontroller(const mjModel* m, mjData* d) {
    // 获取当前仿真时间 (可以用于轨迹生成)
    double time = d->time;

    // 机械狗步态参数配置（对角小跑 Trot Gait）
    float base_height = 0.065f;   // 站立的腿长 (留一点弯曲度，腿总长是 0.075m)
    float step_height = 0.02f;    // 抬腿的高度
    float step_length = 0.025f;   // 步态前后跨度的一半
    float freq = 2.5f * M_PI;     // 迈步的频率

    // 计算相位（用于生成半椭圆前摆+半直线后蹬轨）
    float phase1 = time * freq;
    float phase2 = time * freq + M_PI; // 第二组差半个周期 (180度)

    // 第一组 (腿1右后, 腿4左前): 
    // x = -cos(phase). 当 sin(p)>0 时向前抬腿跨越，当 sin(p)<=0 时紧贴地面向后蹬
    float x1 = -step_length * std::cos(phase1);
    float y1 = base_height;
    if (std::sin(phase1) > 0) {
        y1 -= step_height * std::sin(phase1); // 悬空期：缩短身腿距离，抬脚
    }
    float p1[2];
    leg_ik(x1, y1, p1);
    float a1_0 = p1[0];
    float a1_1 = p1[1];
    
    // 第二组 (腿2右前, 腿3左后):
    float x2 = -step_length * std::cos(phase2);
    float y2 = base_height;
    if (std::sin(phase2) > 0) {
        y2 -= step_height * std::sin(phase2);
    }
    float p2[2];
    leg_ik(x2, y2, p2);
    float a2_0 = p2[0];
    float a2_1 = p2[1];

    float hip1  = (90.0f - a1_0) * M_PI / 180.0f;
    float knee1 = (180.0f - a1_1) * M_PI / 180.0f;
    
    float hip2  = (90.0f - a2_0) * M_PI / 180.0f;
    float knee2 = (180.0f - a2_1) * M_PI / 180.0f;

    // link1, link1_2
    d->ctrl[0] = hip1; 
    d->ctrl[1] = knee1;
    
    // link2, link2_2
    d->ctrl[2] = hip2; 
    d->ctrl[3] = knee2;
    
    // link3, link3_2
    d->ctrl[4] = hip2; 
    d->ctrl[5] = knee2;
    
    // link4, link4_2
    d->ctrl[6] = hip1; 
    d->ctrl[7] = knee1;
}


// 键盘回调函数
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    // 按下 Backspace 键重置仿真
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// 鼠标点击回调函数
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx, &lasty);
}

// 鼠标移动回调函数（控制视角）
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    if (!button_left && !button_middle && !button_right) return;

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

// 鼠标滚轮回调（缩放视角）
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}


int main() {
    // 1. 加载模型（默认加载第二章的测试模型）
    char error[1000] = "Could not load binary model";
    m = mj_loadXML("../assets/xml_test.xml", 0, error, 1000);
    if (!m) {
        std::cerr << "Load model error: " << error << std::endl;
        return 1;
    }

    // 2. 初始化数据
    d = mj_makeData(m);

    // ==========================================================
    // 3. 注册你的控制回调函数 (核心)
    // 这样 mj_step() 在每次计算动力学时都会自动执行 mycontroller 
    // ==========================================================
    mjcb_control = mycontroller;

    // 4. 初始化 GLFW 和 视窗
    if (!glfwInit()) mju_error("Could not initialize GLFW");
    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo Control Interface", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // 开启垂直同步

    // 5. 初始化 MuJoCo 渲染/可视化组件
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    // 配置相机初始位置使其看清模型
    cam.azimuth = 90;
    cam.elevation = -45;
    cam.distance = 2.5;
    cam.lookat[0] = 0; cam.lookat[1] = 0; cam.lookat[2] = 0.5;

    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // 6. 注册鼠标键盘回调
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // 7. 仿真和渲染主循环
    while (!glfwWindowShouldClose(window)) {
        // 当前视窗的系统时间
        mjtNum simstart = d->time;

        // 推进仿真（追赶系统真实时间，设定60fps左右的渲染率）
        // 在这之中，mj_step()内部会自动调用你写的 mycontroller()
        while (d->time - simstart < 1.0 / 60.0) {
            mj_step(m, d);
        }

        // 获取当前渲染缓冲区尺寸
        int viewport_width, viewport_height;
        glfwGetFramebufferSize(window, &viewport_width, &viewport_height);
        mjrRect rect = {0, 0, viewport_width, viewport_height};

        // 更新场景并渲染
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(rect, &scn, &con);

        // 缓冲区交换和事件处理
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // 8. 退出并清理资源
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();

    return 0;
}
