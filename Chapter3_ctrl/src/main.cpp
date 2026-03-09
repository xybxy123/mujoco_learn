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
