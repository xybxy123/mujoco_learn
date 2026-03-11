#include <iostream>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "gait_controller.h"

namespace {

quad::GaitController g_controller;

mjModel* m = nullptr;
mjData* d = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

bool button_left  = false;
bool button_middle = false;
bool button_right = false;
bool key_w = false;
bool key_s = false;
double lastx = 0;
double lasty = 0;

void syncDirection() {
    const int dir = (key_w ? 1 : 0) - (key_s ? 1 : 0);
    g_controller.SetMotionDirection(dir);
}

void mycontroller(const mjModel* model, mjData* data) {
    g_controller.Update(model, data);
}

void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    (void)window;
    (void)scancode;
    (void)mods;

    if (key == GLFW_KEY_W) {
        key_w = (act != GLFW_RELEASE);
        syncDirection();
        return;
    }
    if (key == GLFW_KEY_S) {
        key_s = (act != GLFW_RELEASE);
        syncDirection();
        return;
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
        mj_resetData(m, d);
        g_controller.Initialize(m);
        g_controller.SetInitialPose(m, d);
        key_w = false; key_s = false;
        syncDirection();
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

}  // namespace

int main() {
    char error[1000] = "Could not load XML model";
    m = mj_loadXML("../assets/xml_test.xml", nullptr, error, sizeof(error));
    if (!m) {
        std::cerr << "Load model error: " << error << std::endl;
        return 1;
    }

    d = mj_makeData(m);
    mjcb_control = mycontroller;
    
    g_controller.Initialize(m);
    g_controller.SetInitialPose(m, d);

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

    std::cout << "W=前进  S=后退  松开=停止  Backspace=重置" << std::endl;

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
