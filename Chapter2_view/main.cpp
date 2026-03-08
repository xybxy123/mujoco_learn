// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <thread>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

// MuJoCo data structures
mjModel *m = NULL; // MuJoCo model
mjData *d = NULL;  // MuJoCo data
mjvCamera cam;     // abstract camera
mjvOption opt;     // visualization options
mjvScene scn;      // abstract scene
mjrContext con;    // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// keyboard callback
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
}

// mouse button callback
void mouse_button(GLFWwindow *window, int button, int act, int mods) {
  // update button state
  button_left =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &scn, &cam);
}

// main function
int main(int argc, const char **argv) {

  char error[1000] = "Could not load binary model";

  m = mj_loadXML("../pointer.xml", 0, error, 1000);
  // 如果加载失败，打印错误并退出
  if (!m) {
    std::cerr << "加载XML失败: " << error << std::endl;
    return 1;
  }

  // make data
  d = mj_makeData(m);
  if (!d) {
    std::cerr << "创建mjData失败" << std::endl;
    mj_deleteModel(m); // 释放已加载的model
    return 1;
  }

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  GLFWwindow *window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  float cnt = 0;
  auto step_start = std::chrono::high_resolution_clock::now();
  while (!glfwWindowShouldClose(window)) {

    if (m && d) {
      // 测试step
      d->ctrl[1] = sin(cnt);
      mj_step(m, d);

      cnt += 0.01;

      // 同步时间
      auto current_time = std::chrono::high_resolution_clock::now();
      double elapsed_sec =
          std::chrono::duration<double>(current_time - step_start).count();
      double time_until_next_step = m->opt.timestep * 5 - elapsed_sec;
      if (time_until_next_step > 0.0) {
        auto sleep_duration =
            std::chrono::duration<double>(time_until_next_step);
        std::this_thread::sleep_for(sleep_duration);
      }
      step_start = current_time; // 修复：每次循环更新起始时间
    }

    // 测试step1 step2
    //  mj_step1(m, d);
    //  d->ctrl[1] = sin(cnt);
    //  mj_step2(m, d);

    // 测试forward
    // d->ctrl[0] = sin(cnt);
    // // d->qpos[0] = sin(cnt);
    // mj_forward(m, d);
    // std::cout << "qacc  qvel  qpos" << std::endl;
    // for (int i = 0; i < m->nq; i++) {
    //   std::cout << d->qacc[i] << "  " << d->qvel[i] << "  " << d->qpos[i]
    //             << "  " << std::endl;
    // }
    // std::cout << std::endl;

    // 测试inverse
    //  d->qacc[0] = sin(cnt);
    //  d->qpos[0] = 0;
    //  d->qvel[0] = 0;
    //  mj_inverse(m, d);
    //  std::cout << "qfrc_inverse" << std::endl;
    //  for (int i = 0; i < m->nv; i++) {
    //    std::cout << d->qfrc_inverse[i];
    //  }
    //  std::cout << std::endl;

    // cnt += 0.01;

    // 同步时间
    // auto current_time = std::chrono::high_resolution_clock::now();
    // double elapsed_sec =
    //     std::chrono::duration<double>(current_time - step_start).count();
    // double time_until_next_step = m->opt.timestep * 5 - elapsed_sec;
    // if (time_until_next_step > 0.0) {
    //   auto sleep_duration =
    //   std::chrono::duration<double>(time_until_next_step);
    //   std::this_thread::sleep_for(sleep_duration);
    // }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  // free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif

  return 1;
}
