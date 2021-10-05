#include <guik/imgui_application.hpp>

#include <iostream>
#include <unordered_map>
#include <chrono>
#include <thread>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

namespace guik {

Application::Application() : window(nullptr), max_frame_rate(60.0) {}

Application ::~Application() {
  if(!window) {
    return;
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
}

// dirty implementation
std::unordered_map<GLFWwindow*, Application*> appmap;
void fb_size_callback(GLFWwindow* window, int width, int height) {
  appmap[window]->framebuffer_size_callback(Eigen::Vector2i(width, height));
}

bool Application::init(const char* window_name, const Eigen::Vector2i& size, const char* glsl_version) {
  glfwSetErrorCallback([](int err, const char* desc) { std::cerr << "glfw error " << err << ": " << desc << std::endl; });
  if(!glfwInit()) {
    std::cerr << "failed to initialize GLFW" << std::endl;
    return false;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

  window = glfwCreateWindow(size[0], size[1], window_name, nullptr, nullptr);
  if(window == nullptr) {
    return false;
  }
  appmap[window] = this;

  glfwSetFramebufferSizeCallback(window, fb_size_callback);

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  if(gl3wInit()) {
    std::cerr << "failed to initialize GL3W" << std::endl;
    return false;
  }

  // setup imgui
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();

  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  return true;
}

void Application::run() {
  while(!glfwWindowShouldClose(window)) {
    std::chrono::system_clock::time_point draw_start_time = std::chrono::system_clock::now();
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    draw_ui();

    ImGui::Render();

    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.6f, 0.6f, 0.6f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    draw_gl();

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);

    std::chrono::duration<double, std::milli> draw_time_ms = std::chrono::system_clock::now() - draw_start_time;
    if (draw_time_ms.count() < 1000/max_frame_rate) {
        std::chrono::duration<double, std::milli> delta_ms(1000/max_frame_rate - draw_time_ms.count());
        auto delta_ms_duration = std::chrono::duration_cast<std::chrono::milliseconds>(delta_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(delta_ms_duration.count()));
    }
  }
}

void Application::close() {
  glfwSetWindowShouldClose(window, 1);
}

Eigen::Vector2i Application::framebuffer_size() {
  int width, height;
  glfwGetFramebufferSize(window, &width, &height);
  return Eigen::Vector2i(width, height);
}

void Application::framebuffer_size_callback(const Eigen::Vector2i& size) {
  std::cout << "FB:" << size.transpose() << std::endl;
}

void Application::draw_ui() {
  ImGui::ShowDemoWindow();
}

void Application::draw_gl() {}

}  // namespace guik
