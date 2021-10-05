#ifndef GUIK_IMGUI_APPLICATION_HPP
#define GUIK_IMGUI_APPLICATION_HPP

#include <iostream>

#include <imgui.h>
#include <GL/gl3w.h>
#include <Eigen/Core>

class GLFWwindow;

namespace guik {
class Application {
public:
  Application();
  virtual ~Application();

  virtual bool init(const char* window_name, const Eigen::Vector2i& size, const char* glsl_version = "#version 330");

  void run();

  void close();

  Eigen::Vector2i framebuffer_size();
  virtual void framebuffer_size_callback(const Eigen::Vector2i& size);

  inline void set_max_frame_rate(double frame_rate) { max_frame_rate = frame_rate; }

  virtual void draw_ui();

  virtual void draw_gl();

protected:
  GLFWwindow* window;
  double max_frame_rate;
};

}  // namespace guik

#endif