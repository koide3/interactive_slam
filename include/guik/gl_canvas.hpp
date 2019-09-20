#ifndef GLK_GL_CANVAS_CANVAS_HPP
#define GLK_GL_CANVAS_CANVAS_HPP

#include <imgui.h>

#include <glk/glsl_shader.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/texture_renderer.hpp>

#include <guik/camera_control.hpp>
#include <guik/projection_control.hpp>

namespace guik {

class GLCanvas {
public:
  GLCanvas(const std::string& data_directory, const Eigen::Vector2i& size);

  bool ready() const;

  void reset_camera();
  void set_size(const Eigen::Vector2i& size);
  void mouse_control();

  void bind();
  void unbind();

  void render_to_screen(int color_buffer_id = 0);

  Eigen::Vector4i pick_info(const Eigen::Vector2i& p, int window = 2) const;
  float pick_depth(const Eigen::Vector2i& p, int window = 2) const;
  Eigen::Vector3f unproject(const Eigen::Vector2i& p, float depth) const;

  void draw_ui();
  void show_projection_setting();


public:
  Eigen::Vector2i size;
  std::unique_ptr<glk::GLSLShader> shader;
  std::unique_ptr<glk::FrameBuffer> frame_buffer;
  std::unique_ptr<glk::TextureRenderer> texture_renderer;

  std::unique_ptr<guik::CameraControl> camera_control;
  std::unique_ptr<guik::ProjectionControl> projection_control;

private:
  float min_z;
  float max_z;
};

}  // namespace guik

#endif