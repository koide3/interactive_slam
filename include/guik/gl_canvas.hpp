#ifndef GLK_GL_CANVAS_CANVAS_HPP
#define GLK_GL_CANVAS_CANVAS_HPP

#include <imgui.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <glk/glsl_shader.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/texture_renderer.hpp>
#include <glk/pointcloud_buffer.hpp>

#include <guik/camera_control.hpp>

namespace guik {

enum COLOR_MODES {
  RAINBOW = 0,
  PHONG,
  NUM_COLOR_MODES
};

class GLCanvas {
public:
  /**
   * @brief Construct a new GLCanvas object
   *
   * @param data_directory
   * @param size
   */
  GLCanvas(const std::string& data_directory, const Eigen::Vector2i& size)
  : size(size),
  min_z(-2.0f),
  max_z(5.0f)
  {
    frame_buffer.reset(new glk::FrameBuffer(size, 2));
    shader.reset(new glk::GLSLShader());
    if(!shader->init(data_directory + "/shader/rainbow")) {
      shader.reset();
    }
    shader->set_uniform("z_range", Eigen::Vector2f(min_z, max_z));

    camera_control.reset(new guik::ArcCameraControl());
    texture_renderer.reset(new glk::TextureRenderer(data_directory));
  }

  /**
   * @brief
   *
   * @return true
   * @return false
   */
  bool ready() const {
    return frame_buffer && shader && camera_control && texture_renderer;
  }

  /**
   * @brief Set the Size object
   *
   * @param size
   */
  void set_size(const Eigen::Vector2i& size) {
    this->size = size;
    frame_buffer.reset(new glk::FrameBuffer(size, 2));
  }

  /**
   * @brief Set the color mode object
   *
   * @param mode
   */
  void set_color_mode(COLOR_MODES mode) {
    if(mode < 0 || mode >= NUM_COLOR_MODES) {
      std::cerr << "warning: invalid color mode " << mode << std::endl;
      return;
    }

    shader->set_uniform("color_mode", mode);
  }

  /**
   * @brief
   *
   */
  void bind() {
    frame_buffer->bind();
    glDisable(GL_SCISSOR_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    GLint clear_color[] = {-1, -1, -1, -1};
    glClearTexImage(frame_buffer->color(1).id(), 0, GL_RGBA_INTEGER, GL_INT, clear_color);

    shader->use();

    Eigen::Matrix4f view_matrix = camera_control->view_matrix();
    glm::mat4 proj = glm::perspective<float>(120.0, size[0] / static_cast<float>(size[1]), 1.0, 500.0);
    Eigen::Matrix4f projection_matrix = Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(proj));

    shader->set_uniform("view_matrix", view_matrix);
    shader->set_uniform("projection_matrix", projection_matrix);
    shader->set_uniform("info_values", Eigen::Vector4i(0, 128, 0, 255));
    shader->set_uniform("z_range", Eigen::Vector2f(min_z, max_z));

    glEnable(GL_DEPTH_TEST);
  }

  /**
   * @brief
   *
   */
  void unbind() {
    glDisable(GL_DEPTH_TEST);
    glFlush();

    frame_buffer->unbind();
  }

  /**
   * @brief
   *
   */
  void render_to_screen() {
    texture_renderer->draw(frame_buffer->color(0).id());
  }

  /**
   * @brief
   *
   */
  void mouse_control() {
    ImGuiIO& io = ImGui::GetIO();
    auto mouse_pos = ImGui::GetMousePos();
    auto drag_delta = ImGui::GetMouseDragDelta();

    Eigen::Vector2i p(mouse_pos.x, mouse_pos.y);

    for(int i = 0; i < 3; i++) {
      if(ImGui::IsMouseClicked(i)) {
        camera_control->mouse(p, i, true);
      }
      if(ImGui::IsMouseReleased(i)) {
        camera_control->mouse(p, i, false);
      }
      if(ImGui::IsMouseDragging(i)) {
        camera_control->drag(p, i);
      }

      camera_control->scroll(Eigen::Vector2f(io.MouseWheel, io.MouseWheelH));
    }
  }

  /**
   * @brief
   *
   * @param p
   * @param window
   * @return Eigen::Vector4i
   */
  Eigen::Vector4i pick_info(const Eigen::Vector2i& p, int window = 2) const {
    if(p[0] < 5 || p[1] < 5 || p[0] > size[0] - 5 || p[1] > size[1] - 5) {
      return Eigen::Vector4i(-1, -1, -1, -1);
    }

    std::vector<int> pixels = frame_buffer->color(1).read_pixels<int>(GL_RGBA_INTEGER, GL_INT);

    std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> ps;

    for(int i = -window; i <= window; i++) {
      for(int j = -window; j <= window; j++) {
        ps.push_back(Eigen::Vector2i(i, j));
      }
    }

    std::sort(ps.begin(), ps.end(), [=](const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs) { return lhs.norm() < rhs.norm(); });
    for(int i = 0; i < ps.size(); i++) {
      Eigen::Vector2i p_ = p + ps[i];
      int index = ((size[1] - p[1]) * size[0] + p_[0]) * 4;
      Eigen::Vector4i info = Eigen::Map<Eigen::Vector4i>(&pixels[index]);

      if(info[3] >= 0) {
        return info;
      }
    }

    return Eigen::Vector4i(-1, -1, -1, -1);
  }

  /**
   * @brief
   *
   * @param p
   * @param window
   * @return float
   */
  float pick_depth(const Eigen::Vector2i& p, int window = 2) const {
    if(p[0] < 5 || p[1] < 5 || p[0] > size[0] - 5 || p[1] > size[1] - 5) {
      return -1.0f;
    }

    std::vector<float> pixels = frame_buffer->depth().read_pixels<float>(GL_DEPTH_COMPONENT, GL_FLOAT);

    std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> ps;

    for(int i = -window; i <= window; i++) {
      for(int j = -window; j <= window; j++) {
        ps.push_back(Eigen::Vector2i(i, j));
      }
    }

    std::sort(ps.begin(), ps.end(), [=](const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs) { return lhs.norm() < rhs.norm(); });
    for(int i = 0; i < ps.size(); i++) {
      Eigen::Vector2i p_ = p + ps[i];
      int index = ((size[1] - p[1]) * size[0] + p_[0]);
      float depth = pixels[index];

      if(depth < 1.0f) {
        return depth;
      }
    }

    return 1.0f;
  }

  /**
   * @brief
   *
   * @param p
   * @param depth
   * @return Eigen::Vector3f
   */
  Eigen::Vector3f unproject(const Eigen::Vector2i& p, float depth) const {
    Eigen::Matrix4f view_matrix = camera_control->view_matrix();
    glm::mat4 proj = glm::perspective<float>(120.0, 1.0, 1.0, 500.0);
    Eigen::Matrix4f projection_matrix = Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(proj));

    Eigen::Matrix4f vp = projection_matrix * view_matrix;

    Eigen::Vector2f p_(static_cast<float>(p[0]) / size[0], static_cast<float>(p[1]) / size[1]);
    Eigen::Vector4f unprojected = vp.inverse() * Eigen::Vector4f(p_[0], 1.0f - p_[1], depth, 1.0f);

    return unprojected.head<3>();
  }

  void draw_ui() {
    ImGui::Begin("shader setting");
    ImGui::DragFloat("min_z", &min_z, 0.1f);
    ImGui::DragFloat("max_z", &max_z, 0.1f);

    ImGui::End();
  }

public:
  float min_z;
  float max_z;

  Eigen::Vector2i size;
  std::unique_ptr<glk::GLSLShader> shader;
  std::unique_ptr<glk::FrameBuffer> frame_buffer;
  std::unique_ptr<glk::TextureRenderer> texture_renderer;

  std::unique_ptr<guik::CameraControl> camera_control;
};

}  // namespace guik

#endif