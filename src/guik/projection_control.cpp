#include <guik/projection_control.hpp>

#include <imgui.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace guik {

ProjectionControl::ProjectionControl(const Eigen::Vector2i& size)
: show_window(false),
size(size),
projection_mode(0),
fovy(30.0f),
width(10.0f),
near(1.0f),
far(1000.0f)
{}

ProjectionControl::~ProjectionControl() {}

Eigen::Matrix4f ProjectionControl::projection_matrix() const {
  double aspect_ratio = size[0] / static_cast<float>(size[1]);

  glm::mat4 proj;
  if(projection_mode == 0) {
    proj = glm::perspective<float>(fovy * M_PI / 180.0, aspect_ratio, near, far);
  } else {
    proj = glm::ortho<float>(-width / 2.0f, width / 2.0f, -width / 2.0f / aspect_ratio, width / 2.0 / aspect_ratio, near, far);
  }

  return Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(proj));
}

void ProjectionControl::show() {
  show_window = true;
}

void ProjectionControl::draw_ui() {
  if(!show_window) {
    return;
  }

  ImGui::Begin("projection control", &show_window, ImGuiWindowFlags_AlwaysAutoResize);
  const char* modes[] = { "PERSPECTIVE", "ORTHOGONAL" };
  ImGui::Combo("Mode", &projection_mode, modes, IM_ARRAYSIZE(modes));
  if(projection_mode == 0){
    ImGui::DragFloat("FOV", &fovy, 0.1f, 1.0f, 180.0f);
  } else {
    ImGui::DragFloat("Width", &width, 0.1f, 1.0f, 1000.0f);
  }

  ImGui::DragFloat("Near", &near, 1.0f, 0.1f, far);
  ImGui::DragFloat("Far", &far, 1.0f, near, 10000.0f);

  ImGui::End();
}
}