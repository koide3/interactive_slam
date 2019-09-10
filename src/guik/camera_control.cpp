#include <guik/camera_control.hpp>

#include <memory>
#include <GL/gl3w.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <Eigen/Core>

namespace guik {

ArcCameraControl::ArcCameraControl() : center(0.0f, 0.0f, 0.0f), distance(10.0f), left_button_down(false), theta(0.0f), phi(-60.0f * M_PI / 180.0f) {
  left_button_down = false;
  middle_button_down = false;
}

ArcCameraControl::~ArcCameraControl() {}

void ArcCameraControl::mouse(const Eigen::Vector2i& p, int button, bool down) {
  if (button == 0) {
    left_button_down = down;
  }
  if (button == 2) {
    middle_button_down = down;
  }
  drag_last_pos = p;
}

void ArcCameraControl::drag(const Eigen::Vector2i& p, int button) {
  Eigen::Vector2i rel = p - drag_last_pos;

  if (left_button_down) {
    theta -= rel[0] * 0.01f;
    phi -= rel[1] * 0.01f;

    phi = std::min(M_PI_2 - 0.01, std::max(-M_PI_2 + 0.01, phi));
  }

  if (middle_button_down) {
    center += Eigen::AngleAxisf(theta + M_PI_2, Eigen::Vector3f::UnitZ()) * Eigen::Vector3f(-rel[0], rel[1], 0.0f) * distance * 0.001f;
  }

  drag_last_pos = p;
}

void ArcCameraControl::scroll(const Eigen::Vector2f& rel) {
  if (rel[0] > 0) {
    distance = distance * 0.8f;
  } else if (rel[0] < 0) {
    distance = distance * 1.2f;
  }

  distance = std::max(0.1, distance);
}

Eigen::Quaternionf ArcCameraControl::rotation() const { return Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitY()); }

Eigen::Matrix4f ArcCameraControl::view_matrix() const {
  Eigen::Vector3f offset = rotation() * Eigen::Vector3f(distance, 0.0f, 0.0f);
  Eigen::Vector3f eye = center + offset;

  glm::mat4 mat = glm::lookAt(glm::vec3(eye[0], eye[1], eye[2]), glm::vec3(center[0], center[1], center[2]), glm::vec3(0.0f, 0.0f, 1.0f));
  return Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(mat));
}

}  // namespace guik
