#ifndef PROJECTION_CONTROL_HPP
#define PROJECTION_CONTROL_HPP

#include <Eigen/Core>

namespace guik {

class ProjectionControl {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ProjectionControl(const Eigen::Vector2i& size);
  ~ProjectionControl();

  void set_size(const Eigen::Vector2i& size) {
    this->size = size;
  }

  Eigen::Matrix4f projection_matrix() const;

  void draw_ui();

  void show();

private:
  bool show_window;
  Eigen::Vector2i size;

  int projection_mode;

  float fovy;
  float width;
  float near;
  float far;
};

}

#endif