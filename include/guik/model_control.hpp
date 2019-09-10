#ifndef GUIK_MODEL_CONTROL_HPP
#define GUIK_MODEL_CONTROL_HPP

#include <sstream>

#include <imgui.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace guik {

class ModelControl {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ModelControl(const std::string& name);

  void draw_ui();
  Eigen::Matrix4f model_matrix() const;

private:
  std::string name;
  Eigen::Affine3f pose;
};

}  // namespace guik

#endif