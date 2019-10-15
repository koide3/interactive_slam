#ifndef HDL_GRAPH_SLAM_ROBUST_KERNELS_HPP
#define HDL_GRAPH_SLAM_ROBUST_KERNELS_HPP

#include <string>
#include <vector>
#include <imgui.h>

namespace hdl_graph_slam {

class RobustKernels {
public:
  RobustKernels(int type, float delta)
  : kernel_type(type),
    kernel_delta(delta),
    kernels({"NONE", "Huber", "Cauchy", "DCS", "Fair", "GemanMcClure", "PseudoHuber", "Staturated", "Tukey", "Welsch"})
  {}

  RobustKernels()
  : kernel_type(2),
    kernel_delta(0.01),
    kernels({"NONE", "Huber", "Cauchy", "DCS", "Fair", "GemanMcClure", "PseudoHuber", "Staturated", "Tukey", "Welsch"})
  {}
  ~RobustKernels() {}

  void draw_ui() {
    ImGui::Text("Robust kernel");
    ImGui::Combo("Kernel type", &kernel_type, kernels.data(), kernels.size());
    ImGui::DragFloat("Kernel delta", &kernel_delta, 1e-4f, 1e-4f, 10.0f, "%.4f", 1.0f);
  }

  std::string type() const {
    return kernels[kernel_type];
  }

  float delta() const {
    return kernel_delta;
  }

private:
  int kernel_type;
  float kernel_delta;
  std::vector<const char*> kernels;
};
}

#endif