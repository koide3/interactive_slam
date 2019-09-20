#ifndef HDL_GRAPH_SLAM_VERSION_MODAL_HPP
#define HDL_GRAPH_SLAM_VERSION_MODAL_HPP

#include <memory>

#include <imgui.h>

namespace hdl_graph_slam {

class VersionModal {
public:
  VersionModal();
  ~VersionModal();

  void open();
  bool run();
};

}  // namespace hdl_graph_slam

#endif