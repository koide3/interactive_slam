#include <hdl_graph_slam/view/edge_view.hpp>

#include <imgui.h>
#include <g2o/core/factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/slam3d/edge_se3.h>

#include <hdl_graph_slam/view/line_buffer.hpp>
#include <hdl_graph_slam/view/edge_se3_view.hpp>
#include <hdl_graph_slam/view/edge_plane_view.hpp>
#include <hdl_graph_slam/view/edge_se3_plane_view.hpp>

namespace hdl_graph_slam {

EdgeView::Ptr EdgeView::create(g2o::HyperGraph::Edge* edge, LineBuffer& line_buffer) {
  if(dynamic_cast<g2o::EdgeSE3*>(edge)) {
    return std::make_shared<EdgeSE3View>(edge, line_buffer);
  }

  if(dynamic_cast<g2o::EdgeSE3Plane*>(edge)) {
    return std::make_shared<EdgeSE3PlaneView>(edge, line_buffer);
  }

  if(EdgePlaneView::is_plane_edge(edge)) {
    return std::make_shared<EdgePlaneView>(edge, line_buffer);
  }

  return nullptr;
}

EdgeView::EdgeView(g2o::HyperGraph::Edge* edge, LineBuffer& line_buffer) : edge(edge), line_buffer(line_buffer) {}

EdgeView::~EdgeView() {}

long EdgeView::id() const { return edge->id(); }

void EdgeView::context_menu() {
  g2o::Factory* factory = g2o::Factory::instance();

  g2o::OptimizableGraph::Edge* e = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge);

  ImGui::Text("Type:%s", factory->tag(e).c_str());

  g2o::RobustKernel* robust_kernel = e->robustKernel();
  if(robust_kernel == nullptr) {
    ImGui::Text("Robust kernel:NONE");
  } else {
    std::string robust_kernel_type = "UNKNOWN";
    if(dynamic_cast<g2o::RobustKernelHuber*>(robust_kernel)) {
      robust_kernel_type = "Huber";
    }

    ImGui::Text("\nRobust kernel");
    ImGui::Text("Type:%s", robust_kernel_type.c_str());

    ImGui::SameLine();
    float delta = robust_kernel->delta();
    ImGui::PushItemWidth(128);
    if(ImGui::DragFloat("##Delta", &delta, 0.001f, 0.001f, 1000.0f, "Delta:%.3f")) {
      robust_kernel->setDelta(delta);
    }
  }
}


}  // namespace hdl_graph_slam