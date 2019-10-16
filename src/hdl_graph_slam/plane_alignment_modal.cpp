#include <hdl_graph_slam/plane_alignment_modal.hpp>

#include <g2o/types/slam3d_addons/vertex_plane.h>

namespace hdl_graph_slam {

PlaneAlignmentModal::PlaneAlignmentModal(std::shared_ptr<InteractiveGraphView>& graph) : graph(graph), plane_mode(1), information_scale(100.0f), robust_kernel(0, 0.01) {}
PlaneAlignmentModal::~PlaneAlignmentModal() {}

bool PlaneAlignmentModal::set_begin_plane(int plane_id) {
  auto found = graph->graph->vertices().find(plane_id);
  if (found == graph->graph->vertices().end()) {
    std::cerr << "vertex id " << plane_id << " not found" << std::endl;
    return false;
  }

  g2o::VertexPlane* vertex = dynamic_cast<g2o::VertexPlane*>(found->second);
  if (vertex == nullptr) {
    std::cerr << "vertex id " << plane_id << " is not plane" << std::endl;
    return false;
  }

  plane_begin.reset(new VertexPlaneView(vertex));
  return true;
}

bool PlaneAlignmentModal::set_end_plane(int plane_id) {
  auto found = graph->graph->vertices().find(plane_id);
  if (found == graph->graph->vertices().end()) {
    std::cerr << "vertex id " << plane_id << " not found" << std::endl;
    return false;
  }

  g2o::VertexPlane* vertex = dynamic_cast<g2o::VertexPlane*>(found->second);
  if (vertex == nullptr) {
    std::cerr << "vertex id " << plane_id << " is not plane" << std::endl;
    return false;
  }

  plane_end.reset(new VertexPlaneView(vertex));
  return true;
}

void PlaneAlignmentModal::close() {
  plane_begin = nullptr;
  plane_end = nullptr;
}

bool PlaneAlignmentModal::run() {
  bool close_window = false;
  if (ImGui::BeginPopupModal("plane alignment", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("Plane");
    const char* plane_modes[] = {"IDENTITY", "PARALLEL", "PERPENDICULAR"};
    ImGui::Combo("Plane mode", &plane_mode, plane_modes, IM_ARRAYSIZE(plane_modes));
    ImGui::DragFloat("Information scale", &information_scale, 1.0f, 0.01f, 10000.0f, "%.3f", 1.0f);

    robust_kernel.draw_ui();

    if (ImGui::Button("Add Edge")) {
      switch (plane_mode) {
        case 0:
          graph->add_edge_identity(plane_begin->vertex_plane, plane_end->vertex_plane, information_scale, robust_kernel.type(), robust_kernel.delta());
          break;
        case 1:
          graph->add_edge_parallel(plane_begin->vertex_plane, plane_end->vertex_plane, information_scale, robust_kernel.type(), robust_kernel.delta());
          break;
        case 2:
          graph->add_edge_perpendicular(plane_begin->vertex_plane, plane_end->vertex_plane, information_scale, robust_kernel.type(), robust_kernel.delta());
          break;
      }
      graph->optimize();
      close_window = true;
    }

    if (ImGui::Button("Cancel")) {
      close_window = true;
    }

    if (close_window) {
      ImGui::CloseCurrentPopup();
      plane_begin.reset();
      plane_end.reset();
    }

    ImGui::EndPopup();
  }
  return close_window;
}

void PlaneAlignmentModal::draw_gl(glk::GLSLShader& shader) {
  DrawFlags flags;

  Eigen::Matrix4f scale = (Eigen::Scaling<float>(7.5f, 7.5f, 0.15f) * Eigen::Isometry3f::Identity()).matrix();
  if (plane_begin) {
    plane_begin->draw(flags, shader, Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f), scale);
  }
  if (plane_end) {
    plane_end->draw(flags, shader, Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f), scale);
  }
}
}  // namespace hdl_graph_slam