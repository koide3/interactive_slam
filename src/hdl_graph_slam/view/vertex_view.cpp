#include <hdl_graph_slam/view/vertex_view.hpp>

#include <imgui.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimizable_graph.h>
#include <hdl_graph_slam/view/vertex_plane_view.hpp>

namespace hdl_graph_slam {

VertexView::VertexView(g2o::HyperGraph::Vertex* vertex) : vertex(vertex) {}

VertexView::~VertexView() {}

VertexView::Ptr VertexView::create(g2o::HyperGraph::Vertex* vertex) {
  g2o::VertexPlane* vertex_plane = dynamic_cast<g2o::VertexPlane*>(vertex);
  if (vertex_plane) {
    return std::make_shared<VertexPlaneView>(vertex_plane);
  }

  return nullptr;
}

void VertexView::context_menu() {
  g2o::OptimizableGraph::Vertex* v = dynamic_cast<g2o::OptimizableGraph::Vertex*>(vertex);

  g2o::Factory* factory = g2o::Factory::instance();
  ImGui::Text("Type: %s", factory->tag(v).c_str());
  ImGui::Text("Connected edges %d", v->edges().size());

  bool fixed = v->fixed();
  if(ImGui::Checkbox("Fixed", &fixed)) {
    v->setFixed(fixed);
  }
}

}  // namespace hdl_graph_slam