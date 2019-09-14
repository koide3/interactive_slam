#include <hdl_graph_slam/view/vertex_view.hpp>

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

}  // namespace hdl_graph_slam