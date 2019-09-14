#include <hdl_graph_slam/view/edge_view.hpp>

#include <g2o/types/slam3d/edge_se3.h>

#include <hdl_graph_slam/view/line_buffer.hpp>
#include <hdl_graph_slam/view/edge_se3_view.hpp>
#include <hdl_graph_slam/view/edge_se3_plane_view.hpp>

namespace hdl_graph_slam {

EdgeView::EdgeView(g2o::HyperGraph::Edge* edge, LineBuffer& line_buffer) : edge(edge), line_buffer(line_buffer) {}

EdgeView::~EdgeView() {}

long EdgeView::id() const { return edge->id(); }

EdgeView::Ptr EdgeView::create(g2o::HyperGraph::Edge* edge, LineBuffer& line_buffer) {
  if (dynamic_cast<g2o::EdgeSE3*>(edge)) {
    return std::make_shared<EdgeSE3View>(edge, line_buffer);
  }

  if (dynamic_cast<g2o::EdgeSE3Plane*>(edge)) {
    return std::make_shared<EdgeSE3PlaneView>(edge, line_buffer);
  }

  return nullptr;
}
}  // namespace hdl_graph_slam