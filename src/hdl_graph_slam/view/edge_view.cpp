#include <hdl_graph_slam/view/edge_view.hpp>

namespace hdl_graph_slam {

EdgeView::EdgeView(g2o::HyperGraph::Edge* edge) : edge(edge) {}

EdgeView::~EdgeView() {}

long EdgeView::id() const { return edge->id(); }

EdgeView::Ptr EdgeView::create(g2o::HyperGraph::Edge* edge) {}
}