#ifndef HDL_GRAPH_SLAM_VIEW_VERTEX_VIEW_HPP
#define HDL_GRAPH_SLAM_VIEW_VERTEX_VIEW_HPP

#include <Eigen/Core>
#include <g2o/core/hyper_graph.h>

#include <glk/glsl_shader.hpp>
#include <hdl_graph_slam/view/drawable_object.hpp>

namespace hdl_graph_slam {

class VertexView : public DrawableObject {
public:
  using Ptr = std::shared_ptr<VertexView>;

  VertexView(g2o::HyperGraph::Vertex* vertex);
  virtual ~VertexView();

  static VertexView::Ptr create(g2o::HyperGraph::Vertex* vertex);

  long id() const { return vertex->id(); }

  virtual void context_menu();

private:
  VertexView();

protected:
  g2o::HyperGraph::Vertex* vertex;
};

}  // namespace hdl_graph_slam

#endif