#ifndef INTERACTIVE_EDGE_SE3_VIEW_HPP
#define INTERACTIVE_EDGE_SE3_VIEW_HPP

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <hdl_graph_slam/view/edge_view.hpp>

namespace hdl_graph_slam {

class EdgeSE3View : public EdgeView {
public:
  EdgeSE3View(g2o::HyperGraph::Edge* edge, LineBuffer& line_buffer) : EdgeView(edge, line_buffer) {
    v1 = dynamic_cast<g2o::VertexSE3*>(edge->vertices()[0]);
    v2 = dynamic_cast<g2o::VertexSE3*>(edge->vertices()[1]);

    assert(v1 != nullptr && v2 != nullptr);
  }

  virtual ~EdgeSE3View() override {}

  virtual bool available() const override { return true; }

  virtual void draw(const DrawFlags& flags, glk::GLSLShader& shader) override {
    if (!flags.draw_edges || !flags.draw_se3_edges) {
      return;
    }

    Eigen::Vector3f p1 = v1->estimate().translation().cast<float>();
    Eigen::Vector3f p2 = v2->estimate().translation().cast<float>();

    Eigen::Vector4f color(1.0f, 0.0f, 0.0f, 1.0f);
    Eigen::Vector4i info(EDGE, edge->id(), 0, 0);

    line_buffer.add_line(p1, p2, color, color, info);
  }

private:
  EdgeSE3View();

private:
  g2o::VertexSE3* v1;
  g2o::VertexSE3* v2;
};
}  // namespace hdl_graph_slam

#endif