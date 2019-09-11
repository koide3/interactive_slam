#ifndef INTERACTIVE_EDGE_SE3_HPP
#define INTERACTIVE_EDGE_SE3_HPP

#include <g2o/types/slam3d/vertex_se3.h>
#include <hdl_graph_slam/view/edge_view.hpp>

namespace hdl_graph_slam {

class EdgeSE3View : public EdgeView {
public:
  EdgeSE3View(g2o::HyperGraph::Edge* edge)
  : EdgeView(edge)
  {}
  virtual ~EdgeSE3View() {}

  virtual bool available() const override { return true; }

  virtual void draw(glk::GLSLShader& shader) override {

  }

  virtual void draw(glk::GLSLShader& shader, const Eigen::Vector4f& color, const Eigen::Matrix4f& model_matrix) override {

  }

private:
  EdgeSE3View();
};
}

#endif