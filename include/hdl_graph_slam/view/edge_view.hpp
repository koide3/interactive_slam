#ifndef HDL_GRAPH_SLAM_EDGE_VIEW_HPP
#define HDL_GRAPH_SLAM_EDGE_VIEW_HPP

#include <Eigen/Core>

#include <glk/glsl_shader.hpp>
#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/view/drawable_object.hpp>

namespace hdl_graph_slam {

class EdgeView : public DrawableObject {
public:
  using Ptr = std::shared_ptr<EdgeView>;

  EdgeView(g2o::HyperGraph::Edge* edge);
  virtual ~EdgeView();

  static EdgeView::Ptr create(g2o::HyperGraph::Edge* edge);

  long id() const;

  virtual bool available() const = 0;

  virtual void draw(glk::GLSLShader& shader) = 0;

  virtual void draw(glk::GLSLShader& shader, const Eigen::Vector4f& color, const Eigen::Matrix4f& model_matrix) = 0;

private:
  EdgeView();

private:
  g2o::HyperGraph::Edge* edge;
};

}

#endif
