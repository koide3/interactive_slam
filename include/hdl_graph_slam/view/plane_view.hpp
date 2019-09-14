#ifndef HDL_GRAPH_SLAM_PLANE_VIEW_HPP
#define HDL_GRAPH_SLAM_PLANE_VIEW_HPP

#include <memory>
#include <hdl_graph_slam/view/drawable_object.hpp>

namespace g2o {
  class VertexPlane;
}

namespace hdl_graph_slam {

class PlaneView : public DrawableObject {
public:
  using Ptr = std::shared_ptr<PlaneView>;

  PlaneView(g2o::VertexPlane* vertex_plane);
  virtual ~PlaneView() override;

  virtual bool available() const override { return true; }

  virtual void draw(glk::GLSLShader& shader) override;

  virtual void draw(glk::GLSLShader& shader, const Eigen::Vector4f& color, const Eigen::Matrix4f& model_matrix) override;

private :
  g2o::VertexPlane* vertex_plane;
};

}

#endif