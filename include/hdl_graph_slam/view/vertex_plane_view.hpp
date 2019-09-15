#ifndef HDL_GRAPH_SLAM_VERTEX_PLANE_VIEW_HPP
#define HDL_GRAPH_SLAM_VERTEX_PLANE_VIEW_HPP

#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glk/glsl_shader.hpp>
#include <glk/primitives/primitives.hpp>

#include <g2o/edge_se3_plane.hpp>
#include <g2o/types/slam3d_addons/vertex_plane.h>

#include <hdl_graph_slam/view/vertex_view.hpp>
#include <hdl_graph_slam/view/vertex_plane_cache.hpp>

namespace g2o {
class VertexPlane;
}

namespace hdl_graph_slam {

class VertexPlaneView : public VertexView {
public:
  using Ptr = std::shared_ptr<VertexPlaneView>;

  VertexPlaneView(g2o::VertexPlane* vertex_plane) : VertexView(vertex_plane), vertex_plane(vertex_plane) {
    if (vertex_plane->userData() == nullptr) {
      vertex_plane->addUserData(new VertexPlaneCache(vertex_plane));
    }
  }

  virtual ~VertexPlaneView() override {}

  virtual bool available() const override { return true; }

  virtual void draw(const DrawFlags& flags, glk::GLSLShader& shader) override {
    VertexPlaneCache* cache = dynamic_cast<VertexPlaneCache*>(vertex_plane->userData());
    cache->update();

    if (!flags.draw_verticies || !flags.draw_plane_vertices) {
      return;
    }

    Eigen::Matrix4f model_matrix = (cache->pose() * Eigen::Scaling(5.0, 5.0, 0.1)).matrix().cast<float>();

    shader.set_uniform("color_mode", 1);
    shader.set_uniform("model_matrix", model_matrix);
    shader.set_uniform("material_color", Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f));
    shader.set_uniform("info_values", Eigen::Vector4i(VERTEX | PLANE, vertex->id(), 0, 0));

    auto& cube = glk::Primitives::instance()->primitive(glk::Primitives::CUBE);
    cube.draw(shader);
  }

  virtual void draw(const DrawFlags& flags, glk::GLSLShader& shader, const Eigen::Vector4f& color, const Eigen::Matrix4f& model_matrix_) override {
    VertexPlaneCache* cache = dynamic_cast<VertexPlaneCache*>(vertex_plane->userData());
    cache->update();

    if (!flags.draw_verticies || !flags.draw_plane_vertices) {
      return;
    }

    Eigen::Matrix4f model_matrix = cache->pose().matrix().cast<float>() * model_matrix_;

    shader.set_uniform("color_mode", 1);
    shader.set_uniform("model_matrix", model_matrix);
    shader.set_uniform("material_color", color);
    shader.set_uniform("info_values", Eigen::Vector4i(VERTEX | PLANE, vertex->id(), 0, 0));

    auto& cube = glk::Primitives::instance()->primitive(glk::Primitives::CUBE);
    cube.draw(shader);
  }

public:
  g2o::VertexPlane* vertex_plane;
};

}  // namespace hdl_graph_slam

#endif