#ifndef EDGE_SE3_PLANE_VIEW_HPP
#define EDGE_SE3_PLANE_VIEW_HPP

#include <g2o/edge_se3_plane.hpp>

#include <hdl_graph_slam/view/edge_view.hpp>
#include <hdl_graph_slam/view/vertex_plane_cache.hpp>

namespace hdl_graph_slam {

class EdgeSE3PlaneView : public EdgeView {
public:
  EdgeSE3PlaneView(g2o::HyperGraph::Edge* edge, LineBuffer& line_buffer) : EdgeView(edge, line_buffer) {
    v_se3 = dynamic_cast<g2o::VertexSE3*>(edge->vertices()[0]);
    v_plane = dynamic_cast<g2o::VertexPlane*>(edge->vertices()[1]);

    assert(v_se3 != nullptr && v_plane != nullptr);
  }

  virtual ~EdgeSE3PlaneView() override {}

  virtual bool available() const override { return true; }

  virtual void draw(const DrawFlags& flags, glk::GLSLShader& shader) override {
    if (!flags.draw_edges || !flags.draw_se3_plane_edges) {
      return;
    }

    if (!flags.draw_floor_edges && v_plane->id() < 5) {
      return;
    }

    VertexPlaneCache* cache = dynamic_cast<VertexPlaneCache*>(v_plane->userData());
    if (cache == nullptr) {
      std::cerr << "warning: vertex plane cache has not been created!!" << std::endl;
      return;
    }

    Eigen::Vector3f p1 = v_se3->estimate().translation().cast<float>();
    Eigen::Vector3f p2 = cache->pose().translation().cast<float>();

    Eigen::Vector4f c1(1.0f, 0.0f, 0.0f, 1.0f);
    Eigen::Vector4f c2(0.0f, 1.0f, 0.0f, 1.0f);
    Eigen::Vector4i info(EDGE, edge->id(), 0, 0);

    line_buffer.add_line(p1, p2, c1, c2, info);
  }

private:
  EdgeSE3PlaneView();

private:
  g2o::VertexSE3* v_se3;
  g2o::VertexPlane* v_plane;
};

};  // namespace hdl_graph_slam

#endif