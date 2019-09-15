#ifndef EDGE_PLANE_VIEW_HPP
#define EDGE_PLANE_VIEW_HPP

#include <g2o/edge_plane_parallel.hpp>
#include <g2o/types/slam3d_addons/edge_plane.h>

#include <hdl_graph_slam/view/edge_view.hpp>
#include <hdl_graph_slam/view/vertex_plane_cache.hpp>

namespace hdl_graph_slam {

class EdgePlaneView : public EdgeView {
public:
  EdgePlaneView(g2o::HyperGraph::Edge* edge, LineBuffer& line_buffer) : EdgeView(edge, line_buffer) {
    std::cout << "plane edge view created" << std::endl;
    v1 = dynamic_cast<g2o::VertexPlane*>(edge->vertices()[0]);
    v2 = dynamic_cast<g2o::VertexPlane*>(edge->vertices()[1]);
    assert(v1 != nullptr && v2 != nullptr);
  }

  virtual ~EdgePlaneView() override {}

  virtual bool available() const override { return true; }

  virtual void draw(const DrawFlags& flags, glk::GLSLShader& shader) override {
    if (!flags.draw_edges || !flags.draw_plane_edges) {
      return;
    }

    VertexPlaneCache* cache1 = dynamic_cast<VertexPlaneCache*>(v1->userData());
    VertexPlaneCache* cache2 = dynamic_cast<VertexPlaneCache*>(v2->userData());
    if (cache1 == nullptr || cache2 == nullptr) {
      std::cerr << "warning: vertex plane cache has not been created!!" << std::endl;
      return;
    }

    Eigen::Vector3f p1 = cache1->pose().translation().cast<float>();
    Eigen::Vector3f p2 = cache2->pose().translation().cast<float>();

    Eigen::Vector4f c1(0.0f, 0.0f, 1.0f, 1.0f);
    Eigen::Vector4f c2(0.0f, 0.0f, 1.0f, 1.0f);
    Eigen::Vector4i info(EDGE, edge->id(), 0, 0);

    line_buffer.add_line(p1, p2, c1, c2, info);
  }

  static bool is_plane_edge(g2o::HyperGraph::Edge* edge) { return dynamic_cast<g2o::EdgePlane*>(edge) || dynamic_cast<g2o::EdgePlaneParallel*>(edge) || dynamic_cast<g2o::EdgePlanePerpendicular*>(edge); }

private:
  g2o::VertexPlane* v1;
  g2o::VertexPlane* v2;
};

};  // namespace hdl_graph_slam

#endif