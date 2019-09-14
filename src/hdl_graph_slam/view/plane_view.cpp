#include <hdl_graph_slam/view/plane_view.hpp>

#include <g2o/edge_se3_plane.hpp>
#include <g2o/types/slam3d_addons/vertex_plane.h>

namespace hdl_graph_slam {

PlaneView::PlaneView(g2o::VertexPlane* vertex_plane)
: vertex_plane(vertex_plane)
{}
PlaneView::~PlaneView(){}

void PlaneView::draw(glk::GLSLShader& shader) {
  for(const auto& edge_: vertex_plane->edges()) {
    g2o::EdgeSE3Plane* edge = dynamic_cast<g2o::EdgeSE3Plane*>(edge_);
    if(edge == nullptr) {
      continue;
    }

  }
}

void PlaneView::draw(glk::GLSLShader& shader, const Eigen::Vector4f& color, const Eigen::Matrix4f& model_matrix) {}
}