#ifndef HDL_GRAPH_SLAM_VIEW_PLANE_VERTEX_CACHE_HPP
#define HDL_GRAPH_SLAM_VIEW_PLANE_VERTEX_CACHE_HPP

#include <iostream>

#include <Eigen/Core>
#include <g2o/core/hyper_graph.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/types/slam3d_addons/vertex_plane.h>

namespace hdl_graph_slam {

class VertexPlaneCache : public g2o::HyperGraph::Data {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexPlaneCache(g2o::VertexPlane* vertex) : vertex(vertex) { update(); }

  virtual bool read(std::istream& is) override { return true; }
  virtual bool write(std::ostream& os) const override { return true; }

  void update() {
    int num_edges = 0;
    Eigen::Vector3d sum_pos(0.0, 0.0, 0.0);
    for (const auto& edge_ : vertex->edges()) {
      g2o::EdgeSE3Plane* edge = dynamic_cast<g2o::EdgeSE3Plane*>(edge_);
      if (edge == nullptr) {
        continue;
      }

      g2o::VertexSE3* v_se3 = dynamic_cast<g2o::VertexSE3*>(edge->vertices()[0]);
      num_edges++;
      sum_pos += v_se3->estimate().translation();
    }

    Eigen::Vector4d coeffs = vertex->estimate().coeffs();
    Eigen::Vector3d se3_center = sum_pos / num_edges;

    double t = -(coeffs[3] + (coeffs.head<3>().array() * se3_center.array()).sum()) / coeffs.head<3>().squaredNorm();

    Eigen::Vector3d position = se3_center + t * coeffs.head<3>();

    Eigen::Vector3d axis = std::abs(Eigen::Vector3d::UnitX().dot(coeffs.head<3>())) < 0.9 ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitZ();
    Eigen::Matrix3d rotation;
    rotation.block<3, 1>(0, 2) = coeffs.head<3>().normalized();
    rotation.block<3, 1>(0, 0) = rotation.block<3, 1>(0, 2).cross(axis);
    rotation.block<3, 1>(0, 1) = rotation.block<3, 1>(0, 2).cross(rotation.block<3, 1>(0, 0));

    pose_.setIdentity();
    pose_.translation() = position;
    pose_.linear() = rotation;
  }

  const Eigen::Isometry3d& pose() { return pose_; }

private:
  g2o::VertexPlane* vertex;
  Eigen::Isometry3d pose_;
};

}  // namespace hdl_graph_slam

#endif