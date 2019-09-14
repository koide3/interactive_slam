#ifndef PLANE_DETECTION_MODAL_HPP
#define PLANE_DETECTION_MODAL_HPP

#include <memory>
#include <Eigen/Core>

#include <guik/gl_canvas.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <hdl_graph_slam/view/keyframe_view.hpp>
#include <hdl_graph_slam/view/interactive_graph_view.hpp>

namespace hdl_graph_slam {

struct RegionGrowingResult {
public:
  using Ptr = std::shared_ptr<RegionGrowingResult>;

  std::vector<InteractiveKeyFrame::Ptr> candidates;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  pcl::PointCloud<pcl::Normal>::Ptr normals;
  glk::PointCloudBuffer::Ptr cloud_buffer;
};

struct RANSACResult {
public:
  using Ptr = std::shared_ptr<RANSACResult>;

  std::vector<InteractiveKeyFrame::Ptr> candidates;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  glk::PointCloudBuffer::Ptr cloud_buffer;
};

class PlaneDetectionModal {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PlaneDetectionModal(InteractiveGraphView& graph);
  ~PlaneDetectionModal();

  void show();
  void set_center_point(const Eigen::Vector3f& point);

  RegionGrowingResult::Ptr region_growing();

  void draw_ui();
  void draw_gl(glk::GLSLShader& shader);

private:
  bool show_window;
  InteractiveGraphView& graph;

  Eigen::Vector3f center_point;

  int min_cluster_size;
  int max_cluster_size;
  int num_neighbors;
  float smoothness_threshold;
  float curvature_threshold;

  RegionGrowingResult::Ptr region_growing_result;
};

}

#endif