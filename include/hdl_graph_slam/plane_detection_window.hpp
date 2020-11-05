#ifndef PLANE_DETECTION_WINDOW_HPP
#define PLANE_DETECTION_WINDOW_HPP

#include <memory>
#include <Eigen/Core>

#include <guik/gl_canvas.hpp>
#include <guik/progress_modal.hpp>
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

struct PlaneDetectionResult {
public:
  using Ptr = std::shared_ptr<PlaneDetectionResult>;

  std::vector<InteractiveKeyFrame::Ptr> candidates;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> candidate_inliers;
  std::vector<glk::PointCloudBuffer::Ptr> candidate_inlier_buffers;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> candidate_local_coeffs;

  Eigen::VectorXf coeffs;
};

class PlaneDetectionWindow {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PlaneDetectionWindow(std::shared_ptr<InteractiveGraphView>& graph);
  ~PlaneDetectionWindow();

  void show();
  void close();
  void set_center_point(const Eigen::Vector3f& point);

  RegionGrowingResult::Ptr region_growing(guik::ProgressInterface& progress);
  PlaneDetectionResult::Ptr detect_plane(const RegionGrowingResult::Ptr& region_growing);
  pcl::PointCloud<pcl::PointXYZI>::Ptr detect_plane_with_coeffs(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud, Eigen::Vector4f& coeffs);

  void draw_ui();
  void draw_gl(glk::GLSLShader& shader);

private:
  bool show_window;
  std::shared_ptr<InteractiveGraphView>& graph;

  Eigen::Vector3f center_point;

  float initial_neighbor_search_radius;
  float normal_estimation_radius;
  int min_cluster_size;
  int max_cluster_size;
  int num_neighbors;
  float smoothness_threshold;
  float curvature_threshold;

  bool enable_normal_filtering;
  float normal_threshold;

  float ransac_distance_thresh;
  int min_plane_supports;

  int robust_kernel;
  float robust_kernel_delta;

  guik::ProgressModal region_growing_progress_modal;
  RegionGrowingResult::Ptr region_growing_result;
  PlaneDetectionResult::Ptr plane_detection_result;
};

}  // namespace hdl_graph_slam

#endif