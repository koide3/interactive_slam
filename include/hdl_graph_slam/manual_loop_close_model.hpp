#ifndef MANUAL_LOOP_CLOSE_MODAL_HPP
#define MANUAL_LOOP_CLOSE_MODAL_HPP

#include <memory>
#include <future>
#include <boost/optional.hpp>

#include <imgui.h>
#include <guik/gl_canvas.hpp>
#include <hdl_graph_slam/view/keyframe_view.hpp>
#include <hdl_graph_slam/view/interactive_graph_view.hpp>

namespace hdl_graph_slam {

class ManualLoopCloseModal {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ManualLoopCloseModal(std::shared_ptr<InteractiveGraphView>& graph, const std::string& data_directory);
  ~ManualLoopCloseModal();

  bool set_begin_keyframe(int keyframe_id);

  bool set_end_keyframe(int keyframe_id);

  bool run();

  void close();

  void draw_gl(glk::GLSLShader& shader);

  void draw_canvas();

private:
  void update_fitness_score();

  void auto_align();

  void scan_matching();

private:
  std::shared_ptr<InteractiveGraphView>& graph;

  double fitness_score;
  KeyFrameView::Ptr begin_keyframe;
  KeyFrameView::Ptr end_keyframe;

  Eigen::Isometry3d begin_keyframe_pose;
  Eigen::Isometry3d end_keyframe_pose_init;
  Eigen::Isometry3d end_keyframe_pose;

  std::unique_ptr<guik::GLCanvas> canvas;

  std::atomic_int auto_alignment_progress;
  std::future<std::shared_ptr<Eigen::Isometry3d>> auto_alignment_result;

  int registration_method;

  float fpfh_normal_estimation_radius;
  float fpfh_search_radius;
  int fpfh_max_iterations;
  int fpfh_num_samples;
  int fpfh_correspondence_randomness;
  float fpfh_similarity_threshold;
  float fpfh_max_correspondence_distance;
  float fpfh_inlier_fraction;
  int scan_matching_method;
  float scan_matching_resolution;
};
}

#endif
