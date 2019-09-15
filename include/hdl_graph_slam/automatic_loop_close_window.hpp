#ifndef AUTOMATIC_LOOP_CLOSE_WINDOW_HPP
#define AUTOMATIC_LOOP_CLOSE_WINDOW_HPP

#include <mutex>
#include <thread>
#include <memory>
#include <boost/optional.hpp>

#include <imgui.h>
#include <hdl_graph_slam/view/keyframe_view.hpp>
#include <hdl_graph_slam/view/interactive_graph_view.hpp>

namespace hdl_graph_slam {

class AutomaticLoopCloseWindow {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AutomaticLoopCloseWindow(InteractiveGraphView& graph, const std::string& data_directory);
  ~AutomaticLoopCloseWindow();

  void draw_ui();

  void draw_gl(glk::GLSLShader& shader);

  void show();

  void start();

  void close();

private:
  void loop_detection();
  std::vector<KeyFrameView::Ptr> find_loop_candidates(const KeyFrameView::Ptr& keyframe);

private:
  bool show_window;
  InteractiveGraphView& graph;

  std::mutex loop_detection_mutex;
  std::thread loop_detection_thread;

  std::atomic_bool running;
  int loop_detection_source;

  KeyFrameView::Ptr loop_source;
  std::vector<KeyFrameView::Ptr> loop_candidates;

  int scan_matching_method;
  float scan_matching_resolution;
  float fitness_score_thresh;
  float fitness_score_max_range;

  int search_method;
  float distance_thresh;
  float accum_distance_thresh;

  int robust_kernel;
  float robust_kernel_delta;
};

}

#endif
