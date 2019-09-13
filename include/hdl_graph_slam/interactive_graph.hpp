#ifndef HDL_GRAPH_SLAM_INTERACTIVE_GRAPH_HPP
#define HDL_GRAPH_SLAM_INTERACTIVE_GRAPH_HPP

#include <regex>
#include <unordered_map>

#include <Eigen/Dense>
#include <boost/format.hpp>

#include <hdl_graph_slam/keyframe.hpp>
#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/parameter_server.hpp>

namespace hdl_graph_slam {

class InformationMatrixCalculator;

class InteractiveGraph : protected GraphSLAM {
public:
  InteractiveGraph();
  virtual ~InteractiveGraph();

  bool load_map_data(const std::string& directory);

  g2o::EdgeSE3* add_edge(const KeyFrame::Ptr& key1, const KeyFrame::Ptr& key2, const Eigen::Isometry3d& relative_pose, const std::string& robust_kernel="NONE", double robust_kernel_delta=0.1);

  void optimize();

  using GraphSLAM::num_edges;
  using GraphSLAM::num_vertices;

private:
  bool load_keyframes(const std::string& directory);

public:
  ParameterServer params;

  int iterations;
  double chi2_before;
  double chi2_after;
  double elapsed_time_msec;

  std::unordered_map<long, KeyFrame::Ptr> keyframes;

  std::unique_ptr<InformationMatrixCalculator> inf_calclator;
};

}

#endif