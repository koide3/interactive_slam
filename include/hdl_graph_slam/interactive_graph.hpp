#ifndef HDL_GRAPH_SLAM_INTERACTIVE_GRAPH_HPP
#define HDL_GRAPH_SLAM_INTERACTIVE_GRAPH_HPP

#include <regex>
#include <vector>
#include <Eigen/Dense>
#include <boost/format.hpp>

#include <hdl_graph_slam/keyframe.hpp>
#include <hdl_graph_slam/parameter_server.hpp>

namespace hdl_graph_slam {

class GraphSLAM;
class InformationMatrixCalculator;

class InteractiveGraph {
public:
  InteractiveGraph();
  ~InteractiveGraph();

  bool load(const std::string& directory);

private:
  bool load_keyframes(const std::string& directory);

public:
  ParameterServer params;

  std::vector<KeyFrame::Ptr> keyframes;

  std::unique_ptr<GraphSLAM> graph_slam;
  std::unique_ptr<InformationMatrixCalculator> inf_calclator;
};

}

#endif