#ifndef HDL_GRAPH_SLAM_INTERACTIVE_EDGE_HPP
#define HDL_GRAPH_SLAM_INTERACTIVE_EDGE_HPP

#include <hdl_graph_slam/graph_slam.hpp>

class InteractiveEdge {
public:
  InteractiveEdge() {}
  virtual ~InteractiveEdge() {}

private:
  InteractiveEdge();

private:
  g2o::HyperGraph::Edge* edge;
};

#endif
