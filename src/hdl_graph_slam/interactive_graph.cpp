#include <hdl_graph_slam/interactive_graph.hpp>

#include <boost/filesystem.hpp>
#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/information_matrix_calculator.hpp>


namespace hdl_graph_slam {

  InteractiveGraph::InteractiveGraph() {
    graph_slam.reset(new GraphSLAM(params.param<std::string>("g2o_solver_type", "lm_var")));
    inf_calclator.reset(new InformationMatrixCalculator());
    inf_calclator->load(params);
  }

  InteractiveGraph::~InteractiveGraph() {

  }

  bool InteractiveGraph::load(const std::string& directory) {
    if(!graph_slam->load(directory + "/graph.g2o")) {
      return false;
    }

    if(!load_keyframes(directory)) {
      return false;
    }

    return true;
  }

  bool InteractiveGraph::load_keyframes(const std::string& directory) {
    for(int i = 0;; i++) {
      std::string keyframe_dir = (boost::format("%s/%06d") % directory % i).str();
      if(!boost::filesystem::is_directory(keyframe_dir)) {
        break;
      }

      KeyFrame::Ptr keyframe = std::make_shared<KeyFrame>(keyframe_dir, graph_slam->graph.get());
      if(!keyframe->node) {
        std::cerr << "error : failed to load keyframe!!" << std::endl;
        std::cerr << "      : " << keyframe_dir << std::endl;
        return false;
      }

      keyframes.push_back(keyframe);
    }

    return true;
  }


}