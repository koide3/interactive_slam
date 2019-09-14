#include <hdl_graph_slam/interactive_graph.hpp>


#include <chrono>
#include <boost/filesystem.hpp>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/core/sparse_optimizer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/information_matrix_calculator.hpp>


namespace hdl_graph_slam {

  InteractiveGraph::InteractiveGraph()
  : GraphSLAM("lm_var"),
  iterations(0),
  chi2_before(0.0),
  chi2_after(0.0),
  elapsed_time_msec(0.0)
  {
    inf_calclator.reset(new InformationMatrixCalculator());
    inf_calclator->load(params);
  }

  InteractiveGraph::~InteractiveGraph() {

  }

  bool InteractiveGraph::load_map_data(const std::string& directory) {
    if(!load(directory + "/graph.g2o")) {
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

      InteractiveKeyFrame::Ptr keyframe = std::make_shared<InteractiveKeyFrame>(keyframe_dir, graph.get());
      if(!keyframe->node) {
        std::cerr << "error : failed to load keyframe!!" << std::endl;
        std::cerr << "      : " << keyframe_dir << std::endl;
        return false;
      }

      keyframes[keyframe->id()] = keyframe;
    }

    return true;
  }

  g2o::EdgeSE3* InteractiveGraph::add_edge(const KeyFrame::Ptr& key1, const KeyFrame::Ptr& key2, const Eigen::Isometry3d& relative_pose, const std::string& robust_kernel, double robust_kernel_delta) {
    Eigen::MatrixXd inf = inf_calclator->calc_information_matrix(key1->cloud, key2->cloud, relative_pose);
    g2o::EdgeSE3* edge = add_se3_edge(key1->node, key2->node, relative_pose, inf);

    if(robust_kernel != "NONE") {
      add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
    }

    return edge;
  }

  void InteractiveGraph::optimize() {
    g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());
    auto t1 = std::chrono::high_resolution_clock::now();

    chi2_before = graph->chi2();
    iterations = GraphSLAM::optimize(params.param<int>("g2o_solver_num_iterations", 64));
    chi2_after = graph->chi2();

    auto t2 = std::chrono::high_resolution_clock::now();
    elapsed_time_msec = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
  }

  bool InteractiveGraph::save_pointcloud(const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated(new pcl::PointCloud<pcl::PointXYZI>());
    for(const auto& keyframe: keyframes) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::transformPointCloud(*keyframe.second->cloud, *transformed, keyframe.second->node->estimate().cast<float>());

      std::copy(transformed->begin(), transformed->end(), std::back_inserter(accumulated->points));
    }

    accumulated->is_dense = false;
    accumulated->width = accumulated->size();
    accumulated->height = 1;

    return pcl::io::savePCDFileBinary(filename, *accumulated);
  }
}